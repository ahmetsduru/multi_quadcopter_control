#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <cmath>

class Quadcopter {
public:
    Quadcopter(ros::NodeHandle& nh) {
        loadParameters(nh);
        initializeSubscribers(nh);
        initializePublishers(nh);
        initializeState();
    }

    void update() {
        updateThrust(thrust, thrust_actual, tau_thrust, dt);
        updateTorques(torques, torques_actual, tau_torque, dt);

        Eigen::Vector3d thrust_vec(0, 0, thrust_actual);

        using namespace boost::numeric::odeint;
        typedef runge_kutta_cash_karp54<std::vector<double>> error_stepper_type;
        typedef controlled_runge_kutta<error_stepper_type> controlled_stepper_type;

        integrate_adaptive(
            controlled_stepper_type(),
            [&](const std::vector<double>& x, std::vector<double>& dxdt, double t) {
                computeDerivatives(x, dxdt, thrust_vec, torques_actual, disturbance_force);
            },
            x, t, t + dt, dt
        );

        orthonormalizeRotation();
        t += dt;
        publishState();
    }

private:
    double m, rho, Cd_rot, tau_thrust, tau_torque, I_rotor, dt, t;
    Eigen::Matrix3d I, Cd_trans, A_trans;
    Eigen::Vector3d g, torques_actual, disturbance_force;
    std::vector<double> x;
    double thrust, thrust_actual;
    Eigen::Vector3d torques;
    ros::Subscriber thrust_sub, torques_sub, disturbance_sub;
    ros::Publisher pos_pub, euler_pub, velocity_pub, angular_velocity_pub, acceleration_pub;

    void loadParameters(ros::NodeHandle& nh) {
        std::vector<double> I_vec, g_vec, Cd_vec, A_vec;
        nh.getParam("state_derivative_solver_node/mass", m);
        nh.getParam("state_derivative_solver_node/I", I_vec);
        nh.getParam("state_derivative_solver_node/g", g_vec);
        nh.getParam("state_derivative_solver_node/dt", dt);
        nh.getParam("state_derivative_solver_node/tau_thrust", tau_thrust);
        nh.getParam("state_derivative_solver_node/tau_torque", tau_torque);
        nh.getParam("state_derivative_solver_node/rho", rho);
        nh.getParam("state_derivative_solver_node/A_trans", A_vec);
        nh.getParam("state_derivative_solver_node/Cd_trans", Cd_vec);

        I = vectorToMatrix3x3(I_vec);
        g = Eigen::Vector3d(g_vec.data());
        Cd_trans = vectorToMatrix3x3(Cd_vec);
        A_trans = vectorToMatrix3x3(A_vec);
    }

    Eigen::Matrix3d vectorToMatrix3x3(const std::vector<double>& vec) {
        Eigen::Matrix3d mat;
        for (int i = 0; i < 9; ++i)
            mat(i / 3, i % 3) = vec[i];
        return mat;
    }

    void initializeSubscribers(ros::NodeHandle& nh) {
        thrust_sub = nh.subscribe("reference_thrust", 10, &Quadcopter::thrustCallback, this);
        torques_sub = nh.subscribe("reference_torques", 10, &Quadcopter::torquesCallback, this);
        disturbance_sub = nh.subscribe("disturbance_force", 10, &Quadcopter::disturbanceCallback, this);
    }

    void initializePublishers(ros::NodeHandle& nh) {
        pos_pub = nh.advertise<geometry_msgs::Vector3>("actual_position", 10);
        euler_pub = nh.advertise<geometry_msgs::Vector3>("actual_euler_angles", 10);
        velocity_pub = nh.advertise<geometry_msgs::Vector3>("actual_velocity", 10);
        angular_velocity_pub = nh.advertise<geometry_msgs::Vector3>("actual_angular_velocity", 10);
        acceleration_pub = nh.advertise<geometry_msgs::Vector3>("actual_acceleration", 10);
    }

    void initializeState() {
        x = std::vector<double>(18, 0.0);
        x[3] = x[7] = x[11] = 1.0;
        thrust_actual = 0.0;
        torques_actual.setZero();
        disturbance_force.setZero();
        t = 0.0;
    }

    void computeDerivatives(const std::vector<double>& x, std::vector<double>& dxdt,
                            const Eigen::Vector3d& thrust_vec, const Eigen::Vector3d& tau,
                            const Eigen::Vector3d& disturbance) {
        dxdt.resize(18);
        Eigen::Matrix3d R;
        R << x[3], x[4], x[5],
             x[6], x[7], x[8],
             x[9], x[10], x[11];
        Eigen::Vector3d V(&x[12]), w(&x[15]);

        for (int i = 0; i < 3; ++i) dxdt[i] = V[i];
        Eigen::Matrix3d dR = R * skew(w);
        for (int i = 0; i < 9; ++i) dxdt[3 + i] = dR(i / 3, i % 3);

        Eigen::Vector3d drag = -0.5 * rho * (Cd_trans * A_trans * V.array().square().matrix());
        Eigen::Vector3d acc = (R * thrust_vec + drag + m * g + disturbance) / m;
        for (int i = 0; i < 3; ++i) dxdt[12 + i] = acc[i];

        Eigen::Vector3d dw = I.inverse() * (tau - skew(w) * I * w);
        for (int i = 0; i < 3; ++i) dxdt[15 + i] = dw[i];
    }

    void orthonormalizeRotation() {
        Eigen::Matrix3d R;
        R << x[3], x[4], x[5],
             x[6], x[7], x[8],
             x[9], x[10], x[11];
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();
        for (int i = 0; i < 9; ++i) x[3 + i] = R(i / 3, i % 3);
    }

    Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d s;
        s << 0, -v[2], v[1],
             v[2], 0, -v[0],
            -v[1], v[0], 0;
        return s;
    }

    void publishState() {
        publishVec(pos_pub, x[0], x[1], x[2]);
        publishVec(velocity_pub, x[12], x[13], x[14]);
        publishVec(angular_velocity_pub, x[15], x[16], x[17]);

        Eigen::Matrix3d R;
        R << x[3], x[4], x[5],
             x[6], x[7], x[8],
             x[9], x[10], x[11];
        publishVec(euler_pub,
            atan2(R(2,1), R(2,2)),
            -asin(R(2,0)),
            atan2(R(1,0), R(0,0)));

        Eigen::Vector3d V(&x[12]);
        Eigen::Vector3d drag = -0.5 * rho * (Cd_trans * A_trans * V.array().square().matrix());
        Eigen::Vector3d thrust_vec(0, 0, thrust_actual);
        Eigen::Vector3d acc = (R * thrust_vec + drag + m * g + disturbance_force) / m;
        publishVec(acceleration_pub, acc[0], acc[1], acc[2]);
    }

    void publishVec(ros::Publisher& pub, double x, double y, double z) {
        geometry_msgs::Vector3 msg;
        msg.x = x; msg.y = y; msg.z = z;
        pub.publish(msg);
    }

    void thrustCallback(const std_msgs::Float64::ConstPtr& msg) { thrust = msg->data; }
    void torquesCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        torques = Eigen::Vector3d(msg->x, msg->y, msg->z);
    }
    void disturbanceCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        disturbance_force = Eigen::Vector3d(msg->x, msg->y, msg->z);
    }

    void updateThrust(double desired, double& actual, double tau, double dt) {
        actual += (1.0 / tau) * (desired - actual) * dt;
    }

    void updateTorques(const Eigen::Vector3d& desired, Eigen::Vector3d& actual, double tau, double dt) {
        actual += (1.0 / tau) * (desired - actual) * dt;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_eom");
    ros::NodeHandle nh;
    Quadcopter quad(nh);
    double dt;
    nh.getParam("state_derivative_solver_node/dt", dt);
    ros::Rate rate(1.0/dt);
    while (ros::ok()) {
        quad.update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}