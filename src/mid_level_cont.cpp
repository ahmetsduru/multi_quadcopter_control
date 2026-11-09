#include "../include/multi_quadcopter_control/mid_level_cont.h"
#include <cmath>
#include <algorithm>

namespace MidLevelNS {

double computePID(double des_pos, double a_pos, double des_vel, double a_vel, double& integral,
                  double kp, double ki, double kd, double dt, double integral_min, double integral_max) {
    double pos_error = des_pos - a_pos;
    double vel_error = des_vel - a_vel;
    
    integral += pos_error * dt;
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }

    return kp * pos_error + ki * integral + kd * vel_error;
}
}

MidLevelController::MidLevelController() 
    : m_des_x(0.0), m_des_y(0.0), m_des_z(0.0), m_des_psi(0.0),
      m_current_x(0.0), m_current_y(0.0), m_current_z(0.0),
      m_current_phi(0.0), m_current_theta(0.0), m_current_psi(0.0),
      m_prev_error_thrust_x(0.0), m_integral_thrust_x(0.0),
      m_prev_error_thrust_y(0.0), m_integral_thrust_y(0.0),
      m_prev_error_thrust_z(0.0), m_integral_thrust_z(0.0),
      m_prev_error_ref_phi(0.0), m_integral_ref_phi(0.0),
      m_prev_error_ref_theta(0.0), m_integral_ref_theta(0.0),
      m_desired_acceleration_x(0.0), m_desired_acceleration_y(0.0), m_desired_acceleration_z(0.0), 
      m_disturbance_x(0.0), m_disturbance_y(0.0), m_disturbance_z(0.0)
      {

    // Parameters loading from the parameter server
    m_nh.getParam("mid_level_controller/kp_thrust_x", m_kp_thrust_x);
    m_nh.getParam("mid_level_controller/ki_thrust_x", m_ki_thrust_x);
    m_nh.getParam("mid_level_controller/kd_thrust_x", m_kd_thrust_x);
    m_nh.getParam("mid_level_controller/kp_thrust_y", m_kp_thrust_y);
    m_nh.getParam("mid_level_controller/ki_thrust_y", m_ki_thrust_y);
    m_nh.getParam("mid_level_controller/kd_thrust_y", m_kd_thrust_y);
    m_nh.getParam("mid_level_controller/kp_thrust_z", m_kp_thrust_z);
    m_nh.getParam("mid_level_controller/ki_thrust_z", m_ki_thrust_z);
    m_nh.getParam("mid_level_controller/kd_thrust_z", m_kd_thrust_z);
    m_nh.getParam("mid_level_controller/kp_phi", m_kp_phi);
    m_nh.getParam("mid_level_controller/ki_phi", m_ki_phi);
    m_nh.getParam("mid_level_controller/kd_phi", m_kd_phi);
    m_nh.getParam("mid_level_controller/kp_theta", m_kp_theta);
    m_nh.getParam("mid_level_controller/ki_theta", m_ki_theta);
    m_nh.getParam("mid_level_controller/kd_theta", m_kd_theta);
    m_nh.getParam("mid_level_controller/dt", m_dt);
    m_nh.getParam("mid_level_controller/min_thrust", m_min_thrust);
    m_nh.getParam("mid_level_controller/max_thrust", m_max_thrust);
    m_nh.getParam("mid_level_controller/integral_min", m_integral_min);
    m_nh.getParam("mid_level_controller/integral_max", m_integral_max);
    m_nh.getParam("state_derivative_solver_node/mass", m_mass);

    // Subscribers and publishers initialization
    m_desired_position_sub = m_nh.subscribe("reference_position", 10, &MidLevelController::positionCallback, this);
    m_current_position_sub = m_nh.subscribe("actual_position", 10, &MidLevelController::currentPositionCallback, this);
    m_current_euler_sub = m_nh.subscribe("actual_euler_angles", 10, &MidLevelController::currentEulerCallback, this);
    m_desired_acceleration_sub = m_nh.subscribe("reference_acceleration", 10, &MidLevelController::desiredAccelerationCallback, this);
    m_desired_velocity_sub = m_nh.subscribe("reference_velocity", 10, &MidLevelController::desiredVelocityCallback, this);
    m_current_velocity_sub = m_nh.subscribe("actual_velocity", 10, &MidLevelController::currentVelocityCallback, this);
    m_desired_psi_sub = m_nh.subscribe("reference_psi", 10, &MidLevelController::desiredPsiCallback, this);
    m_disturbance_sub = m_nh.subscribe("disturbance_static", 10, &MidLevelController::disturbanceCallback, this);

    m_desired_thrust_pub = m_nh.advertise<std_msgs::Float64>("reference_thrust", 10);
    m_desired_angles_pub = m_nh.advertise<geometry_msgs::Vector3>("reference_euler_angles", 10);
}

void MidLevelController::spin() {
    ros::Rate rate(1 / m_dt); 
    while (ros::ok()) {
        ros::spinOnce();

        // ÖNCE: Dünya çerçevesindeki PID kuvvetlerini hesapla ve toplam Thrust'ı çıkar
        double thrust = computeThrust();
        
        // SONRA: Ayrıştırılmış kuvvetlere dayanarak temiz referans açıları hesapla
        geometry_msgs::Vector3 ref_angles = computeReferenceAngles();
        
        publishControlSignals(thrust, ref_angles);

        rate.sleep();
    }
}

double MidLevelController::computeThrust() {
    // Dünya çerçevesinde (World Frame) saf PID kuvvetlerinin hesaplanması
    m_thrust_x = MidLevelNS::computePID(m_des_x, m_current_x, m_desired_velocity_x, m_current_velocity_x, m_integral_thrust_x, m_kp_thrust_x, m_ki_thrust_x, m_kd_thrust_x, m_dt, m_integral_min, m_integral_max) + m_mass * m_desired_acceleration_x;
    m_thrust_y = MidLevelNS::computePID(m_des_y, m_current_y, m_desired_velocity_y, m_current_velocity_y, m_integral_thrust_y, m_kp_thrust_y, m_ki_thrust_y, m_kd_thrust_y, m_dt, m_integral_min, m_integral_max) + m_mass * m_desired_acceleration_y;
    m_thrust_z = MidLevelNS::computePID(m_des_z, m_current_z, m_desired_velocity_z, m_current_velocity_z, m_integral_thrust_z, m_kp_thrust_z, m_ki_thrust_z, m_kd_thrust_z, m_dt, m_integral_min, m_integral_max) + m_mass * m_desired_acceleration_z + m_mass * 9.81;

    // KESİN ÇÖZÜM: Eksenleri birbirinden tamamen ayırıyoruz (Decoupling).
    // Toplam thrust, sadece dikey eksendeki kuvvet ihtiyacının aktüel açılara bölünmesiyle bulunur.
    // Böylece yatay ivmelenmeler dikey eksendeki kararlılığı ASLA bozamaz.
    double cos_phi = std::cos(m_current_phi);
    double cos_theta = std::cos(m_current_theta);

    // Singularity (Bölme hatası) koruması: Drone takla atma sınırındaysa veya ters dönmüşse korumaya al.
    double projection_denominator = cos_phi * cos_theta;
    if (projection_denominator < 0.1) {
        projection_denominator = 0.1; 
    }

    // İrtifa korumalı net gövde thrust hesaplaması
    double total_thrust = m_thrust_z / projection_denominator;

    return applyThrustSaturation(total_thrust, m_min_thrust, m_max_thrust);
}

geometry_msgs::Vector3 MidLevelController::computeReferenceAngles() {
    geometry_msgs::Vector3 ref_angles;

    // KRİTİK GÜVENLİK SINIRI: m_thrust_z sıfıra veya negatife düşerse açı hesaplaması çöker (Payda sıfırlanır).
    // m_thrust_z değerini sanal olarak en az 1.0 Newton seviyesinde tutarak açıların patlamasını engelliyoruz.
    double safe_thrust_z = m_thrust_z;
    if (safe_thrust_z < 1.0) {
        safe_thrust_z = 1.0;
    }

    // Dünya eksenindeki hedef yatay kuvvetleri üretecek referans Euler açıları
    ref_angles.x = (m_thrust_x * std::sin(m_current_psi) - m_thrust_y * std::cos(m_current_psi)) / safe_thrust_z;
    ref_angles.y = (m_thrust_x * std::cos(m_current_psi) + m_thrust_y * std::sin(m_current_psi)) / safe_thrust_z;    
    ref_angles.z = m_des_psi;

    // Maksimum referans açı sınırlandırması (Açıların çok büyümesini engellemek için rasyonel sınır: ~35 derece)
    double angle_limit = 0.6; // Radyan cinsinden (~34.3 derece)
    ref_angles.x = std::max(-angle_limit, std::min(ref_angles.x, angle_limit));
    ref_angles.y = std::max(-angle_limit, std::min(ref_angles.y, angle_limit));

    return ref_angles;
}

void MidLevelController::publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles) {
    std_msgs::Float64 thrust_msg;
    thrust_msg.data = thrust;
    m_desired_thrust_pub.publish(thrust_msg);
    m_desired_angles_pub.publish(ref_angles);
}

double MidLevelController::applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
    return std::max(min_thrust, std::min(thrust, max_thrust));
}

void MidLevelController::positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_des_x = msg->x;
    m_des_y = msg->y;
    m_des_z = msg->z;
}

void MidLevelController::currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_x = msg->x;
    m_current_y = msg->y;
    m_current_z = msg->z;
}

void MidLevelController::currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_phi = msg->x;
    m_current_theta = msg->y;
    m_current_psi = msg->z;
}

void MidLevelController::desiredVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_desired_velocity_x = msg->x;
    m_desired_velocity_y = msg->y;
    m_desired_velocity_z = msg->z;
}

void MidLevelController::currentVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_velocity_x = msg->x;
    m_current_velocity_y = msg->y;
    m_current_velocity_z = msg->z;
}

void MidLevelController::desiredAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_desired_acceleration_x = msg->x;
    m_desired_acceleration_y = msg->y;
    m_desired_acceleration_z = msg->z;
}

void MidLevelController::desiredPsiCallback(const std_msgs::Float64::ConstPtr& msg) {
    m_des_psi = msg->data;
}

void MidLevelController::disturbanceCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_disturbance_x = msg->x;
    m_disturbance_y = msg->y;
    m_disturbance_z = msg->z;
}