#include "../include/multi_quadcopter_control/mid_level_cont.h"
#include <cmath>
#include <algorithm>
// Eğer Eigen başlığı .h dosyanızda yoksa buraya eklemeniz gerekebilir:
// #include <Eigen/Dense> 

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
      m_disturbance_x(0.0), m_disturbance_y(0.0), m_disturbance_z(0.0),
      m_zv_x(0.0), m_zv_y(0.0), m_zv_z(0.0),
      m_delta_f_x(0.0), m_delta_f_y(0.0), m_delta_f_z(0.0),
      m_last_thrust(0.0), m_filtered_thrust(0.0) 
      {

    // Parametrelerin çekilmesi
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

    // KADEMELİ GÜVENLİ BAŞLANGIÇ: İlk saniyede NDOB'un motor kapatmasını engellemek için hover dengesine eşitliyoruz
    m_last_thrust = m_mass * 9.81;
    m_filtered_thrust = m_mass * 9.81; 

    // NDOB Kazanç Parametreleri (Bant genişliği rad/s cinsinden)
    m_nh.param("mid_level_controller/lv_x", m_lv_x, 2.0);
    m_nh.param("mid_level_controller/lv_y", m_lv_y, 2.0);
    m_nh.param("mid_level_controller/lv_z", m_lv_z, 2.5);

    // Sub / Pub Tanımlamaları
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
    m_ndob_disturbance_pub = m_nh.advertise<geometry_msgs::Vector3>("ndob_estimated_disturbance", 10);
}

void MidLevelController::spin() {
    ros::Rate rate(1.0 / m_dt); 
    while (ros::ok()) {
        ros::spinOnce();

        double thrust = computeThrust();
        geometry_msgs::Vector3 ref_angles = computeReferenceAngles();
        publishControlSignals(thrust, ref_angles);

        rate.sleep();
    }
}

double MidLevelController::computeThrust() {
    // ------------------------------------------------------------------------
    // MOTOR DİNAMİĞİ FİLTRESİ (Aktüatör Gecikmesini Taklit Etme)
    // ------------------------------------------------------------------------
    double tau_motor = 0.05; 
    double alpha = m_dt / (tau_motor + m_dt);
    m_filtered_thrust = (1.0 - alpha) * m_filtered_thrust + alpha * m_last_thrust;

    double c_phi   = cos(m_current_phi);
    double s_phi   = sin(m_current_phi);
    double c_theta = cos(m_current_theta);
    double s_theta = sin(m_current_theta);
    double c_psi   = cos(m_current_psi);
    double s_psi   = sin(m_current_psi);

    // BODY-Z İTKİSİNİN DÜNYA EKSENİNE İZDÜŞÜMÜ (Observer için zorunludur)
    double f_thrust_world_x = m_filtered_thrust * (c_psi * s_theta * c_phi + s_psi * s_phi);
    double f_thrust_world_y = m_filtered_thrust * (s_psi * s_theta * c_phi - c_psi * s_phi);
    double f_thrust_world_z = m_filtered_thrust * (c_theta * c_phi);

    // ------------------------------------------------------------------------
    // MODEL-DESTEKLİ NDOB ADIMI (Bilinmeyen Bozucuları Kestirir)
    // ------------------------------------------------------------------------
    // KESTİRİM: Entegrasyondan ÖNCE 1 kez hesaplanır
    m_delta_f_x = m_zv_x + m_lv_x * m_mass * m_current_velocity_x;
    m_delta_f_y = m_zv_y + m_lv_y * m_mass * m_current_velocity_y;
    m_delta_f_z = m_zv_z + m_lv_z * m_mass * m_current_velocity_z;

    // TÜREV HESAPLAMASI (Gözlemci doğrudan m_filtered_thrust etkisini baz alıyor)
    double dz_x = - m_lv_x * m_delta_f_x - m_lv_x * f_thrust_world_x;
    double dz_y = - m_lv_y * m_delta_f_y - m_lv_y * f_thrust_world_y;
    double dz_z = - m_lv_z * m_delta_f_z - m_lv_z * (f_thrust_world_z - (m_mass * 9.81));

    // ENTEGRASYON (Euler)
    m_zv_x += dz_x * m_dt;
    m_zv_y += dz_y * m_dt;
    m_zv_z += dz_z * m_dt;

    // ------------------------------------------------------------------------
    // KONTROL YASASI VE NDOB DÜZELTMESİ (PID + İleri Besleme - Kestirilen Hata)
    // ------------------------------------------------------------------------
    m_thrust_x = MidLevelNS::computePID(m_des_x, m_current_x, m_desired_velocity_x, m_current_velocity_x, m_integral_thrust_x, m_kp_thrust_x, m_ki_thrust_x, m_kd_thrust_x, m_dt, m_integral_min, m_integral_max) 
                 + (m_mass * m_desired_acceleration_x) - m_delta_f_x;
                 
    m_thrust_y = MidLevelNS::computePID(m_des_y, m_current_y, m_desired_velocity_y, m_current_velocity_y, m_integral_thrust_y, m_kp_thrust_y, m_ki_thrust_y, m_kd_thrust_y, m_dt, m_integral_min, m_integral_max) 
                 + (m_mass * m_desired_acceleration_y) - m_delta_f_y;
                 
    m_thrust_z = MidLevelNS::computePID(m_des_z, m_current_z, m_desired_velocity_z, m_current_velocity_z, m_integral_thrust_z, m_kp_thrust_z, m_ki_thrust_z, m_kd_thrust_z, m_dt, m_integral_min, m_integral_max) 
                 + (m_mass * m_desired_acceleration_z) + (m_mass * 9.81) - m_delta_f_z;

    // ------------------------------------------------------------------------
    // DÜNYA EKSENİ KUVVETLERİNİ BODY EKSENİNE ÇEVİRME
    // ------------------------------------------------------------------------
    geometry_msgs::Vector3 ref_angles = computeReferenceAngles();
    double phi_ref = ref_angles.x;   
    double theta_ref = ref_angles.y; 
    double psi_ref = ref_angles.z;   

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 
        cos(psi_ref) * cos(theta_ref), sin(psi_ref) * cos(theta_ref), -sin(theta_ref),
        cos(psi_ref) * sin(theta_ref) * sin(phi_ref) - sin(psi_ref) * cos(phi_ref), sin(psi_ref) * sin(theta_ref) * sin(phi_ref) + cos(psi_ref) * cos(phi_ref), cos(theta_ref) * sin(phi_ref),
        cos(psi_ref) * sin(theta_ref) * cos(phi_ref) + sin(psi_ref) * sin(phi_ref), sin(psi_ref) * sin(theta_ref) * cos(phi_ref) - cos(psi_ref) * sin(phi_ref), cos(theta_ref) * cos(phi_ref);

    Eigen::Vector3d world_forces(m_thrust_x, m_thrust_y, m_thrust_z);
    
    // Uygulanabilecek tek itki (Z ekseni itkisi) çıkartılıyor
    Eigen::Vector3d body_forces = rotation_matrix * world_forces;
    m_last_thrust = applyThrustSaturation(body_forces.z(), m_min_thrust, m_max_thrust);
    
    return m_last_thrust;
}

geometry_msgs::Vector3 MidLevelController::computeReferenceAngles() {
    geometry_msgs::Vector3 ref_angles;

    // Tekillik (Singularity) Koruması
    if (std::abs(m_thrust_z) < 1e-5) {
        ref_angles.x = 0.0;
        ref_angles.y = 0.0;
        ref_angles.z = m_des_psi;
        return ref_angles;
    }

    // Doğrusal Olmayan (Exact) Dönüşüm: Küçük açı yaklaşımlarının sebep olduğu izleme hatalarını önler.
    double total_thrust = std::sqrt(m_thrust_x * m_thrust_x + m_thrust_y * m_thrust_y + m_thrust_z * m_thrust_z);
    
    // Asin argümanının [-1, 1] sınırlarında kaldığından emin olmak için koruma (NaN fırlatmasını önler)
    double sin_phi_val = (m_thrust_x * std::sin(m_current_psi) - m_thrust_y * std::cos(m_current_psi)) / total_thrust;
    sin_phi_val = std::max(-1.0, std::min(1.0, sin_phi_val));

    ref_angles.x = std::asin(sin_phi_val);
    ref_angles.y = std::atan2(m_thrust_x * std::cos(m_current_psi) + m_thrust_y * std::sin(m_current_psi), m_thrust_z);
    ref_angles.z = m_des_psi;

    return ref_angles;
}

void MidLevelController::publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles) {
    std_msgs::Float64 thrust_msg;
    thrust_msg.data = thrust;
    m_desired_thrust_pub.publish(thrust_msg);
    m_desired_angles_pub.publish(ref_angles);

    // Kestirilen Delta_f (Bozucu) değerleri ROS'a basılıyor
    geometry_msgs::Vector3 ndob_msg;
    ndob_msg.x = m_delta_f_x;
    ndob_msg.y = m_delta_f_y;
    ndob_msg.z = m_delta_f_z;
    m_ndob_disturbance_pub.publish(ndob_msg);
}

double MidLevelController::applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
    return std::max(min_thrust, std::min(thrust, max_thrust));
}

void MidLevelController::positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_des_x = msg->x; m_des_y = msg->y; m_des_z = msg->z;
}

void MidLevelController::currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_x = msg->x; m_current_y = msg->y; m_current_z = msg->z;
}

void MidLevelController::currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_phi = msg->x; m_current_theta = msg->y; m_current_psi = msg->z;
}

void MidLevelController::desiredVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_desired_velocity_x = msg->x; m_desired_velocity_y = msg->y; m_desired_velocity_z = msg->z;
}

void MidLevelController::currentVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_velocity_x = msg->x; m_current_velocity_y = msg->y; m_current_velocity_z = msg->z;
}

void MidLevelController::desiredAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_desired_acceleration_x = msg->x; m_desired_acceleration_y = msg->y; m_desired_acceleration_z = msg->z;
}

void MidLevelController::desiredPsiCallback(const std_msgs::Float64::ConstPtr& msg) {
    m_des_psi = msg->data;
}

void MidLevelController::disturbanceCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_disturbance_x = msg->x; m_disturbance_y = msg->y; m_disturbance_z = msg->z;
}