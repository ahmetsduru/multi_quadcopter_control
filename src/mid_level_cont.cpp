#include "../include/multi_quadcopter_control/mid_level_cont.h"
#include <cmath>
#include <algorithm>
#include <Eigen/Dense> 

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
} // namespace MidLevelNS

MidLevelController::MidLevelController() 
    : m_des_x(0.0), m_des_y(0.0), m_des_z(0.0), m_des_psi(0.0),
      m_current_x(0.0), m_current_y(0.0), m_current_z(0.0),
      m_current_phi(0.0), m_current_theta(0.0), m_current_psi(0.0),
      m_current_velocity_x(0.0), m_current_velocity_y(0.0), m_current_velocity_z(0.0),
      m_desired_velocity_x(0.0), m_desired_velocity_y(0.0), m_desired_velocity_z(0.0),
      m_prev_error_thrust_x(0.0), m_integral_thrust_x(0.0),
      m_prev_error_thrust_y(0.0), m_integral_thrust_y(0.0),
      m_prev_error_thrust_z(0.0), m_integral_thrust_z(0.0),
      m_prev_error_ref_phi(0.0), m_integral_ref_phi(0.0),
      m_prev_error_ref_theta(0.0), m_integral_ref_theta(0.0),
      m_desired_acceleration_x(0.0), m_desired_acceleration_y(0.0), m_desired_acceleration_z(0.0), 
      m_disturbance_x(0.0), m_disturbance_y(0.0), m_disturbance_z(0.0),
      m_zv_x(0.0), m_zv_y(0.0), m_zv_z(0.0),
      m_delta_f_x(0.0), m_delta_f_y(0.0), m_delta_f_z(0.0),
      m_last_thrust(0.0), m_filtered_thrust(0.0),
      m_last_cmd_x(0.0), m_last_cmd_y(0.0), m_last_cmd_z(0.0),
      m_filtered_cmd_x(0.0), m_filtered_cmd_y(0.0), m_filtered_cmd_z(0.0)
{
    // ROS Parametre sunucusundan parametrelerin çekilmesi
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

    // KADEMELİ GÜVENLİ BAŞLANGIÇ: İlk saniyede motor kapanmasını engellemek
    m_last_thrust = m_mass * 9.81;
    m_filtered_thrust = m_mass * 9.81; 
    
    m_last_cmd_z = m_mass * 9.81;
    m_filtered_cmd_z = m_mass * 9.81;

    // LESO / NDOB Kazanç Parametreleri (Bant genişliği rad/s cinsinden)
    m_nh.param("mid_level_controller/lv_x", m_lv_x, 2.0);
    m_nh.param("mid_level_controller/lv_y", m_lv_y, 2.0);
    m_nh.param("mid_level_controller/lv_z", m_lv_z, 2.5);

    // Subscriber / Publisher Tanımlamaları
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
    // SANAL KONTROL KOMUTU FİLTRESİ (Aktüatör Gecikmesini Taklit Etme)
    // ------------------------------------------------------------------------
    double tau_motor = 0.1; 
    double alpha = m_dt / (tau_motor + m_dt);
    
    m_filtered_cmd_x = (1.0 - alpha) * m_filtered_cmd_x + alpha * m_last_cmd_x;
    m_filtered_cmd_y = (1.0 - alpha) * m_filtered_cmd_y + alpha * m_last_cmd_y;
    m_filtered_cmd_z = (1.0 - alpha) * m_filtered_cmd_z + alpha * m_last_cmd_z;

    // ------------------------------------------------------------------------
    // MODEL-FREE GENİŞLETİLMİŞ DURUM GÖZLEMCİSİ (LESO) ADIMI
    // ------------------------------------------------------------------------
    // Hız kestirim hatası (Ölçülen Hız - Kestirilen Hız)
    double v_err_x = m_current_velocity_x - m_zv_x;
    double v_err_y = m_current_velocity_y - m_zv_y;
    double v_err_z = m_current_velocity_z - m_zv_z;

    // LESO Kazançları (LPD Formu: Bant genişliğine bağlı analitik atama)
    double beta_1_x = 2.0 * m_lv_x;     double beta_2_x = m_lv_x * m_lv_x;
    double beta_1_y = 2.0 * m_lv_y;     double beta_2_y = m_lv_y * m_lv_y;
    double beta_1_z = 2.0 * m_lv_z;     double beta_2_z = m_lv_z * m_lv_z;

    // Nominal Kontrol Kazancı (Model belirsizliğini absorbe eden ölçekleyici sabiti)
    double b0 = 1.0 / m_mass; 

    // Durum Türevlerinin Hesaplanması (Sistem ivme uzayında modellenmiştir)
    double dv_est_x = m_delta_f_x + b0 * m_filtered_cmd_x + beta_1_x * v_err_x;
    double dd_est_x = beta_2_x * v_err_x;

    double dv_est_y = m_delta_f_y + b0 * m_filtered_cmd_y + beta_1_y * v_err_y;
    double dd_est_y = beta_2_y * v_err_y;

    // Z ekseninde yerçekimi ivmesi (9.81 m/s^2) nominal durum olarak çıkarılır.
    double dv_est_z = m_delta_f_z + b0 * m_filtered_cmd_z - 9.81 + beta_1_z * v_err_z;
    double dd_est_z = beta_2_z * v_err_z;

    // Sayısal Entegrasyon (Euler Yöntemi)
    m_zv_x += dv_est_x * m_dt;
    m_delta_f_x += dd_est_x * m_dt; // m_delta_f artık dinamik "Bozucu İvme"dir (m/s^2)

    m_zv_y += dv_est_y * m_dt;
    m_delta_f_y += dd_est_y * m_dt;

    m_zv_z += dv_est_z * m_dt;
    m_delta_f_z += dd_est_z * m_dt;

    // ------------------------------------------------------------------------
    // KONTROL YASASI (ADRC / Modelden Bağımsız Dekuplaj)
    // ------------------------------------------------------------------------
    // PID çıktısı kuvvet (N) cinsinden olduğu için ivme birimine indirgenir (1/m).
    // Kestirilen toplam bozucu ivme (m_delta_f) doğrudan döngüden düşülür.
    
    double des_acc_x = (MidLevelNS::computePID(m_des_x, m_current_x, m_desired_velocity_x, m_current_velocity_x, m_integral_thrust_x, m_kp_thrust_x, m_ki_thrust_x, m_kd_thrust_x, m_dt, m_integral_min, m_integral_max) / m_mass) 
                       + m_desired_acceleration_x - m_delta_f_x;
                 
    double des_acc_y = (MidLevelNS::computePID(m_des_y, m_current_y, m_desired_velocity_y, m_current_velocity_y, m_integral_thrust_y, m_kp_thrust_y, m_ki_thrust_y, m_kd_thrust_y, m_dt, m_integral_min, m_integral_max) / m_mass) 
                       + m_desired_acceleration_y - m_delta_f_y;
                 
    double des_acc_z = (MidLevelNS::computePID(m_des_z, m_current_z, m_desired_velocity_z, m_current_velocity_z, m_integral_thrust_z, m_kp_thrust_z, m_ki_thrust_z, m_kd_thrust_z, m_dt, m_integral_min, m_integral_max) / m_mass) 
                       + m_desired_acceleration_z + 9.81 - m_delta_f_z;

    // Hesaplanan net sanal ivmeler, fiziksel motor komutuna (Newton) geri dönüştürülür.
    m_thrust_x = m_mass * des_acc_x;
    m_thrust_y = m_mass * des_acc_y;
    m_thrust_z = m_mass * des_acc_z;

    // Gelecek iterasyon için komut geçmişi güncellenir
    m_last_cmd_x = m_thrust_x;
    m_last_cmd_y = m_thrust_y;
    m_last_cmd_z = m_thrust_z;

    // ------------------------------------------------------------------------
    // TOPLAM İTKİ MİKTARINI HESAPLAMA
    // ------------------------------------------------------------------------
    double total_thrust = std::sqrt(m_thrust_x * m_thrust_x + m_thrust_y * m_thrust_y + m_thrust_z * m_thrust_z);
    
    if (m_thrust_z < 0.0) {
        total_thrust = m_min_thrust; 
    }

    m_last_thrust = applyThrustSaturation(total_thrust, m_min_thrust, m_max_thrust);
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

    // Doğrusal Olmayan (Exact) Dönüşüm
    double total_thrust = std::sqrt(m_thrust_x * m_thrust_x + m_thrust_y * m_thrust_y + m_thrust_z * m_thrust_z);
    
    // Asin argümanının [-1, 1] sınırlarında kalması için doyuma ulaştırma
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

    // Kestirilen yeni Model-Free bozucu ivme değerleri (m/s^2) ROS'a basılıyor
    geometry_msgs::Vector3 ndob_msg;
    ndob_msg.x = m_delta_f_x;
    ndob_msg.y = m_delta_f_y;
    ndob_msg.z = m_delta_f_z;
    m_ndob_disturbance_pub.publish(ndob_msg);
}

double MidLevelController::applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
    return std::max(min_thrust, std::min(thrust, max_thrust));
}

// ROS Callback Fonksiyonları
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