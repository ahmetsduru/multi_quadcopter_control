#ifndef MID_LEVEL_CONTROLLER_H
#define MID_LEVEL_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace MidLevelNS {
    double computePID(double des_pos, double a_pos, double des_vel, double a_vel, double& integral,
                      double kp, double ki, double kd, double dt, double integral_min, double integral_max);
}

class MidLevelController {
public:
    MidLevelController();
    void spin();

private:
    // ROS Altyapısı (Node Handle, Publisher ve Subscriber Değişkenleri)
    ros::NodeHandle m_nh;
    ros::Subscriber m_desired_position_sub;
    ros::Subscriber m_current_position_sub;
    ros::Subscriber m_current_euler_sub;
    ros::Subscriber m_desired_velocity_sub;
    ros::Subscriber m_current_velocity_sub;
    ros::Subscriber m_desired_acceleration_sub;
    ros::Subscriber m_desired_psi_sub;
    ros::Subscriber m_disturbance_sub;

    ros::Publisher m_desired_thrust_pub;
    ros::Publisher m_desired_angles_pub;
    ros::Publisher m_thrust_components_pub;
    ros::Publisher m_ndob_disturbance_pub; // LESO/MFO kestirimlerini yayınlayan publisher

    // Durum ve Referans Değişkenleri (Kinematik ve Dinamik Seviye)
    double m_des_x, m_des_y, m_des_z, m_des_psi;
    double m_current_x, m_current_y, m_current_z;
    double m_desired_velocity_x, m_desired_velocity_y, m_desired_velocity_z;
    double m_current_velocity_x, m_current_velocity_y, m_current_velocity_z;
    double m_current_phi, m_current_theta, m_current_psi;
    double m_desired_acceleration_x, m_desired_acceleration_y, m_desired_acceleration_z;
    double m_disturbance_x, m_disturbance_y, m_disturbance_z;
    double m_filtered_thrust; // Aktüatör dinamik filtresi için durum değişkeni
    
    // PID Kazançları ve Sınırlandırma Parametreleri
    double m_kp_thrust_x, m_ki_thrust_x, m_kd_thrust_x;
    double m_kp_thrust_y, m_ki_thrust_y, m_kd_thrust_y;
    double m_kp_thrust_z, m_ki_thrust_z, m_kd_thrust_z;
    double m_kp_phi, m_ki_phi, m_kd_phi;
    double m_kp_theta, m_ki_theta, m_kd_theta;
    double m_min_thrust, m_max_thrust;
    double m_integral_min, m_integral_max;
    double m_dt;
    double m_mass; // Nominal kütle parametresi (b0 hesabı ve nihai ivme-kuvvet dönüşümü için)

    // ---- MODEL-FREE LINEAR EXTENDED STATE OBSERVER (LESO) DEĞİŞKENLERİ ----
    // Gözlemci bant genişliği kazançları (omega_o rad/s)
    double m_lv_x, m_lv_y, m_lv_z;

    // LESO Birinci Durum Kestirimleri: Hız değerleri (\hat{v}_x, \hat{v}_y, \hat{v}_z)
    double m_zv_x, m_zv_y, m_zv_z;

    // LESO Genişletilmiş Durum Kestirimleri: Toplam bozucu ivmeler (\hat{a}_{dist} -> m/s^2)
    // Model belirsizlikleri, yerçekimi sapmaları ve dış aerodinamik yüklerin tümünü kapsar.
    double m_delta_f_x, m_delta_f_y, m_delta_f_z;

    // Aktüatör gecikme modeli ve sanal kontrol girdisi (u) geçmiş hafızası
    double m_last_cmd_x, m_last_cmd_y, m_last_cmd_z;
    double m_filtered_cmd_x, m_filtered_cmd_y, m_filtered_cmd_z;

    // Sistem koruması için bir önceki iterasyona ait toplam itki kuvveti
    double m_last_thrust;
    // -----------------------------------------------------------------------

    // Kontrolör Dahili Durum Değişkenleri (Hata Entegratörleri)
    double m_prev_error_thrust_x, m_integral_thrust_x;
    double m_prev_error_thrust_y, m_integral_thrust_y;
    double m_prev_error_thrust_z, m_integral_thrust_z;
    double m_prev_error_ref_phi, m_integral_ref_phi;
    double m_prev_error_ref_theta, m_integral_ref_theta;

    // Dünya eksen takımındaki sanal kontrol kuvvetleri (Sanal İvme Girdileri)
    double m_thrust_x;
    double m_thrust_y;
    double m_thrust_z;

    // Çekirdek Fonksiyonlar (Matematiksel ve Algoritmik Süreçler)
    double computeThrust();
    geometry_msgs::Vector3 computeReferenceAngles();
    geometry_msgs::Vector3 computeThrustInBodyFrame(double thrust_x, double thrust_y, double thrust_z);
    void publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles);
    double applyThrustSaturation(double thrust, double min_thrust, double max_thrust);

    // ROS Mesaj Arayüzleri (Callback Fonksiyonları)
    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredPsiCallback(const std_msgs::Float64::ConstPtr& msg);
    void disturbanceCallback(const geometry_msgs::Vector3::ConstPtr& msg);
};

#endif // MID_LEVEL_CONTROLLER_H