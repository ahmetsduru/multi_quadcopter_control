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
    // ROS Altyapısı
    ros::NodeHandle m_nh;

    ros::Subscriber m_desired_position_sub;
    ros::Subscriber m_current_position_sub;
    ros::Subscriber m_current_euler_sub;
    ros::Subscriber m_desired_velocity_sub;
    ros::Subscriber m_current_velocity_sub;
    ros::Subscriber m_desired_acceleration_sub;
    ros::Subscriber m_desired_psi_sub;

    ros::Publisher m_desired_thrust_pub;
    ros::Publisher m_desired_angles_pub;
    ros::Publisher m_ndob_disturbance_pub; // LESO tarafından kestirilen toplam bozucu ivme yayınlanır

    // Durum ve referans değişkenleri
    double m_des_x, m_des_y, m_des_z, m_des_psi;
    double m_current_x, m_current_y, m_current_z;
    double m_desired_velocity_x, m_desired_velocity_y, m_desired_velocity_z;
    double m_current_velocity_x, m_current_velocity_y, m_current_velocity_z;
    double m_current_phi, m_current_theta, m_current_psi;
    double m_desired_acceleration_x, m_desired_acceleration_y, m_desired_acceleration_z;
    double m_filtered_thrust;

    // PID kazançları ve sınırlandırma parametreleri
    double m_kp_thrust_x, m_ki_thrust_x, m_kd_thrust_x;
    double m_kp_thrust_y, m_ki_thrust_y, m_kd_thrust_y;
    double m_kp_thrust_z, m_ki_thrust_z, m_kd_thrust_z;
    double m_kp_phi, m_ki_phi, m_kd_phi;
    double m_kp_theta, m_ki_theta, m_kd_theta;
    double m_min_thrust, m_max_thrust;
    double m_integral_min, m_integral_max;
    double m_dt;

    // Nominal kütle parametresi.
    // Bu yapı tamamen model-free değildir; kütle ve yerçekimi nominal telafi için kullanılır.
    // Downwash kuvveti veya dış bozucu topic'i kontrolcüye verilmez.
    double m_mass;

    // ---- NOMİNAL MODEL DESTEKLİ LINEAR EXTENDED STATE OBSERVER (LESO) DEĞİŞKENLERİ ----
    // Gözlemci bant genişliği kazançları
    double m_lv_x, m_lv_y, m_lv_z;

    // LESO birinci durum kestirimleri: hız değerleri
    double m_zv_x, m_zv_y, m_zv_z;

    // LESO genişletilmiş durum kestirimleri: toplam bozucu ivmeler
    // Bu terimler fiziksel downwash kuvveti değildir.
    // Downwash, aerodinamik etkileşim, model hatası ve diğer belirsizliklerin birleşik etkisini temsil eder.
    double m_delta_f_x, m_delta_f_y, m_delta_f_z;

    // Aktüatör gecikme modeli ve sanal kontrol girdisi geçmiş hafızası
    double m_last_cmd_x, m_last_cmd_y, m_last_cmd_z;
    double m_filtered_cmd_x, m_filtered_cmd_y, m_filtered_cmd_z;

    // Sistem koruması için bir önceki iterasyona ait toplam itki kuvveti
    double m_last_thrust;

    // Kontrolör dahili durum değişkenleri
    double m_prev_error_thrust_x, m_integral_thrust_x;
    double m_prev_error_thrust_y, m_integral_thrust_y;
    double m_prev_error_thrust_z, m_integral_thrust_z;
    double m_prev_error_ref_phi, m_integral_ref_phi;
    double m_prev_error_ref_theta, m_integral_ref_theta;

    // Dünya eksen takımındaki sanal kontrol kuvvetleri
    double m_thrust_x;
    double m_thrust_y;
    double m_thrust_z;

    // Çekirdek fonksiyonlar
    double computeThrust();
    geometry_msgs::Vector3 computeReferenceAngles();
    void publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles);
    double applyThrustSaturation(double thrust, double min_thrust, double max_thrust);

    // ROS callback fonksiyonları
    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredPsiCallback(const std_msgs::Float64::ConstPtr& msg);
};

#endif // MID_LEVEL_CONTROLLER_H