#!/usr/bin/env python3

def generate_launch_file():
    # 5 Temel konfigürasyonun X koordinatları (Sizin orijinal dosyanızdaki map)
    # Config 1 -> X=0, Config 2 -> X=-2, vs.
    x_coords = {
        1: 0.0,
        2: -2.0,
        3: 2.0,
        4: 4.0,
        5: -4.0
    }

    # Dosya Başlığı
    xml_content = """<launch>
    <rosparam file="$(find multi_quadcopter_control)/config/trajectory_crescent.yaml" command="load"/>
    <rosparam file="$(find multi_quadcopter_control)/config/all_trajectories.yaml" command="load"/>
    <rosparam file="$(find multi_quadcopter_control)/config/trajectory_hover.yaml" command="load"/>
    <rosparam file="$(find multi_quadcopter_control)/config/trajectory_pid_tuning.yaml" command="load"/>
"""

    total_drones = 50
    
    for i in range(1, total_drones + 1):
        # 1'den 5'e kadar olan config dosyalarını döngüsel kullan
        # (i-1) % 5 + 1 işlemi sonucu her zaman 1,2,3,4,5 verir.
        config_id = (i - 1) % 5 + 1
        
        # Grid Yerleşimi Hesaplama
        # Her 5 drone bir satır oluşturur.
        row_number = (i - 1) // 5 
        
        # X koordinatı config ID'sine bağlı (sabit)
        pos_x = x_coords[config_id]
        
        # Y koordinatı satır numarasına göre artar (3 metre aralıklarla)
        # Örn: Drone 1-5 (Y=0), Drone 6-10 (Y=3), Drone 11-15 (Y=6)...
        pos_y = row_number * 3.0
        
        # Drone Bloğu
        drone_block = f"""
    <group ns="drone{i}">
        <rosparam file="$(find multi_quadcopter_control)/config/trajectory_manager_drone{config_id}.yaml" command="load"/>
        <rosparam file="$(find multi_quadcopter_control)/config/quadcopter_params_2.yaml" command="load"/>
        <param name="initial_x" value="{pos_x}"/>
        <param name="initial_y" value="{pos_y}"/>
        <param name="initial_z" value="0.0"/>
        <node name="waypoint_server" pkg="multi_quadcopter_control" type="multi_waypoint_server_node" output="screen" launch-prefix="nice -n -5"/>
        <node name="trajectory_generator" pkg="multi_quadcopter_control" type="multi_trajectory_generator_node" output="screen" launch-prefix="nice -n -5"/>
        <node name="mid_level_cont" pkg="multi_quadcopter_control" type="multi_mid_level_cont_node" output="screen" launch-prefix="nice -n -5"/>
        <node name="low_level_cont" pkg="multi_quadcopter_control" type="multi_low_level_cont_node" output="screen" launch-prefix="nice -n -5"/>
        <node name="eom" pkg="multi_quadcopter_control" type="multi_eom" output="screen" launch-prefix="nice -n -5"/>
        <node name="rviz_data_handler" pkg="multi_quadcopter_control" type="multi_rviz_data_handler" output="screen" launch-prefix="nice -n -5"/>
        <param name="robot_description" command="$(find xacro)/xacro $(find multi_quadcopter_control)/urdf/quad_model_v1.urdf" />
    </group>
"""
        # İlk drone için v2 modeli kullanılıyordu, onu koruyalım (Opsiyonel)
        if i == 1:
             drone_block = drone_block.replace("quad_model_v1.urdf", "quad_model_v2.urdf")
             drone_block = drone_block.replace("quadcopter_params_2.yaml", "quadcopter_params_1.yaml")

        xml_content += drone_block

    # Dosya Sonu
    xml_content += f"""
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find multi_quadcopter_control)/rviz/quad_rviz.rviz" output="screen"/>
    <node name="relative_disturbance_generator" pkg="multi_quadcopter_control" type="relative_disturbance_generator" output="screen" launch-prefix="nice -n -5"/>
    <param name="num_drones" value="{total_drones}" />
    <node name="data_logger" pkg="multi_quadcopter_control" type="multi_data_logger" output="screen" launch-prefix="nice -n -5"/>
</launch>
"""
    
    print(xml_content)

if __name__ == "__main__":
    generate_launch_file()