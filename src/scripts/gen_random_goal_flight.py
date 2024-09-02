def generate_launch_file(num_drones, init_x, init_y_spacing, output_file='random_goal_flight.launch'):
    with open(output_file, 'w') as file:
        file.write('<launch>\n\n')
        
        # 写入公共参数配置
        file.write('    <arg name="map_size_x" value="500.0"/>\n')
        file.write('    <arg name="map_size_y" value="500.0"/>\n')
        file.write('    <arg name="map_size_z" value="3.0"/>\n')
        file.write('    <arg name="odom_topic" value="visual_slam/odom" />\n\n')
        
        # 写入swarm_bridge的配置
        file.write('    <!-- swarm topic transmitter bridge-->\n')
        file.write('    <include file="$(find swarm_bridge)/launch/bridge_udp.launch">\n')
        file.write('        <arg name="drone_id" value="999"/>\n')
        file.write('        <arg name="broadcast_ip" value="127.0.0.255"/>\n')
        file.write('    </include>\n\n')
        
        # 写入map_generator的配置
        file.write('    <!-- map -->\n')
        file.write('    <node pkg="map_generator" name="random_forest" type="random_forest" output="screen">\n')
        file.write('        <param name="map/x_size" value="5" />\n')
        file.write('        <param name="map/y_size" value="5" />\n')
        file.write('        <param name="map/z_size" value="3" />\n')
        file.write('        <param name="map/resolution" value="0.1"/>\n')
        file.write('        <param name="seed" value="8"/>\n')
        file.write('        <param name="map/obs_num" value="16"/>\n')
        file.write('        <param name="ObstacleShape/lower_rad" value="0.2"/>\n')
        file.write('        <param name="ObstacleShape/upper_rad" value="0.6"/>\n')
        file.write('        <param name="ObstacleShape/lower_hei" value="0.0"/>\n')
        file.write('        <param name="ObstacleShape/upper_hei" value="3.0"/>\n')
        file.write('        <param name="map/circle_num" value="16"/>\n')
        file.write('        <param name="ObstacleShape/radius_l" value="0.7"/>\n')
        file.write('        <param name="ObstacleShape/radius_h" value="0.5"/>\n')
        file.write('        <param name="ObstacleShape/z_l" value="0.7"/>\n')
        file.write('        <param name="ObstacleShape/z_h" value="0.8"/>\n')
        file.write('        <param name="ObstacleShape/theta" value="0.5"/>\n')
        file.write('        <param name="pub_rate" value="1.0"/>\n')
        file.write('        <param name="min_distance" value="1.3"/>\n')
        file.write('    </node>\n\n')
        
        file.write('    <!-- <include file="$(find random_goals)/launch/random_goals.launch"/> -->\n\n')
        
        file.write('    <include file="$(find odom_visualization)/launch/run_vis_rotate.launch"/>\n\n')
    
        
        # 写入每架飞机的启动配置
        for i in range(num_drones):
            if num_drones % 2 == 0:
                # 如果飞机数量是偶数，中心飞机位于原点
                init_y = init_y_spacing * i - (num_drones - 1) * init_y_spacing / 2.0
            else:
                # 如果飞机数量是奇数，中心飞机稍微偏离原点
                init_y = init_y_spacing * i - (num_drones - 1) * init_y_spacing / 2.0 - init_y_spacing / 2.0
            file.write('    <!-- Drone {} -->\n'.format(i))
            file.write('    <include file="$(find primitive_planner)/launch/run_in_sim.xml">\n')
            file.write('        <arg name="drone_id" value="{}"/>\n'.format(i))
            file.write('        <arg name="init_x" value="{}"/>\n'.format(init_x))
            file.write('        <arg name="init_y" value="{}"/>\n'.format(init_y))
            file.write('        <arg name="init_z" value="1.0"/>\n')
            file.write('        <arg name="flight_type" value="1"/>\n')
            file.write('        <arg name="map_size_x" value="$(arg map_size_x)"/>\n')
            file.write('        <arg name="map_size_y" value="$(arg map_size_y)"/>\n')
            file.write('        <arg name="map_size_z" value="$(arg map_size_z)"/>\n')
            file.write('        <arg name="odom_topic" value="$(arg odom_topic)"/>\n')
            file.write('    </include>\n\n')
        
        # 写入XML尾部
        file.write('</launch>')

# 使用示例
num_drones = 20  # 假设我们需要启动5架飞机
init_x = -8.0
init_y_spacing = 1.0
generate_launch_file(num_drones, init_x, init_y_spacing)
