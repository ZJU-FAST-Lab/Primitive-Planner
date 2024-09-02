import numpy as np
import os
import sys

def generate_launch_file(start_num, end_num, total_num, radius, obs_density, output_file):
    
    obs_map_width = (radius - 1.0) / 1.414 * 2
    with open(output_file, 'w') as file:
        file.write('<launch>\n\n')
        file.write('    <arg name="map_size_x" value="{}"/>\n'.format((radius + 10.0) * 2))
        file.write('    <arg name="map_size_y" value="{}"/>\n'.format((radius + 10.0) * 2))
        file.write('    <arg name="map_size_z" value="3.0"/>\n')
        file.write('    <arg name="odom_topic" value="visual_slam/odom" />\n')
        file.write('\n')
        file.write('    <!-- swarm topic transmitter bridge-->\n')
        file.write('    <include file="$(find swarm_bridge)/launch/bridge_udp.launch">\n')
        file.write('    <arg name="drone_id" value="0"/>\n')
        file.write('    <arg name="broadcast_ip" value="127.0.0.255"/>\n')
        file.write('    </include> \n')
        file.write('\n')
        file.write('    <!-- map --> \n')
        file.write('	<node pkg ="map_generator" name ="random_forest" type ="random_forest" output = "screen">\n')
        file.write('        <param name="map/x_size"     value="{}"/>\n'.format(obs_map_width))
        file.write('        <param name="map/y_size"     value="{}"/>\n'.format(obs_map_width))
        file.write('        <param name="map/z_size"     value="3.0" />\n')
        file.write('        <param name="map/resolution" value="0.1"/>\n')
        file.write('        <param name="map/obs_num"    value="{}"/>\n'.format(obs_map_width * obs_map_width * obs_density / 2.0))
        file.write('        <param name="ObstacleShape/lower_rad" value="0.2"/>\n')
        file.write('        <param name="ObstacleShape/upper_rad" value="0.5"/>\n')
        file.write('        <param name="ObstacleShape/lower_hei" value="0.0"/>\n')
        file.write('        <param name="ObstacleShape/upper_hei" value="3.0"/>\n')
        file.write('        <param name="map/circle_num" value="{}"/>\n'.format(obs_map_width * obs_map_width * obs_density / 2.0))
        file.write('        <param name="ObstacleShape/radius_l" value="0.7"/>\n')
        file.write('        <param name="ObstacleShape/radius_h" value="0.5"/>\n')
        file.write('        <param name="ObstacleShape/z_l" value="0.7"/>\n')
        file.write('        <param name="ObstacleShape/z_h" value="0.8"/>\n')
        file.write('        <param name="ObstacleShape/theta" value="0.5"/>\n')
        file.write('        <param name="pub_rate"   value="1.0"/>\n')
        file.write('        <param name="min_distance" value="0.8"/>\n')
        file.write('    </node> \n')
        file.write('\n')
        file.write('    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find primitive_planner)/launch/{}.rviz" required="true" />\n'.format("verbose" if total_num <= 40 else "drone_1000"))
        file.write('\n')
    
        step = 2 * np.pi / total_num
        for i in range(start_num, end_num + 1):
            angle = i * step
            init_x = np.cos(angle) * radius
            init_y = np.sin(angle) * radius
            file.write('    <include file="$(find primitive_planner)/launch/run_in_sim.xml">\n')
            file.write('        <arg name="drone_id"   value="{}"/>\n'.format(i))
            file.write('        <arg name="init_x"     value="{}"/>\n'.format(init_x))
            file.write('        <arg name="init_y"     value="{}"/>\n'.format(init_y))
            file.write('        <arg name="init_z"     value="0.5"/>\n')
            file.write('        <arg name="target0_x"   value="{}"/>\n'.format(-init_x))
            file.write('        <arg name="target0_y"   value="{}"/>\n'.format(-init_y))
            file.write('        <arg name="target0_z"   value="0.5"/>\n')
            file.write('        <arg name="map_size_x" value="$(arg map_size_x)"/>\n')
            file.write('        <arg name="map_size_y" value="$(arg map_size_y)"/>\n')
            file.write('        <arg name="map_size_z" value="$(arg map_size_z)"/>\n')
            file.write('        <arg name="odom_topic" value="$(arg odom_topic)"/>\n')
            file.write('    </include>\n')
        
        file.write('</launch>')
        
        
if __name__ == "__main__":
        
    if len(sys.argv) >= 2:
        drone_num = int(sys.argv[1])
    else:
        drone_num = 20
    radius = drone_num / 2.0 / np.pi * 2.0
    launch_dir = os.path.dirname(os.path.abspath(__file__)) + '/../planner/plan_manage/launch/'
    generate_launch_file(0, drone_num - 1, drone_num, radius, 1.0, launch_dir + 'swarm.launch')
