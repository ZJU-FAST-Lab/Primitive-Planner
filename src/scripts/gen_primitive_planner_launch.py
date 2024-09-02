#!/usr/bin/env python
# coding=utf-8

import ast
import os

def read_from_txt(file_path):
    square_starts = None
    square_goals = None

    with open(file_path, 'r') as file:
        lines = file.readlines()

        # 用ast.literal_eval来解析每一行，比eval更安全
        for line in lines:
            if "square_starts" in line:
                square_starts = ast.literal_eval(line.split('=')[1].strip())
            elif "square_goals" in line:
                square_goals = ast.literal_eval(line.split('=')[1].strip())

    return square_starts, square_goals

def generate_launch_content(starts, goals):
    launch_template = '''<launch>

    <arg name="map_size_x" value="50.0"/>
    <arg name="map_size_y" value="50.0"/>
    <arg name="map_size_z" value="6.0"/>
    <arg name="odom_topic" value="visual_slam/odom" />

    <!-- swarm topic transmitter bridge-->
    <include file="$(find swarm_bridge)/launch/bridge_udp.launch">
        <arg name="drone_id" value="999"/>
        <arg name="broadcast_ip" value="127.0.0.255"/>
    </include>

    <!-- map -->
    <node pkg="map_generator" name="random_forest" type="random_forest" output="screen">
        <param name="map/x_size" value="5" />
        <param name="map/y_size" value="5" />
        <param name="map/z_size" value="3" />
        <param name="map/resolution" value="0.1"/>
        <param name="seed" value="12"/>
        <param name="map/obs_num" value="16"/>
        <param name="ObstacleShape/lower_rad" value="0.2"/>
        <param name="ObstacleShape/upper_rad" value="0.6"/>
        <param name="ObstacleShape/lower_hei" value="0.0"/>
        <param name="ObstacleShape/upper_hei" value="3.0"/>
        <param name="map/circle_num" value="16"/>
        <param name="ObstacleShape/radius_l" value="0.7"/>
        <param name="ObstacleShape/radius_h" value="0.5"/>
        <param name="ObstacleShape/z_l" value="0.7"/>
        <param name="ObstacleShape/z_h" value="0.8"/>
        <param name="ObstacleShape/theta" value="0.5"/>
        <param name="pub_rate" value="1.0"/>
        <param name="min_distance" value="1.3"/>
    </node>
    
    {includes}
    '''
    
    includes_template = '''
    <include file="$(find primitive_planner)/launch/run_in_sim.xml">
        <arg name="drone_id" value="{drone_id}"/>
        <arg name="init_x" value="{init_x}"/>
        <arg name="init_y" value="{init_y}"/>
        <arg name="init_z" value="{init_z}"/>
        <arg name="target0_x" value="{target_x}"/>
        <arg name="target0_y" value="{target_y}"/>
        <arg name="target0_z" value="{target_z}"/>
        <arg name="map_size_x" value="$(arg map_size_x)"/>
        <arg name="map_size_y" value="$(arg map_size_y)"/>
        <arg name="map_size_z" value="$(arg map_size_z)"/>
        <arg name="odom_topic" value="$(arg odom_topic)"/>
    </include>
    '''

    includes_content = ""
    for drone_id, (start, goal) in enumerate(zip(starts, goals), start=0):
        includes_content += includes_template.format(
            drone_id=drone_id,
            init_x=start[0], init_y=start[1], init_z=start[2],
            target_x=goal[0], target_y=goal[1], target_z=goal[2]
        )

    launch_content = launch_template.format(includes=includes_content)
    return launch_content + "\n</launch>"

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = script_dir + '/../scripts/start_and_goals.txt'
    try:
        starts, goals = read_from_txt(file_path)
        launch_content = generate_launch_content(starts, goals)

        # 写入生成的ROS启动文件
        with open(script_dir + '/../planner/plan_manage/launch/primitive_swarm.launch', 'w') as launch_file:
            launch_file.write(launch_content)
        
        print("ROS launch file generated successfully.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()