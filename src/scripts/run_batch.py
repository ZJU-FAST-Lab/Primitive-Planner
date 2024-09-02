import subprocess
import time
import os
import signal
import threading

def start_process(name, cmd):
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(f"Started process: {name}")
    # 定义一个线程来读取进程的输出
    thread = threading.Thread(target=print_output, args=(name, process))
    thread.start()
    return process

def print_output(name, process):
    # 持续读取进程的输出直到进程结束
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            print(f"{name} output: {output.strip()}")

def send_sigint(process):
    print(f"Sending SIGINT to process: {process.pid}")
    os.kill(process.pid, signal.SIGINT)

def simulate_input(process):
    print(f"Simulating input to process: {process.pid}")
    os.system(f"echo -n '\n' | nc {process.pid} 0")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    num_executions = 30
    timeout_seconds = 50 # 35, 40, 45, 50, 50
    num_drones = 80
    processes = []  # 存储所有进程的列表
    
    
    for _ in range(num_executions):
        core_process = start_process("roscore", "roscore")
        time.sleep(4)
        # Start the first script
        gen_start_goal_process = start_process("gen_start_goal.py", "python3 gen_start_goal.py " + str(num_drones))
        
        # Wait for 5 seconds before starting the other two scripts
        time.sleep(1)
        gen_ego_planner_launch_process = start_process("gen_primitive_planner_launch.py", "python3 gen_primitive_planner_launch.py")
        
        time.sleep(1)
        run_ego_planner_process = start_process("primitive_swarm.launch", "roslaunch primitive_planner primitive_swarm.launch")
        # rviz_process = start_process("rviz.launch", "roslaunch primitive_planner rviz.launch")
        time.sleep(20) # 5, 8, 12, 14, 20
        # record_odom_process = start_process("rosbag", "cd " + script_dir + "/../../bags_and_logs &&  bash " + script_dir + "/./scripts/record_odom.sh")
        # time.sleep(6)
        pub_trigger_process = start_process("pub_trigger", "bash " + script_dir + "/../scripts/pub_trigger.sh")
        
        # Wait for 60 seconds before sending SIGINT to record_odom.sh
        time.sleep(timeout_seconds)
        send_sigint(run_ego_planner_process)
        # send_sigint(rviz_process)
        # send_sigint(record_odom_process)
        
        # Wait for another 5 seconds before sending SIGINT to the other two scripts
        time.sleep(1)
        # send_sigint(record_odom_process)
        
        # 模拟输入以结束run_rmader.py
        # simulate_input(run_rmader_process)

        # 等待所有进程结束
        # for process in processes:
        #     process.wait()

        # # 清空进程列表，为下一次迭代做准备
        # processes.clear()
        kill_ros_process = start_process("pkill ros", "rosnode kill -a")
        time.sleep(35) # 5, 10, 18, 26, 35
        kill_all_process = start_process("pkill all", "pkill ros")
        kill_all_process = start_process("pkill all", "pkill primitive_plann")
        time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:    
        kill_ros_process = start_process("pkill ros", "rosnode kill -a")
        time.sleep(20)
        kill_all_process = start_process("pkill all", "pkill ros")
        kill_all_process = start_process("pkill all", "pkill primitive_plann")
        time.sleep(10)
    
    kill_ros_process = start_process("pkill ros", "rosnode kill -a")
    time.sleep(20)
    kill_all_process = start_process("pkill all", "pkill ros")
    kill_all_process = start_process("pkill all", "pkill primitive_plann")
    time.sleep(10)