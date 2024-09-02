#!/usr/bin/env python
# coding=utf-8
import sys
import copy
import random
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt
# import numpy as np

def write_to_txt(file_path, square_starts, square_goals):
    with open(file_path, 'w') as file:
        # 将square_starts转换为字符串并写入文件
        file.write(f"square_starts = {square_starts}\n")
        # 将square_goals转换为字符串并写入文件
        file.write(f"square_goals = {square_goals}\n")
        
def read_from_txt(file_path):
    square_starts = None
    square_goals = None

    with open(file_path, 'r') as file:
        lines = file.readlines()

        # 用eval来解析每一行，注意eval存在安全风险，实际使用时需谨慎
        for line in lines:
            if "square_starts" in line:
                square_starts = eval(line.split('=')[1].strip())
            elif "square_goals" in line:
                square_goals = eval(line.split('=')[1].strip())

    return square_starts, square_goals

def plot_square_starts_goals_3d(square_starts, square_goals):
    # 检查square_starts和square_goals的长度是否一致
    if len(square_starts) != len(square_goals):
        raise ValueError("The lengths of square_starts and square_goals must be the same.")
    
    # 提取所有点的X, Y, Z坐标
    starts_x = [p[0] for p in square_starts]
    starts_y = [p[1] for p in square_starts]
    starts_z = [p[2] for p in square_starts]
    
    goals_x = [p[0] for p in square_goals]
    goals_y = [p[1] for p in square_goals]
    goals_z = [p[2] for p in square_goals]
    
    # 创建一个新的图和子图
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制初始位置点
    ax.scatter(starts_x, starts_y, starts_z, c='blue', marker='o', label='Starts')
    
    # 绘制目标位置点
    ax.scatter(goals_x, goals_y, goals_z, c='red', marker='o', label='Goals')
    
    # 绘制连线
    for start, goal in zip(square_starts, square_goals):
        ax.plot([start[0], goal[0]], [start[1], goal[1]], [start[2], goal[2]], 'k-')
    
    # 设置图形属性
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    ax.set_title('3D Connections between Initial and Goal Positions')
    ax.legend()
    ax.grid(True)
    plt.show()

def shuffle_and_unmatch(square_starts):
    # 复制square_starts到square_starts_temp
    square_starts_temp = copy.deepcopy(square_starts)

    def is_adjacent(p1, p2):
        # 判断p1和p2是否相邻
        return (p1[0] == p2[0] and abs(p1[1] - p2[1]) == space) or \
            (p1[1] == p2[1] and abs(p1[0] - p2[0]) == space)

    def is_same_side(p1, p2):
        # 判断p1和p2是否在同一条边
        return (p1[0] == p2[0] and p1[1] != width / 2 and p2[1] != width / 2) or \
            (p1[1] == p2[1] and p1[0] != width / 2 and p2[0] != width / 2)

    def is_corner(p1):
        # 判断p1是否是角点
        return p1[0] == -width / 2 or p1[0] == width / 2 or p1[1] == -width / 2 or p1[1] == width / 2

    square_goals = []
    for i in range(len(square_starts)):
        p1 = square_starts[i]
        found = False
        for j in range(len(square_starts_temp)):
            p2 = square_starts_temp[j]
            if p1 == p2:
                continue
            if is_corner(p1) and is_corner(p2):
                if not is_adjacent(p1, p2):
                    square_goals.append(p2)
                    square_starts_temp.remove(p2)
                    found = True
                    break
            elif is_same_side(p1, p2):
                if not is_adjacent(p1, p2):
                    square_goals.append(p2)
                    square_starts_temp.remove(p2)
                    found = True
                    break
            elif not is_same_side(p1, p2):
                square_goals.append(p2)
                square_starts_temp.remove(p2)
                found = True
                break

        if not found:
            # 选择一个与p1不相等的点作为最终的p2
            remaining_points = [p for p in square_starts_temp if p[0] != p1[0] and p[1] != p1[1]  ]
            if remaining_points:
                p2 = random.choice(remaining_points)
                square_goals.append(p2)
                square_starts_temp.remove(p2)
                print("Not satisfied point: ", p1, p2)

    # print("Initial positions:", square_starts)
    # print("Goal positions:", square_goals)
    # plot_square_starts_goals_3d(square_starts, square_goals)

    return square_goals


if __name__ == '__main__':

    loop_number = int(sys.argv[1])
    width = 20.0

    if (loop_number == 8 or loop_number == 20):

        height1 = 1.0

        upper_multiple = (loop_number + 3) // 4 * 4
        lower_multiple = loop_number // 4 * 4
        if (loop_number - lower_multiple) <= (upper_multiple - loop_number):
            goal_num_in = lower_multiple
        else:
            goal_num_in = upper_multiple
        space = width / (goal_num_in / 4)

        print("space="+str(space))

        square_starts=[]
        for i in range(goal_num_in // 4):
            square_starts.append([-width / 2, width / 2 - space * i, height1])
            square_starts.append([width / 2, -width / 2 + space * i, height1])
            square_starts.append([-width / 2 + space * i, -width / 2, height1])
            square_starts.append([width / 2 - space * i, width / 2, height1])

        random.shuffle(square_starts)

        # square_goals = copy.deepcopy(square_starts)

        square_goals = shuffle_and_unmatch(square_starts)

        print(square_starts)
        print(square_goals)

    elif (loop_number == 40):

        height1 = 1.0
        height2 = 2.0

        loop_number = 20

        upper_multiple = (loop_number + 3) // 4 * 4
        lower_multiple = loop_number // 4 * 4
        if (loop_number - lower_multiple) <= (upper_multiple - loop_number):
            goal_num_in = lower_multiple
        else:
            goal_num_in = upper_multiple
        space = width / (goal_num_in / 4)

        square_starts=[]
        for i in range(goal_num_in // 4):
            square_starts.append([-width / 2, width / 2 - space * i,  height1])
            square_starts.append([width / 2, -width / 2 + space * i,  height1])
            square_starts.append([-width / 2 + space * i, -width / 2, height1])
            square_starts.append([width / 2 - space * i, width / 2,   height1])
            square_starts.append([-width / 2, width / 2 - space * i,  height2])
            square_starts.append([width / 2, -width / 2 + space * i,  height2])
            square_starts.append([-width / 2 + space * i, -width / 2, height2])
            square_starts.append([width / 2 - space * i, width / 2,   height2])

        # square_goals = copy.deepcopy(square_starts)

        random.shuffle(square_starts)

        square_goals = shuffle_and_unmatch(square_starts)

        print(square_starts)
        print(square_goals)

    elif (loop_number == 60):

        height1 = 1.0
        height2 = 2.0
        height3 = 3.0

        loop_number = 20

        upper_multiple = (loop_number + 3) // 4 * 4
        lower_multiple = loop_number // 4 * 4
        if (loop_number - lower_multiple) <= (upper_multiple - loop_number):
            goal_num_in = lower_multiple
        else:
            goal_num_in = upper_multiple
        space = width / (goal_num_in / 4)

        square_starts=[]
        for i in range(goal_num_in // 4):
            square_starts.append([-width / 2, width / 2 - space * i,  height1])
            square_starts.append([width / 2, -width / 2 + space * i,  height1])
            square_starts.append([-width / 2 + space * i, -width / 2, height1])
            square_starts.append([width / 2 - space * i, width / 2,   height1])
            square_starts.append([-width / 2, width / 2 - space * i,  height2])
            square_starts.append([width / 2, -width / 2 + space * i,  height2])
            square_starts.append([-width / 2 + space * i, -width / 2, height2])
            square_starts.append([width / 2 - space * i, width / 2,   height2])
            square_starts.append([-width / 2, width / 2 - space * i,  height3])
            square_starts.append([width / 2, -width / 2 + space * i,  height3])
            square_starts.append([-width / 2 + space * i, -width / 2, height3])
            square_starts.append([width / 2 - space * i, width / 2,   height3])

        # square_goals = copy.deepcopy(square_starts)

        random.shuffle(square_starts)

        square_goals = shuffle_and_unmatch(square_starts)

        print(square_starts)
        print(square_goals)

    elif (loop_number == 80):

        height1 = 1.0
        height2 = 2.0
        height3 = 3.0
        height4 = 4.0

        loop_number = 20

        upper_multiple = (loop_number + 3) // 4 * 4
        lower_multiple = loop_number // 4 * 4
        if (loop_number - lower_multiple) <= (upper_multiple - loop_number):
            goal_num_in = lower_multiple
        else:
            goal_num_in = upper_multiple
        space = width / (goal_num_in / 4)

        square_starts=[]
        for i in range(goal_num_in // 4):
            square_starts.append([-width / 2, width / 2 - space * i,  height1])
            square_starts.append([width / 2, -width / 2 + space * i,  height1])
            square_starts.append([-width / 2 + space * i, -width / 2, height1])
            square_starts.append([width / 2 - space * i, width / 2,   height1])
            square_starts.append([-width / 2, width / 2 - space * i,  height2])
            square_starts.append([width / 2, -width / 2 + space * i,  height2])
            square_starts.append([-width / 2 + space * i, -width / 2, height2])
            square_starts.append([width / 2 - space * i, width / 2,   height2])
            square_starts.append([-width / 2, width / 2 - space * i,  height3])
            square_starts.append([width / 2, -width / 2 + space * i,  height3])
            square_starts.append([-width / 2 + space * i, -width / 2, height3])
            square_starts.append([width / 2 - space * i, width / 2,   height3])
            square_starts.append([-width / 2, width / 2 - space * i,  height4])
            square_starts.append([width / 2, -width / 2 + space * i,  height4])
            square_starts.append([-width / 2 + space * i, -width / 2, height4])
            square_starts.append([width / 2 - space * i, width / 2,   height4])

        # square_goals = copy.deepcopy(square_starts)

        random.shuffle(square_starts)

        square_goals = shuffle_and_unmatch(square_starts)

        print(square_starts)
        print(square_goals)

    else:
        print("[ERROR] unsupported drone number")
        exit()

    # 写入文件
    write_to_txt('start_and_goals.txt', square_starts, square_goals)

    # 从文件读取
    # starts, goals = read_from_txt('start_and_goals.txt')
    # print("Read square_starts:", starts)
    # print("Read square_goals:", goals)