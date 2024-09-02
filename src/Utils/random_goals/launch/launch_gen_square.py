import sys
import numpy as np
import os
import math

script_dir = os.path.dirname(os.path.abspath(__file__))
fname = script_dir + "/../Utils/random_goals/launch/random_goals.launch"
def main(argv):
    height1 = 0.5
    height2 = 1.5
    width = 6.0
    goal_num_in_int = 24
    drone_num_in = 20

    upper_multiple = (goal_num_in_int + 3) // 4 * 4
    lower_multiple = goal_num_in_int // 4 * 4
    if (goal_num_in_int - lower_multiple) <= (upper_multiple - goal_num_in_int):
        goal_num_in = lower_multiple
    else:
        goal_num_in = upper_multiple
    space = width / (goal_num_in / 4)

    file = open(fname, "w")

    str_head = "<launch>\n\
    <node pkg=\"random_goals\" name=\"random_goals_node\" type=\"random_goals_node\" output=\"screen\">\n\
        <param name=\"drone_num\" value=\"{drone_num}\"/>\n\
        <param name=\"goal_num\" value=\"{goal_num}\"/>\n".format(goal_num=goal_num_in * 2, drone_num=drone_num_in)
    file.write(str_head)

    for i in range(goal_num_in // 4):
        str_goals = "         <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=8 * i + 0, p0= -width / 2, p1= width / 2 - space * i,  p2=height1)
        str_goals += "        <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=8 * i + 1, p0= width / 2, p1= -width / 2 + space * i,  p2=height1)
        str_goals += "        <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=8 * i + 2, p0= -width / 2 + space * i, p1= -width / 2, p2=height1)
        str_goals += "        <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=8 * i + 3, p0= width / 2 - space * i, p1= width / 2,   p2=height1)
        str_goals += "        <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=8 * i + 4, p0= -width / 2, p1= width / 2 - space * i,  p2=height2)
        str_goals += "        <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=8 * i + 5, p0= width / 2, p1= -width / 2 + space * i,  p2=height2)
        str_goals += "        <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=8 * i + 6, p0= -width / 2 + space * i, p1= -width / 2, p2=height2)
        str_goals += "        <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=8 * i + 7, p0= width / 2 - space * i, p1= width / 2,   p2=height2)
        file.write(str_goals)

    str_tail = "    </node>\n</launch>"
    file.write(str_tail)
    
    file.close()

if __name__ == '__main__':
    main(sys.argv)

