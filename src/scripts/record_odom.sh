# #!/bin/bash

# # 定义录制的起始编号和结束编号
# START_NUM=1
# END_NUM=8

# # 初始化一个空的topics字符串
# topics=""

# # 循环构建所有topic名称并添加到topics字符串中
# for (( i=$START_NUM; i<=$END_NUM; i++ ))
# do
#   # 确保i是两位数，如果是个位数，则前面补0
#   printf -v formatted_i "%02d" "$i"
  
#   # 构建topic名称
#   topic="/SQ${formatted_i}s/rmader/actual_odom"
  
#   # 将topic添加到topics字符串中，用空格分隔
#   topics+=" $topic"
# done

# # 打印开始录制的提示信息
# echo "Starting to record topics to $BAG_PATH with topics: $topics"

# # 录制bag文件，一次性添加所有topic
# rosbag record $topics

# echo "Recording completed for all topics."

#!/bin/bash

# 定义录制的起始编号和结束编号
START_NUM=0
END_NUM=80

# 初始化一个空的topics字符串
topics=""

# 循环构建所有topic名称并添加到topics字符串中
for (( i=$START_NUM; i<=$END_NUM; i++ ))
do
  
  # 构建topic名称
  topic="/drone_${i}_visual_slam/odom"
  
  # 将topic添加到topics字符串中，用空格分隔
  topics+=" $topic"
done

# 打印开始录制的提示信息
echo "Starting to record topics to $BAG_PATH with topics: $topics"

# 录制bag文件，一次性添加所有topic
rosbag record $topics

echo "Recording completed for all topics."