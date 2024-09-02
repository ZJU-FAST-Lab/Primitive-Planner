python3 gen_position_swap.py $1 & sleep 2;

sleeptime=$(echo "scale=1; ($1*2)/10+5" | bc)
echo $sleeptime
roslaunch primitive_planner swarm.launch & sleep $sleeptime;

bash pub_trigger.sh & sleep 1;

wait;


