
#!/bin/bash
source ~/catkin_ws/devel/setup.bash
echo "killing controller node"
rosnode kill mybot_controller
echo "return to start"

i="0"

while [ $i -lt 100 ]
do


timeout 5s rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'



i=$[$i+1]
done

