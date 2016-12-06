trap "kill 0" SIGINT

sudo service mongodb stop
echo "Launch interfaced planning system"
roslaunch kth2_planning interfaced_planning_system_linnea.launch &
sleep 2s
echo "Add knowledge"
(cd /home/laperss/catkin_ws/src/ascourse/planning/scripts/ && python add_knowledge_linnea.py &)
sleep 2s
echo "Launch rqt"
rqt --standalone rosplan_rqt.dispatcher.ROSPlanDispatcher &
sleep 2s
echo "Run planner"
rosservice call /kcl_rosplan/planning_server

wait
