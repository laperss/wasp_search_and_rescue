trap "kill 0" SIGINT
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

sudo service mongodb stop
echo "Launch interfaced planning system"
roslaunch drone_planning start_planning_system.launch &
sleep 2s
echo "Add knowledge"
(cd "$parent_path/scripts/" && python add_knowledge_linnea.py &)
sleep 2s
echo "Launch rqt"
rqt --standalone rosplan_rqt.dispatcher.ROSPlanDispatcher &
sleep 1s
echo "Run planner"
rosservice call /kcl_rosplan/planning_server

wait
