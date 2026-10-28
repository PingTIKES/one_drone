Vendored from ZJU-FAST-Lab/ego-planner-swarm, ros2_version branch, commit 23a8d5a191711dd65633df689bd00f55d4dea8f9.

This copy is adapted for one drone: the active pipeline uses ROS goal input,
a depth occupancy grid, B-spline planning and trajectory output. Fleet exchange,
preset waypoint sequencing and unused moving-object prediction are excluded;
waypoint missions are owned by decision_manager.
