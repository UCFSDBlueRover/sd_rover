std_msgs/Header header
geometry_msgs/PoseStamped pose_target
geometry_msgs/Twist motors    # motor commands
std_msgs/Bool cancel     # cancels current actions
std_msgs/Bool shutdown   # if true, stop robot execution