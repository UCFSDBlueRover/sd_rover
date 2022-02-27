# std_msgs/Header header
geometry_msgs/Point target
# geometry_msgs/Twist motors  # motor commands
std_msgs/Bool start         # if true, Rover exits BOOT state (if all is well)
std_msgs/Bool cancel        # if true, cancels current actions
std_msgs/Bool shutdown      # if true, stop robot execution
std_msgs/Bool rc_preempt    # if true, preempts current actions with motor commands