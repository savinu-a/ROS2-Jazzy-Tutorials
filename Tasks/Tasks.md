# Tasks

Solve the following tasks using ROS 2 and provide the solutions in the subfolders given for each task

## Task 1

Write a node `pose_re_pub` that will subscribe to topic `pose_with_covariance_stamped` where type is  `PoseWithCovarianceStamped` and publishes to new topic pose, with type Pose from `geometry_msgs`, values to `Pose` message comes from original message.

## Task 2

Make a ros node called `params_setter` that sets a global ros parameter from variable `param` with value from variable `value`

## Task 3

There is a file called `values.csv` where is twist values in one line is linear x y z and angular x y z like this `0.4,0.2, 0.0, 0.0, 0.0, 0.1` Make a node called `twist_from_database` and in the node there is a publisher that publishes to topic `twist_from_database` a twist message type from `geometry_msgs/Twist` twist with `queue_size` of `20` and `rate` of `10hz`. Read file a line by line, and publish values to topic

## Task 4

Write a node called `zero_twist` that will subscribe to topic `is_stopped`. Publish zero message to twist topic using `geometry_msgs/Twist` message, (meaning that all values of message is 0.0), as long you get string `true` from is_stopped topic, type `std_msgs/String`, if the message is false, or there is no message in topic, don't publish anything. The `queue_size` should be `10` 

## Task 5

Write a launch file that:

Start node `chatter.py` from package `tests`, output should go to terminal, and you must `remap` the topic from `/chatter` topic to `/test/chatter`, node name should be `test_chatter`

Start node `listener.py` from package tests, output should go to terminal, node name should be `listener`

Also from launch file publish one message to topic `twist`, with type `geometry_msgs/Twist`, and values `linear 0 0 0` and `angular 0 0 0`

Hint! You can take a look for example this(https://answers.ros.org/question/265816/how-to-publish-a-complex-msg-via-launch-file/)
Hint2!  publish only one message to topic twist

## Task 6

Make a ROS node called `param_reader` that will read parameter from `/robot_name` and prints that to terminal



