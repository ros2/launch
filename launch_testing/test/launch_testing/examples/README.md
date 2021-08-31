# Launch testing examples

This directory contains examples to help the user get an idea of how to use ```launch_testing``` to write their own test cases. It contains the following examples :

* ```hello_world_launch_test.py``` : Demonstrates basic usage and code flow
* ```check_node_launch_test.py``` : There might be situations where nodes, once launched, take some time to actually start and we need to wait for the node to start to perform some action. We can simulate this using ```launch.actions.TimerAction```. This example shows one way to detect when a node has been launched. We delay the launch by 5 seconds, and wait for the node to start with a timeout of 8 seconds.
* ```check_msgs_launch_test.py``` : Consider a problem statement where you need to launch a node and check if messages are published on a particular topic. This example demonstrates how to do that, using the talker node from ```demo_nodes_cpp``` package. It uses the ```Event``` object to end the test as soon as the first message is received on the chatter topic, with a timeout of 5 seconds. It is recommended to keep the tests as short as possible.
* ```set_param_launch_test.py``` : Launch a node, set a parameter in it and check if that was successful.
* ```good_proc_launch_test.py``` : TODO : Add description
* ```parameters_launch_test.py``` : TODO : Add desscription
* ```ready_action_test.py```: TODO : Add description
* ```terminating_proc_launch_test.py```: TODO : Add description
* ```args_launch_test.py```: TODO : Add description
* ```context_launch_test.py```: TODO : Add description


