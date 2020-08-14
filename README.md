Ozay Lab ROS package from controlling the MCity MKZ

Depends on packages of
sensor_msgs, nav_msgs : Can be found from ROS common messages
mcity_msg             : Can be found from shaobang

Process:
Creates one node "laptop" in scripts/mainNode.py
loads supporting modules from the python package include/vehicleControl

Supporting Modules High Level:

getData       :  GPS             -> Data
road          :  Data            -> States
steeringWheel :  Wheel Force     -> User Input
supervisor    :  States, Inputs  -> Supervised cmds
command       :  cmds            -> Car          
