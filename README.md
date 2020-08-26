Ozay Lab ROS package from controlling the MCity MKZ

Depends on packages of
sensor_msgs, nav_msgs : Can be found from ROS common messages
mcity_msg             : Can be found from shaobang

Scripts:
record      : Records RTK data to cvs file                  \n
raw2route   : Converts RTK data to route format             \n
control     : Supervises the user input on a set RTK route  \n

Supporting Modules High Level:

getData       :  GPS             -> Data              \n
road          :  Data            -> States            \n
steeringWheel :  Wheel Force     -> User Input        \n
supervisor    :  States, Inputs  -> Supervised cmds   \n
command       :  cmds            -> Car               \n
