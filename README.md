Ozay Lab ROS package from controlling the MCity MKZ

Depends on packages of \
sensor_msgs, nav_msgs : Can be found from ROS common messages \
mcity_msg             : Can be found from shaobang \

Scripts: \
record      : Records RTK data to cvs file                  \
raw2route   : Converts RTK data to route format             \
control     : Supervises the user input on a set RTK route  \

Supporting Modules High Level:

getData       :  GPS             -> Data             \
road          :  Data            -> States           \ 
steeringWheel :  Wheel Force     -> User Input       \
supervisor    :  States, Inputs  -> Supervised cmds  \
command       :  cmds            -> Car              \     
