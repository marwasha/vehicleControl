Ozay Lab ROS package from controlling the MCity MKZ.

Implimented using rospy on python 3.8.5

Depends on ROS packages of

sensor_msgs, nav_msgs : Can be found from ROS common messages \
mcity_msg             : Can be found from shaobang

Depends on python3 packages of

evdev       : Feedback control of steeringwheel                       \
numpy       : For matrix math                                         \
threading   : To allow for opperation concurency for speed            \
pygame      : For joystick input and potential realtime graphics      \
rospy       : To commincate with ros                                  \
csv         : To read/write from csv files                            \
pymap3d     : To convert latitude and longitude data                  \
scipy       : For optimization and to read mat files                  \
json        : To be able to pretty print python dictonarys            \
itertools   : To be able find permutations


These packages should be preinstalled on the lab laptop and otherwise can be installed on a local machine using pip3. The one that requires extra work is pygame but instructions can be found online

Further Documentation can be found in each script. Hear is a high level overview

Scripts High Level:

record      : Records RTK data to cvs file                  \
raw2route   : Converts RTK data to route format             \
control     : Supervises the user input on a set RTK route  

Supporting Modules High Level:

getData       :  GPS             -> Data             \
road          :  Data            -> States           \
steeringWheel :  Wheel Force     -> User Input       \
supervisor    :  States, Inputs  -> Supervised cmds  \
command       :  cmds            -> Car              

Note these modules are implimented using a python3 package. I'm still figuring out how to fully use them properly, but this is how it was recommended online
