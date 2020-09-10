#!/usr/local/bin/python3.8
import rospy
import numpy as np
import threading
import csv
from mcity_msg.msg import Control
from dbw_mkz_msgs.msg import SteeringReport

# we wannt a loop that sends 0 for a second than a step to a setpoint

REF = np.pi/2
HZ = 50

class steeringData:
    wheelAngle = 0
    def __init__(self):
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.update)

    def update(self, data):
        self.lock.acquire()
        self.wheelAngle = data.steering_wheel_angle
        self.lock.release()

    def get(self):
        self.lock.acquire()
        temp = self.wheelAngle
        self.lock.release()
        return temp

    def __str__(self):
        return str(self.get())


def run():
    rospy.init_node('StepTest', anonymous=True)
    pub = rospy.Publisher('/mkz_bywire_intf/control', Control, queue_size=10)
    toSave = {"time": 0, "Wheel": 0, "Cmd": 0}
    csv_columns = list(toSave.keys())
    csv_file = "/home/laptopuser/mkz/data/tests/StepInput.csv"
    print("saving results to " + csv_file)
    csvfile = open(csv_file, 'w')
    writer = csv.DictWriter(csvfile,
                            fieldnames=csv_columns,
                            quoting=csv.QUOTE_NONNUMERIC)
    writer.writeheader()
    wheel = steeringData()
    rate = rospy.Rate(HZ)
    control = Control();
    control.brake_cmd = 0.01
    print("starting loop")
    startTime = rospy.get_time()
    while not rospy.is_shutdown():
        control.timestamp = rospy.get_time() - startTime
        pub.publish(control)
        toSave["time"] = control.timestamp
        toSave["Wheel"] = wheel.get()
        toSave["Cmd"] = control.steering_cmd
        writer.writerow(toSave)
        control.count += 1
        if control.count > 200 and control.steering_cmd != REF:
            control.steering_cmd = REF
            print("step")
        if control.count > 500:
            break
        rate.sleep()
    csvfile.close()
    print("ending")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
