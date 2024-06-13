#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from sr_robot_msgs.msg import EthercatDebug

class SensorReadingInterpreter:
    def __init__(self):
        self.pub = rospy.Publisher('joint_readings_for_calibration', JointState, queue_size=1)
        self.readings = JointState()

        self.SENSOR_FIELD_CNT = 37 # number of sensor entries (+1 IGNORE) in the protocol
        self.SENSOR_FIELDS = [ # yes, this is not all the fields, but only the prefix part of the array with joint information
            "FFJ1",  "FFJ2",  "FFJ3", "FFJ4",
            "MFJ1",  "MFJ2",  "MFJ3", "MFJ4",
            "RFJ1",  "RFJ2",  "RFJ3", "RFJ4",
            "LFJ1",  "LFJ2",  "LFJ3", "LFJ4", "LFJ5",
            "THJ1",  "THJ2",  "THJ3", "THJ4", "THJ5A", "THJ5B",
            "WRJ1A", "WRJ1B", "WRJ2",
        ]

        self.readings.name = [
            "FFJ1", "FFJ2", "FFJ3", "FFJ4",
            "MFJ1", "MFJ2", "MFJ3", "MFJ4",
            "RFJ1", "RFJ2", "RFJ3", "RFJ4",
            "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
            "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
            "WRJ1", "WRJ2",
        ]
        self.sub = rospy.Subscriber('debug_etherCAT_data', EthercatDebug, self.callback)

    def callback(self, msg : EthercatDebug):
        if (len(msg.sensors) != self.SENSOR_FIELD_CNT):
            rospy.logfatal_once(f"Found {len(msg.sensors)} sensor values, but the interface code defines {self.SENSOR_FIELD_CNT}\nSee sr-ros-interface-ethercat/sr_external_dependencies/include/sr_external_dependencies/external/common/common_edc_ethercat_protocol.h")
            rospy.signal_shutdown("Mismatch in sensor values")
            return

        self.readings.header.stamp = msg.header.stamp
        self.readings.position = []

        for name, reading in zip(self.SENSOR_FIELDS, msg.sensors):
            if name[-1] == "A":
                self.readings.position.append(0.5*reading)
            elif name[-1] == "B":
                self.readings.position[-1] += 0.5*reading
            else:
                self.readings.position.append(reading)

        assert len(self.readings.position) == len(self.readings.name), f"Logic bug: {len(self.readings.position)} != {len(self.readings.name)}"

        self.pub.publish(self.readings)


if __name__ == '__main__':
    rospy.init_node('shadow_hand_raw_sensor_joint_readings')

    sensor_reading_interpreter = SensorReadingInterpreter()

    rospy.spin()