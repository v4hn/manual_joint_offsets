#!/usr/bin/env python

import rospy
import threading

from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState

class JointOffsetInterpreter:
    def __init__(self):
        self.js_original = {}
        self.joint_states_lock = threading.Lock()

        self.js_offsets = {}
        self.joint_states_offsets_lock = threading.Lock()

        rospy.Subscriber('joint_states_offsets', JointState, self.joint_states_offsets_callback)
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        with self.joint_states_lock:
            for name,position in zip(msg.name, msg.position):
                self.js_original[name] = position

    def joint_states_offsets_callback(self, msg):
        with self.joint_states_offsets_lock:
            for name,position in zip(msg.name, msg.position):
                self.js_offsets[name] = position

    @property
    def joint_states(self):
        msg = JointState()
        with self.joint_states_lock and self.joint_states_offsets_lock:
            for joint in self.js_original:
                msg.name.append(joint)
                msg.position.append(self.js_original[joint] + self.js_offsets.get(joint, 0.0))
        return msg


def main():
    rospy.init_node('joint_offset_interpreter')
    pub = rospy.Publisher('display_robot_state', DisplayRobotState, queue_size=1)

    joint_offset_interpreter = JointOffsetInterpreter()

    rospy.sleep(1.0)
    rate = rospy.Rate(10)
    msg = DisplayRobotState()
    while not rospy.is_shutdown():
        joint_states = joint_offset_interpreter.joint_states
        msg.state.joint_state = joint_states
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()