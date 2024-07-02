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

        self.pub = rospy.Publisher('display_robot_state', DisplayRobotState, queue_size=1)
        self.pub_js = rospy.Publisher('joint_states_offsetted', JointState, queue_size=1)

        self.offset_sub = rospy.Subscriber('joint_states_offsets', JointState, self.joint_states_offsets_callback)
        self.js_sub = rospy.Subscriber('joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        with self.joint_states_lock:
            for name,position in zip(msg.name, msg.position):
                self.js_original[name] = position

        drs = DisplayRobotState()
        drs.state.joint_state = self.joint_states
        self.pub.publish(drs)
        self.pub_js.publish(self.joint_states)

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
    joint_offset_interpreter = JointOffsetInterpreter()
    rospy.spin()


if __name__ == '__main__':
    main()