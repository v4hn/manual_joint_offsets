#!/usr/bin/env python

import os
import pandas as pd
import rospy
import threading

from moveit_msgs.msg import DisplayRobotState
from sensor_msgs.msg import JointState

# from hanging_threads import start_monitoring
# monitoring_thread = start_monitoring()

class SnapshotTuningCurves:
    def __init__(self):
        self.joint_state = None
        self.joint_state_lock = threading.Lock()
        self.readings = None
        self.readings_lock = threading.Lock()

        self.sub_tuned = rospy.Subscriber('display_robot_state', DisplayRobotState, self.tuned_callback)
        self.sub_readings = rospy.Subscriber('hand/rh/joint_readings_for_calibration', JointState, self.readings_callback)

    def tuned_callback(self, msg : DisplayRobotState):
        with self.joint_state_lock:
            self.joint_state = msg.state.joint_state

    def readings_callback(self, msg):
        with self.readings_lock:
            self.readings = msg

    def snap(self):
        with self.joint_state_lock and self.readings_lock:
            if self.joint_state is None:
                raise RuntimeError("No joint_state available yet")
            if self.readings is None:
                raise RuntimeError("No readings available yet")
            S = {}
            for reading_i, sensorname in enumerate(self.readings.name):
                try:
                    js_i = next(i for i, name in enumerate(self.joint_state.name) if sensorname in name)
                    S[sensorname] = {'position': self.joint_state.position[js_i], 'reading': self.readings.position[reading_i]}
                except StopIteration:
                    rospy.logerr(f"Could not find joint for sensor {j} in joint_state")

            return self.readings.header.stamp.to_sec(), S


def dump(D, file):
    rospy.loginfo(f"Writing {len(D)} samples to '{file}'")

    Dserial = []
    for t, S in D:
        for name in S:
            Dserial.append((t, name, S[name]['position'], S[name]['reading']))

    pd.DataFrame(Dserial, columns=['time', 'joint', 'position', 'reading']).to_csv(file, index=False)


if __name__ == '__main__':
    rospy.init_node('snapshot_tuning_curves')

    file = rospy.get_param('manual_calibration/samples_file', 'tuning_curve_samples.csv')

    autosnap = False

    snapshotter = SnapshotTuningCurves()
    D= []

    autosnap_rate = None
    autosnap_dump = 0

    try:
        while not rospy.is_shutdown():
            if not autosnap:
                try:
                    user_input = input("<Press enter to take snapshot / 'd' to dump / 'a' for autosnap>")
                except EOFError:
                    print()
                    break
            if user_input == 'd':
                dump(D, file)
            elif rospy.is_shutdown():
                break
            elif autosnap or user_input == 'a':
                autosnap = True
                D.append(snapshotter.snap())
                print("Snap, ", end='', flush=True)
                if autosnap_dump == 0:
                    dump(D, file)
                    autosnap_dump = 50
                else:
                    autosnap_dump -= 1
                if autosnap_rate is None:
                    autosnap_rate = rospy.Rate(10)
                autosnap_rate.sleep()
            else:
                D.append(snapshotter.snap())
                dump(D, file)
                rospy.loginfo(f"Done")
    except KeyboardInterrupt:
        pass

    rospy.loginfo(f"Finalizing {len(D)} samples...")

    dump(D, file)
