#! /usr/bin/env python

from __future__ import print_function

import rospy
import actionlib
import pump_controller.msg

def pump_client():
    client = actionlib.SimpleActionClient('pump_action', pump_controller.msg.ControlPumpAction)

    client.wait_for_server()

    goal = pump_controller.msg.ControlPumpGoal(direction=False, frequency=300, time=5.0) # 300-800
    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('pump_client_py')
        result = pump_client()
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
