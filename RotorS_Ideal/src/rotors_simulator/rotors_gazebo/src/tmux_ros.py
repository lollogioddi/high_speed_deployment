#!/usr/bin/env python
import os
import rospy
from std_srvs.srv import Trigger, TriggerResponse

def handle_tmux_request(req):
    commands = rospy.get_param('~commands', '')
    os.system(commands)
    return TriggerResponse(success=True, message="tmux commands executed")

def tmux_server():
    rospy.init_node('tmux_ros')
    s = rospy.Service('tmux_service', Trigger, handle_tmux_request)
    rospy.spin()

if __name__ == "__main__":
    tmux_server()
