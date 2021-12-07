#!/usr/bin/env python

import roslib
import rospy
import threading

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as inputMsg
from time import sleep
from assembly_ros.srv import (RobotiqGripperActivate,
                            RobotiqGripperActivateRequest,
                            RobotiqGripperActivateResponse,
                            RobotiqGripperSetPosition,
                            RobotiqGripperSetPositionResponse,
                            RobotiqGripperSetSpeed,
                            RobotiqGripperSetSpeedResponse,
                            RobotiqGripperSetForce,
                            RobotiqGripperSetForceResponse)


class RobotiqService():

    def __init__(self):

        gripper_name = rospy.get_param('~gripper_name', 'robotiq_gripper')

        # Services to control the gripper's activation, position, speed, and force
        self.activate_command = rospy.Service('~activate_gripper', RobotiqGripperActivate, self._activate_gripper)
        self.pos_command = rospy.Service('~set_gripper_position', RobotiqGripperSetPosition, self._set_gripper_position)
        self.speed_command = rospy.Service('~set_gripper_speed', RobotiqGripperSetSpeed, self._set_gripper_speed)
        self.force_command = rospy.Service('~set_gripper_force', RobotiqGripperSetForce, self._set_gripper_force)

        # A publisher to send commands to the gripper
        self._command_pub = rospy.Publisher(gripper_name + '/output', outputMsg, queue_size=1)
        self._curr_command = outputMsg()
        self._command_lock = threading.Lock()

        # A subscriber to check the current status of the gripper
        self._state_sub= rospy.Subscriber(gripper_name + '/input', inputMsg, self._set_curr_status)
        self._curr_status= inputMsg()
        self._status_lock = threading.Lock()

    def is_activated(self):
        status = self._get_curr_status()
        return status.gSTA == 3 and status.gACT == 1

    def is_reset(self):
        status = self._get_curr_status()
        return status.gSTA == 0 and status.gACT == 0

    def is_moving(self):
        status = self._get_curr_status()
        return status.gGTO == 1 and status.gOBJ == 0

    def is_stopped(self):
        status = self._get_curr_status()
        return status.gOBJ != 0

    def is_faulted(self):
        status = self._get_curr_status()
        # Check if there is a fault
        return status.gFLT != 0 and status.gFLT !=5

    def object_detected(self):
        status = self._get_curr_status()
        return status.gOBJ == 1 or status.gOBJ == 2

    # Wait until given status function is true, if timeout is negative, wait forever
    def _wait_until_status(self, status, timeout=5):
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and rospy.get_time() - start_time > timeout) or self.is_faulted():
                return False
            if status():
                return True
            r.sleep()
        return False

    def _set_curr_status(self, status):
        with self._status_lock:
          self._curr_status = status

    def _get_curr_status(self):
        with self._status_lock:
            status = self._curr_status
        return status

    def _set_curr_command(self, command):
        with self._command_lock:
            self._curr_command = command

    def _get_curr_command(self):
        with self._command_lock:
            command = self._curr_command
        return command

    def _publisher(self, command):
        self._set_curr_command(command)
        self._command_pub.publish(command)

    def _activate_gripper(self, request = RobotiqGripperActivateRequest()):
        # Reset gripper before activating
        command = outputMsg()
        command.rACT =  0

        self._publisher(command)

        if not self._wait_until_status(self.is_reset):
            return RobotiqGripperActivateResponse(False)

        # Activate gripper, set default speed and force
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

        self._publisher(command)

        return RobotiqGripperActivateResponse(self._wait_until_status(self.is_activated))

    def _set_gripper_position(self, request):
        command = self._get_curr_command()
        response = RobotiqGripperSetPositionResponse(True)

        # Activate gripper if it isn't activated
        if not self.is_activated():
            self._activate_gripper()
            command = self._get_curr_command()

        command.rPR = request.position
        self._publisher(command)

        if not self._wait_until_status(self.is_moving):
            response.success = False
            return response

        response.success = self._wait_until_status(self.is_stopped)

        return response

    def _set_gripper_speed(self, request):
        command = self._get_curr_command()

        # Activate gripper if it isn't activated
        if not self.is_activated():
            self._activate_gripper()
            command = self._get_curr_command()

        command.rSP = request.speed
        self._publisher(command)

        return RobotiqGripperSetSpeedResponse(not self.is_faulted())

    def _set_gripper_force(self, request):
        command = self._get_curr_command()

        # Activate gripper if it isn't activated
        if not self.is_activated():
            self._activate_gripper()
            command = self._get_curr_command()

        command.rFR = request.force
        self._publisher(command)

        return RobotiqGripperSetForceResponse(not self.is_faulted())

if __name__ == '__main__':
    rospy.init_node("robotiq_gripper_service")
    gripper = RobotiqService()
    rospy.spin()
