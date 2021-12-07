from __future__ import absolute_import, division, print_function #, unicode_literals
from builtins import input
__metaclass__ = type

import bitsets

import rospy
import actionlib
import iiwa_msgs.msg
import iiwa_msgs.srv

class iiwa:

    status_bits = bitsets.bitset('StatusBits', (
        'x_negative',
        'joint_4_negative',
        'joint_6_nonpositive',
    ))

    turn_bits = bitsets.bitset('StatusBits', (
        'joint_1_negative',
        'joint_2_negative',
        'joint_3_negative',
        'joint_4_negative',
        'joint_5_negative',
        'joint_6_negative',
        'joint_7_negative',
    ))

    def __init__(self, robot_name='iiwa', joint_speed=0.1, joint_acc=1, sim=True):
        self._name = robot_name
        self._sim = sim

        # Create action methods for each of the iiwa actions you want to use
        self.joint_action = actionlib.SimpleActionClient(
            'action/move_to_joint_position',
            iiwa_msgs.msg.MoveToJointPositionAction
        )

        self.cart_action = actionlib.SimpleActionClient(
            'action/move_to_cartesian_position',
            iiwa_msgs.msg.MoveToCartesianPoseAction
        )

        self.spline_action = actionlib.SimpleActionClient(
            'action/move_along_spline',
            iiwa_msgs.msg.MoveAlongSplineAction
        )

        self.joint_spline_action = actionlib.SimpleActionClient(
            'action/move_along_joint_spline',
            iiwa_msgs.msg.MoveAlongJointSplineAction
        )

        if not sim:
            #srv_name = '{}/configuration/setPTPJointLimits'.format(self._name)
            srv_name = 'configuration/setPTPJointLimits'
            rospy.wait_for_service(srv_name)
            self.set_speed = rospy.ServiceProxy(
                srv_name,
                iiwa_msgs.srv.SetPTPJointSpeedLimits
            )
            self.set_speed(joint_speed, joint_acc)

    @staticmethod
    def _make_iiwa_position(position, time_from_start):
        """Create an iiwa_msgs joint position from a python list type"""
        jp = iiwa_msgs.msg.JointPosition()
        jp.header.stamp = time_from_start
        jp.position.a1 = position[0]
        jp.position.a2 = position[1]
        jp.position.a3 = position[2]
        jp.position.a4 = position[3]
        jp.position.a5 = position[4]
        jp.position.a6 = position[5]
        jp.position.a7 = position[6]

        return jp

    @staticmethod
    def calculate_redundancy(pose_stamped, joint_point, joint_names):
        j1_idx = joint_names.index('iiwa_joint_1')
        j2_idx = joint_names.index('iiwa_joint_2')
        j3_idx = joint_names.index('iiwa_joint_3')
        j4_idx = joint_names.index('iiwa_joint_4')
        j5_idx = joint_names.index('iiwa_joint_5')
        j6_idx = joint_names.index('iiwa_joint_6')
        j7_idx = joint_names.index('iiwa_joint_7')

        redundancy = iiwa_msgs.msg.RedundancyInformation()
        redundancy.e1 = joint_point.positions[j3_idx]
        status = iiwa.status_bits.frombools([
            pose_stamped.pose.position.x < 0,
            joint_point.positions[j4_idx] < 0,
            joint_point.positions[j6_idx] <= 0,
        ])
        redundancy.status = int(status)
        turn = iiwa.turn_bits.frombools([
            joint_point.positions[j1_idx] < 0,
            joint_point.positions[j2_idx] < 0,
            joint_point.positions[j3_idx] < 0,
            joint_point.positions[j4_idx] < 0,
            joint_point.positions[j5_idx] < 0,
            joint_point.positions[j6_idx] < 0,
            joint_point.positions[j7_idx] < 0,
        ])
        redundancy.turn = int(turn)
        return redundancy

    def follow_cart_trajectory(self, traj):
        goal = iiwa_msgs.msg.MoveAlongSplineGoal()

        #cycle through all points of the trajectory returned from trajopt
        for (idx, cart_pose) in enumerate(traj.cart_trajectory):
            iiwa_cart_pose = iiwa_msgs.msg.CartesianPose()
            iiwa_cart_pose.poseStamped = cart_pose
            iiwa_cart_pose.redundancy = iiwa.calculate_redundancy(
                        cart_pose,
                        traj.joint_trajectory.points[idx],
                        traj.joint_trajectory.joint_names)

            #create spline_segment in the correct type for each iteration
            spline_segment = iiwa_msgs.msg.SplineSegment()

            #populate segment with the corresponding line from the trajectory solution in cartesian coordinates
            spline_segment.point = iiwa_cart_pose
            spline_segment.type=0 #type 0 is SPL, type 1 is LIN

            #add this new segment to the goal
            goal.spline.segments.append(spline_segment)

        #This is used for debugging to see the points in the trajectory being sent
        #for segment in goal.spline.segments:
            #rospy.logdebug("x = {p.x}, y = {p.y}, z = {p.z}".format(
                #p=segment.point.poseStamped.pose.position)

        self.execute_trajectory(self.spline_action, goal)

    def follow_joint_trajectory(self, traj):
        goal = iiwa_msgs.msg.MoveAlongJointSplineGoal()
        for pt in traj.points:
            goal.spline.segments.append(self._make_iiwa_position(pt.positions, rospy.Time(0) + pt.time_from_start))

        self.execute_trajectory(self.joint_spline_action, goal)
        
    def cancel_joint_trajectory(self):
        
        self.joint_spline_action.cancel_all_goals()

    def execute_trajectory(self, action_client, action_goal):
        if not self._sim:
            connect = action_client.wait_for_server(rospy.Duration(10.0))
            if not connect:
                raise RuntimeError('Timed out connecting to server')

            action_client.send_goal(action_goal)
            action_client.wait_for_result(rospy.Duration())
            result = action_client.get_result()

            if result != None:
                if result.success:
                    rospy.loginfo('Execution succeeded')
                else:
                    raise RuntimeError('Execution failed: {}'.format(result.error))
            else:
                raise RuntimeError('Execution timeout')
        else:
            if isinstance(action_goal, iiwa_msgs.msg.MoveAlongJointSplineGoal):
                rospy.loginfo("Sim joint trajectory move, ending at: " +
                    "[{jp.a1:.3}, {jp.a2:.3}, {jp.a3:.3}, {jp.a4:.3}, {jp.a5:.3}, {jp.a6:.3}, {jp.a7:.3}]".format(
                    jp=action_goal.spline.segments[-1].position))
            elif isinstance(action_goal, iiwa_msgs.msg.MoveAlongSplineGoal):
                rospy.loginfo("Sim cart trajectory move, ending at: ({pt.x:.3}, {pt.y:.3}, {pt.z:.3})".format(
                    pt=action_goal.spline.segments[-1].point.poseStamped.pose.position))
            input('  press enter to continue')

    def go_to_joint(self, pos):

        #defines goal
        goal = iiwa_msgs.msg.MoveToJointPositionGoal()

        #populates goal with output from make_iiwa_position above (type joint_position)
        print(pos)
        goal.joint_position = self._make_iiwa_position(pos)


        if not self._sim:

            #tells iiwa to go to joint_position
            connect = self.joint_action.wait_for_server(rospy.Duration(10.0))
            if not connect:
                raise RuntimeError('Timed out connecting to server')

            self.joint_action.send_goal(goal)
            self.joint_action.wait_for_result(rospy.Duration(10.0))

            result = self.joint_action.get_result()

            if result != None:
                if result.success:
                    rospy.loginfo('MoveToJointPosition succeeded')
                else:
                    raise RuntimeError(
                        'MoveToJointPosition failed: {}'.format(result.error))
            else:
                raise RuntimeError('MoveToJointPosition timeout')

        else:
            rospy.loginfo("Sim joint move: {}".format(pos))
            input('  press enter to continue')
