#! /usr/bin/env python

import rospy
from copy import deepcopy
import math
import operator
import numpy as np
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from std_msgs.msg import (
    UInt16,
)

#import actionlib_tutorials.msg

class ScorbotAction(object):
    # create messages that are used to publish feedback/result


    def __init__(self, name,rate):
        self._name = name
        self._server = actionlib.SimpleActionServer(self._name, FollowJointTrajectoryAction, 						execute_cb=self._on_trajectory_action, auto_start = False)
        self._server.start()
	self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()
	self._alive = True
        # Controller parameters from arguments, messages, and dynamic
        # reconfigure
        self._control_rate = rate  # Hz
        self._control_joints = []
        self._pid_gains = {'kp': dict(), 'ki': dict(), 'kd': dict()}
        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()
        rospy.loginfo('goal_time')


    def clean_shutdown(self):
	self._alive = False
    def _get_current_position(self, joint_names):
        return [0,0,0,0,0]#[self._limb.joint_angle(joint) for joint in joint_names]

    def _get_current_velocities(self, joint_names):
        return [0,0,0,0,0]#[self._limb.joint_velocity(joint) for joint in joint_names]
    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        return zip(joint_names, error)
    def _update_feedback(self, cmd_point, jnt_names, cur_time):
        self._fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._fdbk.joint_names = jnt_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.actual.positions = self._fdbk.desired.positions#self._get_current_position(jnt_names)
        self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.error.positions = map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions
                                        )
        self._fdbk.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._server.publish_feedback(self._fdbk)

    def _command_joints(self, joint_names, point, start_time, dimensions_dict):
       return True
    def _determine_dimensions(self, trajectory_points):
        # Determine dimensions supplied
        position_flag = True
        velocity_flag = (len(trajectory_points[0].velocities) != 0 and
                         len(trajectory_points[-1].velocities) != 0)
        acceleration_flag = (len(trajectory_points[0].accelerations) != 0 and
                             len(trajectory_points[-1].accelerations) != 0)
        return {'positions':position_flag,
                'velocities':velocity_flag,
                'accelerations':acceleration_flag}   

    def _on_trajectory_action(self, goal):
        print(goal)
        joint_names = goal.trajectory.joint_names
	trajectory_points = goal.trajectory.points
      # Load parameters for trajectory
        #self._get_trajectory_parameters(joint_names, goal)
        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._name,))
            self._server.set_aborted()
            return
        rospy.loginfo("%s: Executing requested joint trajectory" %
                      (self._name,))
        rospy.logdebug("Trajectory Points: {0}".format(trajectory_points))
        #for jnt_name, jnt_value in self._get_current_error(
        #        joint_names, trajectory_points[0].positions):
        #    if abs(self._path_thresh[jnt_name]) < abs(jnt_value):
        #        rospy.logerr(("{0}: Initial Trajectory point violates "
        #                     "threshold on joint {1} with delta {2} radians. "
        #                     "Aborting trajectory execution.").format(
        #                     self._name, jnt_name, jnt_value))
        #        self._server.set_aborted()
        #        return
        control_rate = rospy.Rate(self._control_rate)
        dimensions_dict = self._determine_dimensions(trajectory_points)

        # Force Velocites/Accelerations to zero at the final timestep
        # if they exist in the trajectory
        # Remove this behavior if you are stringing together trajectories,
        # and want continuous, non-zero velocities/accelerations between
        # trajectories
        if dimensions_dict['velocities']:
            trajectory_points[-1].velocities = [0.0] * len(joint_names)
        if dimensions_dict['accelerations']:
            trajectory_points[-1].accelerations = [0.0] * len(joint_names)
        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
        rospy.logdebug(pnt_times)

        start_time = goal.trajectory.header.stamp.to_sec()
        if start_time == 0.0:
            start_time = rospy.get_time()
        # Loop until end of trajectory time.  Provide a single time step
        # of the control rate past the end to ensure we get to the end.
        # Keep track of current indices for spline segment generation
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        while (now_from_start < end_time and not rospy.is_shutdown()):
            now = rospy.get_time()
            now_from_start = now - start_time
            #ir adquiriendo cada punto de trajectory_opoint de acuerdo a .time_from_start.to_sec()

#aqui tenemos que usar Preemted
#if self._server.is_preempt_requested():
#           rospy.logwarn("{0}: Trajectory execution Preempted. Stopping execution.".format(
#                            self._action_name))
#           self._server.set_preempted()
#return False
	    point=trajectory_points[-1]
            self._update_feedback(deepcopy(point), joint_names, now_from_start)
            # Sleep to make sure the publish is at a consistent time
            control_rate.sleep()
        # Keep trying to meet goal until goal_time constraint expired
        last = trajectory_points[-1]
        last_time = trajectory_points[-1].time_from_start.to_sec()
        end_angles = dict(zip(joint_names, last.positions))
        while (now_from_start < (last_time + self._goal_time)
               and not rospy.is_shutdown()):

            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)
#aqui tenemos que usar Preemted
#if self._server.is_preempt_requested():
#           rospy.logwarn("{0}: Trajectory execution Preempted. Stopping execution.".format(
#                            self._action_name))
#           self._server.set_preempted()
#return False
            control_rate.sleep()

        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)

        # Verify goal constraint
        result = True
        if result is True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded for %s arm" %
                          (self._name, self._name))
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold for %s arm" %
                         (self._name, self._name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s for %s arm" %
                         (self._name, result, self._name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
        self._server.set_aborted(self._result)
        
        
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = ScorbotAction('scorbot/joint_trajectory_action',10)
    rospy.spin()
