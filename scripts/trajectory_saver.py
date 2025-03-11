#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy

from trajectory_saver_msg.srv import AddTrajectory, AddTrajectoryResponse
from trajectory_saver_msg.srv import GetTrajectory, GetTrajectoryResponse
from trajectory_saver_msg.srv import RemoveTrajectory, RemoveTrajectoryResponse

from trajectory_saver_msg.srv import AddPose, AddPoseResponse
from trajectory_saver_msg.srv import GetPose, GetPoseResponse
from trajectory_saver_msg.srv import RemovePose, RemovePoseResponse

from std_srvs.srv import Trigger, TriggerResponse

import rospy
from std_msgs.msg import String

class TrajectorySaver:

    def __init__(self, node_name):

        self.msg_store = MessageStoreProxy()

        self.node_name_ = node_name

        self.add_pose_service = rospy.Service("~/robot/arm/trajectory_saver/add_pose", AddTrajectory, self.callback_add_pose)
        self.add_trajectory_service = rospy.Service("~/robot/arm/trajectory_saver/add_trajectory", AddTrajectory, self.callback_add_trajectory)
        self.get_pose_service = rospy.Service("~/robot/arm/trajectory_saver/get_pose", GetTrajectory, self.callback_get_pose)
        self.get_trajectory_service = rospy.Service("~/robot/arm/trajectory_saver/get_trajectory", GetTrajectory, self.callback_get_pose)
        self.remove_pose_service = rospy.Service("~/robot/arm/trajectory_saver/remove_pose", RemoveTrajectory, self.callback_remove_trajectory)
        self.remove_trajectory_service = rospy.Service("~/robot/arm/trajectory_saver/remove_trajectory", RemoveTrajectory, self.callback_remove_trajectory)
        self.amount_trajectory_service = rospy.Service("~/robot/arm/trajectory_saver/amount_of_trajectories", Trigger, self.callback_amount_of_trajectory)

        rospy.loginfo("Trajectory Saver Node Started!")


    def callback_add_pose(self, req):

        tf_pose = req.name + "_tf"
        rpy_pose = req.name + "_rpy"
        joint_pose = req.name + "_joint"

        try:
            self.msg_store.update_named(tf_pose, req.tf_pose)
        except:
                return AddPoseResponse(False, "Failed to add tf pose")

        try:
            self.msg_store.update_named(rpy_pose, req.rpy_pose)
        except:
                return AddPoseResponse(False, "Failed to add rpy pose")

        try:
            self.msg_store.update_named(joint_pose, req.joint_pose)
        except:
                return AddPoseResponse(False, "Failed to add joint pose")

        return AddPoseResponse(True, "Updated pose")

    def callback_add_trajectory(self, req):

        trajectory_msg = String()
        trajectory_msg.data = req.trajectory

        trajectory = self.msg_store.query_named(req.name, String._type)

        if(trajectory[0] is None):
            try:
                self.msg_store.insert_named(req.name, trajectory_msg)
            except:
                return AddTrajectoryResponse(False, "")

            return AddTrajectoryResponse(True, "Added")
        try:
            self.msg_store.update_named(req.name, trajectory_msg)
        except:
                return AddTrajectoryResponse(False, "")

        return AddTrajectoryResponse(True, "Updated")

    def callback_get_pose(self, req):

        tf_pose = req.name + "_tf"
        rpy_pose = req.name + "_rpy"
        joint_pose = req.name + "_joint"

        try:
            tf_pose_msg = self.msg_store.query_named(tf_pose, String._type)
        except:
            return GetPoseResponse(False, "Database connection failed")

        if(tf_pose_msg[0] is None):
            return GetPoseResponse(False, "TF Pose does not exist")

        try:
            rpy_pose_msg = self.msg_store.query_named(rpy_pose, String._type)
        except:
            return GetPoseResponse(False, "Database connection failed")

        if(rpy_pose_msg[0] is None):
            return GetPoseResponse(False, "RPY Pose does not exist")

        try:
            joint_pose_msg = self.msg_store.query_named(joint_pose, String._type)
        except:
            return GetPoseResponse(False, "Database connection failed")

        if(joint_pose_msg[0] is None):
            return GetPoseResponse(False, "JOINT Pose does not exist")

        return GetPoseResponse(True, "", tf_pose_msg, rpy_pose_msg, joint_pose_msg)

    def callback_get_trajectory(self, req):

        try:
            trajectory_msg = self.msg_store.query_named(req.name, String._type)
        except:
            return GetTrajectoryResponse(False, "Database connection failed")

        if(trajectory_msg[0] is None):
            return GetTrajectoryResponse(False, "NOT EXISTS")

        return GetTrajectoryResponse(True, str(trajectory_msg[0].data))

    def callback_remove_pose(self, req):

        tf_pose = req.name + "_tf"
        rpy_pose = req.name + "_rpy"
        joint_pose = req.name + "_joint"

        try:
            tf_pose_msg = self.msg_store.query_named(tf_pose, String._type)
        except:
            return RemovePoseResponse(False, "Database connection failed")

        if(tf_pose_msg[0] is None):
            return RemovePoseResponse(False, "Could not find tf pose in database")

        tf_pose_msg_id = tf_pose_msg[1]["_id"]

        try:
            self.msg_store.delete(str(tf_pose_msg_id))
        except:
            return RemovePoseResponse(False, 'Failed to delete tf pose from database')

        try:
            rpy_pose_msg = self.msg_store.query_named(rpy_pose, String._type)
        except:
            return RemovePoseResponse(False, "Database connection failed")

        if(rpy_pose_msg[0] is None):
            return RemovePoseResponse(False, "Could not find rpy pose in database")

        rpy_pose_msg_id = rpy_pose_msg[1]["_id"]

        try:
            self.msg_store.delete(str(rpy_pose_msg_id))
        except:
            return RemovePoseResponse(False, 'Failed to delete rpy pose from database')

        try:
            joint_pose_msg = self.msg_store.query_named(joint_pose, String._type)
        except:
            return RemovePoseResponse(False, "Database connection failed")

        if(joint_pose_msg[0] is None):
            return RemovePoseResponse(False, "Could not find joint pose in database")

        joint_pose_msg_id = joint_pose_msg[1]["_id"]

        try:
            self.msg_store.delete(str(joint_pose_msg_id))
        except:
            return RemovePoseResponse(False, 'Failed to delete joint pose from database')

        return RemovePoseResponse(True, '')

    def callback_remove_trajectory(self, req):

        try:
            trajectory_msg = self.msg_store.query_named(req.name, String._type)
        except:
            return RemoveTrajectoryResponse(False)

        if(trajectory_msg[0] is None):
            return RemoveTrajectoryResponse(False)

        trajectory_msg_id = trajectory_msg[1]["_id"]

        try:
            self.msg_store.delete(str(trajectory_msg_id))
        except:
            return RemoveTrajectoryResponse(False)

        return RemoveTrajectoryResponse(True)

    def callback_amount_of_trajectory(self, req):
        try:
            data = self.msg_store.query(String._type)
        except:
            return TriggerResponse(False, "")

        if(len(data) > 0):
            return TriggerResponse(True, str(len(data)))

        return TriggerResponse(False, str(len(data)))

if __name__ == '__main__':
    nm = 'trajectory_saver_node'
    rospy.init_node(nm)
    TrajectorySaver(nm)
    rospy.spin()
