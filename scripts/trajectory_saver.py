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

from std_srvs.srv import Trigger, TriggerResponse

import rospy
from std_msgs.msg import String

class TrajectorySaver:

    def __init__(self):

        self.msg_store = MessageStoreProxy()

        self.add_trajectory_service = rospy.Service("~/trajectory_saver/add_trajectory", AddTrajectory, self.callback_add_trajectory)
        self.get_trajectory_service = rospy.Service("~/trajectory_saver/get_trajectory", GetTrajectory, self.callback_get_trajectory)
        self.remove_trajectory_service = rospy.Service("~/trajectory_saver/remove_trajectory", RemoveTrajectory, self.callback_remove_trajectory)
        self.amount_trajectory_service = rospy.Service("~/trajectory_saver/amount_of_trajectories", Trigger, self.callback_amount_of_trajectory)

        rospy.loginfo("Trajectory Saver Node Started!")


    def callback_add_trajectory(self, req):

        trajectory_msg = String()
        trajectory_msg.data = req.trajectory

        trajectory = self.msg_store.query_named(req.name, String._type)

        if(trajectory[0] is None):
            self.msg_store.insert_named(req.name, trajectory_msg)
            return AddTrajectoryResponse(True, "Added")

        self.msg_store.update_named(req.name, trajectory_msg)
        return AddTrajectoryResponse(True, "Updated")


    def callback_get_trajectory(self, req):

        trajectory_msg = self.msg_store.query_named(req.name, String._type)
        if(trajectory_msg[0] is None):
            return GetTrajectoryResponse(False, "NOT EXISTS")

        return GetTrajectoryResponse(True, str(trajectory_msg[0].data))

    def callback_remove_trajectory(self, req):

        trajectory_msg = self.msg_store.query_named(req.name, String._type)

        if(trajectory_msg[0] is None):
            return RemoveTrajectoryResponse(False)

        trajectory_msg_id = trajectory_msg[1]["_id"]
        self.msg_store.delete(str(trajectory_msg_id))
        return RemoveTrajectoryResponse(True)

    def callback_amount_of_trajectory(self, req):
        data = self.msg_store.query(String._type)
        if(len(data) > 0):
            return TriggerResponse(True, str(len(data)))

        return TriggerResponse(False, str(len(data)))

if __name__ == '__main__':
    rospy.init_node('trajectory_saver')
    TrajectorySaver()
    rospy.spin()
