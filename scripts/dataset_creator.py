#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger, TriggerResponse


class DatasetCreator:
    def __init__(self):
        self._create_dataset_server = rospy.Service(
            "~start_trigger", Trigger, self._handle_create_dataset
        )
        self._start_flag = False
        self._pose = PoseWithCovarianceStamped()

    def _handle_create_dataset(self, req: Trigger) -> TriggerResponse:
        rospy.loginfo("Creating dataset...")
        self._start_flag = True
        return TriggerResponse(success=True, message="Dataset created")


if __name__ == "__main__":
    try:
        rospy.init_node("dataset_creator")
        dataset_creator = DatasetCreator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
