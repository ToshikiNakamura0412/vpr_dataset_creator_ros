#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional

import cv2 as cv
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion


@dataclass
class Data:
    image_name: str
    x: float
    y: float
    theta: float


class DatasetCreator:
    def __init__(self):
        self._dist_threshold: float = rospy.get_param("~dist_threshold", 5.0)
        self._dir_path: str = rospy.get_param("~dir_path", "dataset")
        self._create_dataset_server: rospy.Service = rospy.Service(
            "~start_trigger", Trigger, self._handle_create_dataset
        )
        self._pose_sub: rospy.Subscriber = rospy.Subscriber(
            "/localized_pose", PoseWithCovarianceStamped, self._pose_callback
        )
        self._image_sub: rospy.Subscriber = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self._image_callback
        )
        self._start_flag: bool = False
        self._pose: Pose = Pose()
        self._base_pose: Optional[Pose] = None
        self._data_count: int = 0

    def _handle_create_dataset(self, req: Trigger) -> TriggerResponse:
        rospy.loginfo("Creating dataset...")
        self._start_flag = True
        return TriggerResponse(success=True, message="Start creating dataset...")

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self._pose = msg.pose.pose
        if self._start_flag and self._base_pose is None:
            self._base_pose = self._pose

    def _image_callback(self, msg: Image):
        if self._base_pose is not None:
            dist = np.hypot(
                self._pose.position.x - self._base_pose.position.x,
                self._pose.position.y - self._base_pose.position.y,
            )
            if dist > self._dist_threshold:
                self._save_image(msg)
                self._save_data()
                self._data_count += 1
                self._base_pose = None

    def _save_image(self, msg: Image):
        image = CvBridge().imgmsg_to_cv2(msg, "mono8")
        image = self._scale_to_resolution(image, 240)
        cv.imwrite(f"{self._dir_path}/image_{self._data_count}.png", image)
        rospy.loginfo(f"Saved image_{self._data_count}.png")

    def _scale_to_resolution(self, image:Image, resolution: int):
       h, w = image.shape[:2]
       return cv.resize(image, (int(w * resolution / h), resolution))

    def _save_data(self):
        with open(f"{self._dir_path}/data.csv", "a") as f:
            _, _, yaw = euler_from_quaternion(
                [
                    self._pose.orientation.x,
                    self._pose.orientation.y,
                    self._pose.orientation.z,
                    self._pose.orientation.w,
                ]
            )
            data = Data(
                image_name=f"image_{self._data_count}.png",
                x=self._pose.position.x,
                y=self._pose.position.y,
                theta=yaw,
            )
            f.write(f"{data.image_name},{data.x},{data.y},{data.theta}\n")
            rospy.loginfo(f"Saved data_{self._data_count}\n")


if __name__ == "__main__":
    try:
        rospy.init_node("dataset_creator")
        dataset_creator = DatasetCreator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
