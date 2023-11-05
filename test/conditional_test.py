#!/usr/bin/env python3
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes


class condition_test_node:
    def __init__(self):
        rospy.init_node('yolo_test_node', anonymous=True)
        self.yolo_test_callback_test = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.yolo_test_callback)
        self.labels = []
        self.str_labels = ""
        self.flag_list = ["tag", "green_box", "blue_box", "tag_a", "tag_b", "tag_c"]





    def yolo_test_callback(self, data):
        rospy.loginfo("yolo_test_callback is running")
        self.labels = [bbox.Class for bbox in data.bounding_boxes]
        rospy.loginfo(self.labels)
        rospy.loginfo(type(self.labels))

        self.str_labels = ", ".join(map(str, self.labels))
        rospy.loginfo(self.str_labels)
        rospy.loginfo(type(self.str_labels))

        rospy.loginfo("--------------------------")

        rospy.loginfo(self.flag_list[2])
        rospy.loginfo(type(self.flag_list[2]))

        if self.flag_list[0] == self.labels or self.flag_list[1] == self.labels or self.flag_list[2] == self.labels or self.flag_list[3] == self.labels or self.flag_list[4] == self.labels or self.flag_list[5] == self.labels:
            rospy.loginfo("OKKKKKKKKKKKKKK")





if __name__ == '__main__':
    test = condition_test_node()
    DURATION = 0.1
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        r.sleep()