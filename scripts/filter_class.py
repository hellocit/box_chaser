#!/usr/bin/env python3
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import Header, String, Bool
from std_srvs.srv import SetBool, SetBoolResponse

class FilterSubscriberNode:
    def __init__(self):
        rospy.init_node('filter_subscriber')
        self.detect_box = False
        self.detect_box_2 = False
        self.detect_result_flag_flag = False
        self.labels = []
        self.start_time = None
        self.detected = False
        self.filter_strings = ['tag', 'green_box', 'blue_box']
        
        self.label_publisher = rospy.Publisher('/label_string', String, queue_size=1)
        self.detect_box_2_service = rospy.Service('detect_box_2', SetBool, self.detect_box_2_srv)
        self.detect_result_flag_service = rospy.Service('detect_result_flag', SetBool, self.detect_result_flag_srv)
        rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.boundingBoxesCallback)
        self.rate = rospy.Rate(10)

    def boundingBoxesCallback(self, data):
        self.labels = [bbox.Class for bbox in data.bounding_boxes]
        self.detected = bool(self.labels)

    def detect_box_2_srv(self, data):
        self.detect_box_2 = data.data
        resp = SetBoolResponse()
        if self.detect_box_2:
            resp.message = "called"
            resp.success = True
        else:
            resp.message = "ready"
            resp.success = False
        print(resp.message)
        return resp

    def detect_result_flag_srv(self, data):
        self.detect_result_flag_flag = data.data
        resp = SetBoolResponse()
        if self.detect_result_flag_flag:
            resp.message = "called"
            resp.success = True
        else:
            resp.message = "ready"
            resp.success = False
        print(resp.message)
        return resp

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("labels: {}".format(self.labels))
            filtered_labels = list(filter(lambda label: label not in self.filter_strings, self.labels))
            label_str = ' '.join(filtered_labels)
            label_msg = String()
            label_msg.data = label_str
            detection_msg = Bool()
            detection_msg.data = self.detected
            self.label_publisher.publish(label_msg)

            if self.start_time is None:
                self.start_time = rospy.get_time()
            else:
                rospy.logwarn("start_time is false")
            current_time = rospy.get_time()
            elapsed_time = current_time - self.start_time
            rospy.loginfo("elapsed_time: %f", elapsed_time)

            if elapsed_time >= 10.0 and self.detect_result_flag_flag:
                rospy.wait_for_service('detect_result')
                try:
                    service_call = rospy.ServiceProxy('detect_result', SetBool)
                    service_call(True)
                    rospy.loginfo("finish detect_result")
                    self.detect_result_flag_flag = False
                except rospy.ServiceException as e:
                    print ("Service call failed: %s" % e)
            else:
                rospy.loginfo("elapsed_time is false")
            self.rate.sleep()

if __name__ == '__main__':
    filter_subscriber = FilterSubscriberNode()
    filter_subscriber.run()
