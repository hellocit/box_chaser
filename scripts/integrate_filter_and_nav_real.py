#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, SetBoolResponse
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import Header, String, Bool

class D1_node:
    def __init__(self):
        rospy.init_node('D1_node', anonymous=True)
        self.position_error = 0.0
        self.width = 0.0
        self.average_range = 0.0
        self.desired_distance = 0.45
        self.start_time = None
        self.go_on_flag = True

        self.flag_desired_distance = True
        self.approached_box = False
        self.detect_box = False
        self.wait_process_start_time = None
        self.wait_process_current_time = 0
        self.wait_process_elapsed_time = 0
        self.time = 0
        self.finish_camera_forward_process_flag = False
        self.camera_send_control_commands_flag = False
        self.back_process_flag = True
        self.camera_send_control_commands_is_finished_flag = True

        self.vel = Twist()
        self.str = String()



        self.detect_box_2 = False

        self.labels = []
        self.filter_strings = ['tag', 'green_box','blue_box']
        self.detected = False
        self.label_string_count = 1
        self.label_msg2 = 0
        self.detected_publisher = rospy.Publisher('/detection_status', Bool, queue_size=10)
        self.string_subscriber = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.string_callback)


        self.start_time = None
        self.finish_flag_flag = True
        self.cmd_vel_publisher = rospy.Publisher('/yolo_vel', Twist, queue_size=1)

        self.label_publisher = rospy.Publisher('/label_string', String, queue_size=1)
        self.publisher_cmd_vel_by_camera = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.boundingBoxesCallback)
        self.calculate_bbox_width = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.calculate_xmax_xmin)
        self.laser_scan_subscriber = rospy.Subscriber('/low_scan', LaserScan, self.laserscanCallback)
        self.detect_box_ = rospy.Service("detect_box", SetBool, self.detect_box_srv)
    #     self.label_string_subscriber = rospy.Subscriber("/label_string", SetBool, self.label_string)

    def string_callback(self, data):
        self.labels = [bbox.Class for bbox in data.bounding_boxes]
        # rospy.loginfo(self.labels)
        self.detected = bool(self.labels)
        # rospy.loginfo(self.detected)

    def label_string(self):
        self.label_string_count += 1
        rospy.loginfo(self.label_string_count)
        label_str = ' '.join(self.labels)
        label_msg = String()
        label_msg.data = label_str
        self.label_msg2 = label_str
        detection_msg = Bool()
        detection_msg.data = self.detected
        self.label_publisher.publish(label_msg)
        self.detected_publisher.publish(detection_msg)
        self.detect_result_client()

    def detect_box_srv(self, data):
        rospy.loginfo("detect_box_srv")
        resp = SetBoolResponse()
        if data.data:
            resp.message = "called"
            resp.success = True
            self.detect_box = True
            self.go_on_flag = True
        else:
            resp.message = "ready"
            resp.success = False
            self.detect_box = False
        return resp

    def finish_flag(self):
        rospy.loginfo("Finish")
        try:
            service_call = rospy.ServiceProxy('finish', SetBool)
            service_call(True)
            rospy.loginfo("finish detect_result")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def boundingBoxesCallback(self, msg):
        # rospy.loginfo("boundingBoxesCallback")
        for box in msg.bounding_boxes:
            if box.Class in ["tag", "green_box", "blue_box", "tag_a", "tag_b", "tag_c"]:
                xmin = box.xmin
                xmax = box.xmax
                self.position_error = (xmin + xmax) / 2.0 - 320  # Adjust 320 as needed

    def laserscanCallback(self, msg):
        self.laser_scan_msg = msg
        # rospy.loginfo("width: %f", self.width)
        # rospy.loginfo("lidar_send_control_commands")
        # Get the number of laser ranges in the LaserScan message
        num_ranges = len(self.laser_scan_msg.ranges)
        # Calculate the index to split the ranges in half (assuming symmetrical LiDAR data)
        split_num = int(num_ranges / 2)
        # Calculate the sum of a small range of laser measurements around the center
        sum_ranges = sum(self.laser_scan_msg.ranges[split_num - 3:split_num + 4])
        # Calculate the average range in the selected region
        self.average_range = sum_ranges / 7
        # rospy.loginfo("Average range: %f", self.average_range)
        # rospy.loginfo("average_range-desired_distance: %f", self.average_range-self.desired_distance)

    def calculate_xmax_xmin(self, msg):
        for bbox in msg.bounding_boxes:
            xmin = bbox.xmin
            xmax = bbox.xmax
            self.width = xmax - xmin
            # rospy.loginfo(self.width)

    def camera_send_control_commands(self):
        # if self.width > 0:
        rospy.loginfo("camera_send_control_commands")
        self.vel.linear.x = 0.2  # Linear velocity (m/s)
        self.vel.angular.z = -float(self.position_error) / 1000
        # rospy.loginfo("self.vel.linear.x: %f", self.vel.linear.x)
        # rospy.loginfo("self.vel.angular.z: %f", self.vel.angular.z)
        rospy.loginfo(self.width>140)
        self.cmd_vel_publisher.publish(self.vel)

    def lidar_send_control_commands(self):
        rospy.loginfo("average_range: %f", self.average_range)
        rospy.loginfo(self.average_range > self.desired_distance)
        # Check if the average range is greater than the desired distance and the flag_desired_distance is set
        # if self.average_range > self.desired_distance and self.flag_desired_distance:
        if self.average_range > self.desired_distance:
            # Set the linear velocity to move forward (0.05 m/s)
            self.vel.linear.x = 0.2
            # Publish the linear velocity commands
            self.cmd_vel_publisher.publish(self.vel)
            rospy.loginfo("self.vel.linear.x: %f", self.vel.linear.x)
        else:
            # If none of the above conditions are met, wait for a specified time
            rospy.loginfo('wait process')
            start_time = time.time()
            while time.time() - start_time < 6.0:
                # Set a very low linear velocity for waiting
                self.vel.linear.x = 0.000001
                self.vel.angular.z = 0.000001
                # Publish the linear velocity commands for waiting
                self.cmd_vel_publisher.publish(self.vel)
                # Update the flag to indicate that the desired distance has been reached
                self.flag_desired_distance = False
                # Mark that the robot has approached a box
                self.approached_box = True

    def back_process(self):
        rospy.loginfo(self.label_msg2)
        rospy.loginfo("back process")
        # rospy.loginfo("self.average_range: %f", self.average_range)
        if self.start_time is None:
            self.start_time = rospy.get_time()
        else:
            rospy.logwarn("start_time is false")
        current_time = rospy.get_time()
        elapsed_time = current_time - self.start_time
        self.vel.linear.x = -0.3
        self.cmd_vel_publisher.publish(self.vel)
        rospy.loginfo(elapsed_time)
        rospy.loginfo("back process")
        if elapsed_time >= 20.0:
            rospy.loginfo("elapsed_time ok")
            self.go_on_flag = False
            self.back_process_flag = False
            self.approached_box = False
            self.label_string_count = 1
            # if self.finish_flag_flag:
            #     self.finish_flag()
            #     self.finish_flag_flag = False
        else:
            rospy.loginfo("elapsed_time is false")

    def detect_result_client(self):
        
        if self.start_time is None:
            self.start_time = rospy.get_time()
        else:
            rospy.logwarn("start_time is false")
        self.current_time = rospy.get_time()
        self.elapsed_time = self.current_time - self.start_time
        rospy.loginfo("elapsed_time: %f", self.elapsed_time)
        if self.elapsed_time >= 10.0:
            rospy.wait_for_service('detect_result')
            try:
                service_call = rospy.ServiceProxy('detect_result', SetBool)
                service_call(True)
                rospy.loginfo("finish detect_result")
            except rospy.ServiceException as e:
                print ("Service call failed: %s" % e)
        else:
            rospy.loginfo("elapsed_time is false")

    # Change to accept laser_scan_msg argument
    def loop(self):
        rospy.loginfo("D1_node started")
        rospy.loginfo(self.label_msg2)
        rospy.loginfo(self.detect_box)
        rospy.loginfo(self.go_on_flag)
        if self.go_on_flag and self.detect_box:
            
        # Call lidar_send_control_commands with laser_scan_msg argument
            self.camera_send_control_commands() # Pass the appropriate laser_scan_msg
            
            # if self.label_string_count < 2 and self.detected:
            rospy.loginfo(self.width)
            rospy.loginfo(self.average_range)

            if self.width > 140 or (self.width > 30 and self.average_range < 1.0) or self.average_range < 0.45:
                if self.label_string_count < 2:
                    self.label_string()
                self.lidar_send_control_commands()  # Pass the appropriate laser_scan_msg again
                if self.approached_box:
                    self.back_process()
                    if self.label_msg2 == 'blue_box':
                        self.finish_flag()

if __name__ == '__main__':
    D1 = D1_node()
    DURATION = 0.02
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        D1.loop()
        r.sleep()