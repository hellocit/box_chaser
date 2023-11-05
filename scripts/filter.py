#!/usr/bin/env python3
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import Header, String, Bool
from std_srvs.srv import SetBool, SetBoolResponse

# デフォルトで検出が行われていないと仮定
detect_box = False
# ラベル情報を格納するリストを初期化
labels = []

# フィルタリングする文字列のリスト
filter_strings = ['tag','green_box','blue_box']

start_time = None
detect_result_flag = True

detected = False  # 検出ステータスを初期化

detect_box_2 = False

detect_result_flag_flag = False

# コールバック関数
def boundingBoxesCallback(data):
    global detected  # グローバル変数を変更するために必要
    global labels
    global start_time
    global detect_result_flag
    global detect_box
    if detect_box_2:
        # ラベル情報を取得
        labels = [bbox.Class for bbox in data.bounding_boxes]
        # 検出が行われたかどうかを設定
        detected = bool(labels)
    else:
        rospy.loginfo("detect_box is False")
        rospy.loginfo(detect_box_2)

# def detect_box_srv_filter(data):
#     resp = SetBoolResponse()
#     global detect_box
#     if data.data == True:
#         resp.message = "called"
#         resp.success = True
#         detect_box = True
#     else:
#         resp.message = "ready"
#         resp.success = False
#         detect_box = False
#     print(resp.message)
#     return resp

def detect_box_srv2(data):
    resp = SetBoolResponse()
    global detect_box_2
    if data.data == True:
        resp.message = "called"
        resp.success = True
        detect_box_2 = True
    else:
        resp.message = "ready"
        resp.success = False
        detect_box_2 = False
    print(resp.message)
    return resp

def detect_result_flag_flag_server(data):
    resp = SetBoolResponse()
    global detect_result_flag_flag
    if data.data == True:
        resp.message = "called"
        resp.success = True
        detect_result_flag_flag = True
    else:
        resp.message = "ready"
        resp.success = False
        detect_result_flag_flag = False
    print(resp.message)
    return resp


if __name__ == '__main__':
    rospy.init_node('filter_subscriber')

    # メッセージのパブリッシャーを作成
    label_publisher = rospy.Publisher('/label_string', String, queue_size=1)
    # srv = rospy.Service('detect_box', SetBool, detect_box_srv_filter)
    rospy.Service('detect_box_2', SetBool, detect_box_srv2)
    rospy.Service('detect_result_flag', SetBool, detect_result_flag_flag_server)
    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, boundingBoxesCallback)

    rate = rospy.Rate(10)  # パブリッシュの頻度を設定（10Hzを意味します）

    while not rospy.is_shutdown():
        rospy.loginfo("labels: {}".format(labels))
        # フィルタリング
        filtered_labels = list(filter(lambda label: label not in filter_strings, labels))
        
        # ラベル情報を結合してスペースで区切った文字列に変換
        label_str = ' '.join(filtered_labels)
        
        # 新しいメッセージを作成
        label_msg = String()
        label_msg.data = label_str

        # 検出ステータスを新しいメッセージに設定
        detection_msg = Bool()
        detection_msg.data = detected

        # ラベル情報と検出ステータスをパブリッシュ
        label_publisher.publish(label_msg)

        if start_time is None:
            start_time = rospy.get_time()  # Start the timer when desired_distance is reached
        else:
            rospy.logwarn("start_time is false")
        current_time = rospy.get_time()
        elapsed_time = current_time - start_time
        rospy.loginfo("elapsed_time: %f", elapsed_time)
        
        if elapsed_time >= 10.0 and detect_result_flag_flag == True:
        # if elapsed_time >= 20.0:
            rospy.wait_for_service('detect_result')
            try:
                service_call = rospy.ServiceProxy('detect_result', SetBool)
                service_call(True)
                rospy.loginfo("finish detect_result")
                detect_result_flag = False
            except rospy.ServiceException as e:
                print ("Service call failed: %s" % e)
        else:
            rospy.loginfo("elasped_time is false")
        rate.sleep()