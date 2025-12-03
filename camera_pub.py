#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('gst_camera_publisher', anonymous=True)
    
    # [수정] 런치 파일에서 파라미터로 받기 (기본값은 0)
    device_id = rospy.get_param('~device_id', 0) 
    # [수정] 토픽 이름도 카메라마다 다르게 (기본값 camera)
    camera_name = rospy.get_param('~camera_name', 'camera')

    image_pub = rospy.Publisher(f'/{camera_name}/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    rospy.loginfo(f"Trying to open camera: /dev/video{device_id}")

    # GStreamer 파이프라인 (device 부분에 변수 적용)
    gst_str = f"v4l2src device=/dev/video{device_id} ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"

    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        rospy.logerr(f"카메라 /dev/video{device_id} 열기 실패! 파이프라인 확인 필요.")
        return

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = f"{camera_name}_frame"
            image_pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
