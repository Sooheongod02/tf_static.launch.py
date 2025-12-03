#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('gst_camera_node', anonymous=True)
    
    # [핵심] 런치 파일에서 넘겨주는 파라미터 받기 (기본값: 0번, 이름 camera)
    device_id = rospy.get_param('~device_id', 0)
    topic_name = rospy.get_param('~camera_name', 'camera')

    # 토픽 이름: /cam_front/image_raw 등으로 송출
    pub = rospy.Publisher(f'/{topic_name}/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    rospy.loginfo(f"[{topic_name}] Opening /dev/video{device_id} ...")

    # GStreamer 파이프라인 (해상도/FPS 조절 필요시 여기서 width, height 수정)
    # USB 대역폭 문제 방지를 위해 해상도를 640x480으로 낮춤
    gst_str = (
        f"v4l2src device=/dev/video{device_id} ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    )

    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        rospy.logerr(f"[{topic_name}] 카메라 열기 실패! (/dev/video{device_id})")
        return

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = f"{topic_name}_link"
            pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
