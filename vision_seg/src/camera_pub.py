#!/usr/bin/env python

import rospy, cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class CameraPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/arm/compressed', CompressedImage, queue_size=1)
        self.capture = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # 버퍼 비우기
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def publish_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            if ret:
                # 이미지를 JPEG 형식으로 압축
                _, compressed_image = cv2.imencode('.jpg', frame)
                
                # 압축된 이미지를 CompressedImage 메시지로 변환
                compressed_image_msg = CompressedImage()
                compressed_image_msg.header.stamp = rospy.Time.now()
                compressed_image_msg.format = "jpeg"  # 압축 형식을 지정 (jpeg 또는 png)
                compressed_image_msg.data = compressed_image.tobytes()

                # 압축된 이미지를 퍼블리시
                self.image_pub.publish(compressed_image_msg)

def main():
    rospy.init_node('camera_publisher')
    # CameraPublisher 인스턴스 생성
    camera_publisher = CameraPublisher()
    camera_publisher.publish_image()

if __name__ == "__main__":
    main()
