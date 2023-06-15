#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("cv_camera/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            # Menurunkan resolusi gambar
            width = 320
            height = 240

            # Mengubah gambar menjadi array numpy
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Mengubah ukuran gambar
            frame = cv2.resize(cv_image, (width, height))

        except CvBridgeError as e:
            print(e)

        # Menampilkan gambar pada jendela
        cv2.imshow("Image window", frame)
        cv2.waitKey(1)

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    ic = ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
