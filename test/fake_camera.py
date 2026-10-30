# TASK 11: Dry-run helper — publishes a still image as /image_raw (rgb8, fresh stamps) when no camera is attached.
"""Usage: python3 fake_camera.py IMAGE [rate_hz]. Not used in the real mission."""
import sys
import cv2
import rclpy
from sensor_msgs.msg import Image


def main():
    img = cv2.resize(cv2.imread(sys.argv[1]), (640, 480))[:, :, ::-1].copy()
    rclpy.init(); n = rclpy.create_node('fake_camera')
    pub = n.create_publisher(Image, '/image_raw', 1)
    def tick():
        m = Image(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = 'camera_link'
        m.height, m.width = img.shape[:2]; m.encoding = 'rgb8'; m.step = 3*m.width; m.data = img.tobytes()
        pub.publish(m)
    n.create_timer(1./float(sys.argv[2] if len(sys.argv) > 2 else 5.), tick)
    rclpy.spin(n)


if __name__ == '__main__': main()
