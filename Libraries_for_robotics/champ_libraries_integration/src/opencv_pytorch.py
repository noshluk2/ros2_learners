#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from torchvision.models import resnet18, ResNet18_Weights
from PIL import Image as PILImage

class TorchVisionNode(Node):
    def __init__(self):
        super().__init__('torch_vision_node')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.pub_gray  = self.create_publisher(Image,  '/image/gray', 10)
        self.pub_label = self.create_publisher(String, '/ai/label',   10)

        weights = ResNet18_Weights.DEFAULT
        self.categories = weights.meta['categories']
        self.preprocess = weights.transforms()
        self.model = resnet18(weights=weights).eval()
        self.get_logger().info('Model & transforms loaded (ResNet-18).')

    def cb(self, msg: Image):
        # to OpenCV (BGR) → grayscale → publish
        cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2GRAY)
        self.pub_gray.publish(self.bridge.cv2_to_imgmsg(gray, encoding='mono8'))

        # to PIL (RGB) → preprocess → forward → top-1 label
        cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(cv_rgb)

        with torch.no_grad():
            x = self.preprocess(pil).unsqueeze(0)           # [1,3,224,224]
            logits = self.model(x)                          # [1,1000]
            probs = logits.softmax(1)[0]
            i = int(probs.argmax())
            msg_txt = f'{self.categories[i]} {float(probs[i]):.4f}'

        self.pub_label.publish(String(data=msg_txt))

def main():
    rclpy.init()
    node = TorchVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
