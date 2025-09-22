import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2, json, time
import numpy as np
import torch

class YOLODetector(Node):
    def __init__(self, model_name='yolov5s', conf_thres=0.3):
        super().__init__('yolo_detector')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb_image, 1)
        self.det_pub = self.create_publisher(String, '/yolo/detections', 10)
        self.annot_pub = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.bridge = CvBridge()

        # load YOLOv5 model via torch.hub (first run will download weights)
        self.get_logger().info('Loading YOLO model (this may take a while the first run)...')
        self.model = torch.hub.load('ultralytics/yolov5', model_name, pretrained=True)
        self.model.conf = conf_thres
        self.get_logger().info(f'Loaded {model_name}')

    def cb_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # run inference
        # model accepts numpy array (BGR/RGB depending on model), yolov5 expects BGR/ndarray or PIL; it handles numpy
        results = self.model(frame)  # single image
        # results.pandas().xyxy[0] is a pandas DataFrame with xmin,ymin,xmax,ymax,confidence,class,name
        df = results.pandas().xyxy[0]

        detections = []
        # Draw boxes on the frame
        for _, row in df.iterrows():
            xmin = int(row['xmin']); ymin = int(row['ymin'])
            xmax = int(row['xmax']); ymax = int(row['ymax'])
            conf = float(row['confidence'])
            cls_name = str(row['name'])
            detections.append({
                'class': cls_name,
                'confidence': conf,
                'bbox': [xmin, ymin, xmax, ymax]
            })
            # draw
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            label = f"{cls_name} {conf:.2f}"
            cv2.putText(frame, label, (xmin, max(ymin-6,0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        payload = {'timestamp': time.time(), 'detections': detections}
        self.det_pub.publish(String(data=json.dumps(payload)))

        # publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.annot_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warning(f'Could not publish annotated image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

