#!/usr/bin/env python3
"""Run YOLO on the latest ROS RGB frame, publish annotated images and JSON detections."""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('bai05_yolo_detector')
        defaults = {'image_topic': '/oakd/rgb/preview/image_raw',
                    'output_dir': str(Path.home() / 'bai05_run'), 'model': 'yolo11n.pt',
                    'device': '0', 'confidence': 0.12, 'observation_confidence': 0.012,
                    'class_aliases': '{"a red cylinder with a white stripe": "fire extinguisher"}',
                    'imgsz': 960,
                    'inference_period': 0.25, 'show_window': True}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.out = Path(self.get_parameter('output_dir').value)
        (self.out / 'detections').mkdir(parents=True, exist_ok=True)
        self.log = (self.out / 'detections.jsonl').open('a', buffering=1)
        self.bridge = CvBridge()
        self.latest = None
        self.last_stamp = None
        self.last_save = self.last_print = 0
        self.counter = 0
        self.mission = 'Loading map / AMCL'
        torch.set_num_threads(2)
        self.model = YOLO(self.get_parameter('model').value, task='detect')
        aliases = json.loads(self.get_parameter('class_aliases').value)
        # TorchScript models expose names as a read-only property. Keep display
        # aliases on the node and use them for JSON/overlay labels instead.
        if isinstance(self.model.names, dict):
            self.class_names = {int(key): aliases.get(value, value)
                                for key, value in self.model.names.items()}
        else:
            self.class_names = {index: aliases.get(value, value)
                                for index, value in enumerate(self.model.names)}
        self.device = self.get_parameter('device').value
        if self.device != 'cpu' and not torch.cuda.is_available():
            self.get_logger().warning('CUDA unavailable, using CPU')
            self.device = 'cpu'
        self.model.predict(np.zeros((480, 640, 3), dtype=np.uint8),
                           imgsz=self.get_parameter('imgsz').value, device=self.device, verbose=False)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.receive, qos)
        self.create_subscription(String, '/bai05/mission_status', self.on_status, 10)
        self.image_pub = self.create_publisher(Image, '/bai05/yolo/image', 1)
        self.json_pub = self.create_publisher(String, '/bai05/yolo/detections', 10)
        self.timer = self.create_timer(self.get_parameter('inference_period').value, self.detect)
        self.show_window = self.get_parameter('show_window').value
        if self.show_window:
            cv2.namedWindow('Bai05 - TurtleBot4 RGB + YOLO', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Bai05 - TurtleBot4 RGB + YOLO', 800, 600)
        metadata = {'model': self.get_parameter('model').value,
                    'device': self.device,
                    'image_topic': self.get_parameter('image_topic').value,
                    'confidence': self.get_parameter('confidence').value,
                    'observation_confidence': self.get_parameter('observation_confidence').value,
                    'class_aliases': aliases, 'imgsz': self.get_parameter('imgsz').value,
                    'class_names': self.class_names, 'torch': torch.__version__,
                    'camera_source': 'ROS sensor_msgs/Image from Gazebo OAK-D RGB camera'}
        (self.out / 'yolo_metadata.json').write_text(json.dumps(metadata, indent=2))
        self.get_logger().info('YOLO ready, subscribing to ' + metadata['image_topic'])

    def receive(self, msg): self.latest = msg

    def on_status(self, msg):
        info = json.loads(msg.data)
        event = info['event']
        if event == 'GOAL_SENT': self.mission = 'Navigating to Goal ' + str(info['goal'])
        elif event == 'GOAL_RESULT': self.mission = f"Goal {info['goal']}: {info['result']}"
        elif event == 'MISSION_COMPLETE': self.mission = 'MISSION COMPLETE - 3 goals'
        elif event == 'MISSION_ERROR': self.mission = 'MISSION ERROR - see navigation log'
        elif event == 'LOCALIZATION_READY': self.mission = 'AMCL / Nav2 ready'
        elif event == 'OBSERVATION_SENT': self.mission = 'Looking at extinguisher before Goal 3'
        elif event == 'OBSERVATION_RESULT': self.mission = 'Extinguisher view complete'

    def detect(self):
        msg = self.latest
        if msg is None: return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == self.last_stamp: return
        self.last_stamp = stamp
        start = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('cv_bridge conversion failed: ' + str(exc))
            return
        confidence = self.get_parameter('confidence').value
        if self.mission == 'Looking at extinguisher before Goal 3':
            confidence = self.get_parameter('observation_confidence').value
        result = self.model.predict(frame, conf=confidence,
                    imgsz=self.get_parameter('imgsz').value, device=self.device, verbose=False)[0]
        latency = (time.perf_counter()-start)*1000
        detections = [{'class_id': int(box.cls.item()),
                       'class_name': self.class_names[int(box.cls.item())],
                       'confidence': float(box.conf.item()),
                       'xyxy': [float(v) for v in box.xyxy[0].tolist()]}
                      for box in result.boxes
                      if self.class_names[int(box.cls.item())]
                      in ('fire extinguisher', 'person')]
        record = {'frame_index': self.counter, 'image_stamp': stamp,
                  'frame_id': msg.header.frame_id, 'width': msg.width, 'height': msg.height,
                  'pipeline_ms': latency, 'mission': self.mission, 'detections': detections}
        annotated = frame.copy()
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 42), (35, 35, 35), -1)
        cv2.putText(annotated, self.mission, (5, 17), cv2.FONT_HERSHEY_SIMPLEX, .37, (255,255,255), 1)
        cv2.putText(annotated, f'YOLO-World | sim {stamp:.1f}s | {latency:.1f} ms | {len(detections)} boxes',
                    (5, 34), cv2.FONT_HERSHEY_SIMPLEX, .31, (100,255,180), 1)
        # Keep labels inside the image, including boxes touching the top image edge.
        for index, det in enumerate(detections):
            label = f"{det['class_name']} {det['confidence']:.2f}"
            x = max(1, min(int(det['xyxy'][0]), frame.shape[1]-105))
            y = max(57, min(int(det['xyxy'][1])+15, frame.shape[0]-8))
            (width, height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .4, 1)
            color = {0:(255,80,0), 1:(255,220,0), 2:(0,0,255), 3:(0,220,0)}.get(det['class_id'],(255,255,255))
            x1, y1, x2, y2 = [int(v) for v in det['xyxy']]
            cv2.rectangle(annotated, (x1,y1), (x2,y2), color, 2)
            cv2.rectangle(annotated, (x,y-height-3), (min(frame.shape[1]-1,x+width+4),y+baseline),
                          (30,70,20),-1)
            cv2.putText(annotated,label,(x+2,y),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
        image_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        image_msg.header = msg.header
        self.image_pub.publish(image_msg)
        self.json_pub.publish(String(data=json.dumps(record)))
        wall = time.monotonic()
        visible_props = any(d['class_name'] in ('fire extinguisher', 'person')
                            and d['confidence'] >= 0.08 for d in detections)
        if self.counter == 0 or visible_props or (detections and wall-self.last_save >= 4):
            stem = f'frame_{self.counter:05d}_{stamp:.3f}'
            cv2.imwrite(str(self.out / 'detections' / (stem + '_raw.jpg')), frame)
            cv2.imwrite(str(self.out / 'detections' / (stem + '_yolo.jpg')), annotated)
            record['saved_stem'] = stem
            self.last_save = wall
        self.log.write(json.dumps(record) + '\n')
        if wall-self.last_print >= 2:
            self.get_logger().info(f'RGB frame {stamp:.3f}: {json.dumps(detections)}; {latency:.1f} ms')
            self.last_print = wall
        if self.show_window:
            cv2.imshow('Bai05 - TurtleBot4 RGB + YOLO', cv2.resize(annotated, (720, 540)))
            cv2.waitKey(1)
        self.counter += 1

    def close(self):
        self.log.close()
        if self.show_window: cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = YoloDetector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
