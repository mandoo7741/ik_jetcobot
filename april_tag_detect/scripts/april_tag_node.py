#!/usr/bin/env python3
import threading
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from flask import Flask, Response, redirect
import apriltag


app = Flask(__name__)
frame_lock = threading.Lock()
latest_frame = None

class AprilTagNode(Node):
    def __init__(self):
        super().__init__('april_tag_node')

        # 카메라 초기화
        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        self.detector = apriltag.Detector()
        self.publisher = self.create_publisher(PoseStamped, '/april_tag_pose', 10)

        # Intrinsic 파라미터 (예시값)
        self.fx = 978.5
        self.fy = 981.7
        self.cx = 293.1
        self.cy = 163.1
        self.tag_size = 0.028  # 28mm

        self.timer = self.create_timer(0.05, self.process_frame)

    def process_frame(self):
        global latest_frame
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        for r in results:
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"ID: {r.tag_id}", (cX-20, cY-20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 간단한 Z 추정
            z = (self.fx * self.tag_size) / (r.corners[1][0] - r.corners[0][0])
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "camera_link"
            pose.pose.position.x = (cX - self.cx) * z / self.fx
            pose.pose.position.y = (cY - self.cy) * z / self.fy
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            self.publisher.publish(pose)

        with frame_lock:
            latest_frame = frame.copy()

def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                continue
            ret, buffer = cv2.imencode('.jpg', latest_frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return redirect('/stream')

@app.route('/stream')
def stream():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()

    t = threading.Thread(target=flask_thread, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
