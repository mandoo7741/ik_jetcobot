#!/usr/bin/env python3
import threading
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from flask import Flask, Response, redirect
from pupil_apriltags import Detector
import numpy as np

app = Flask(__name__)
frame_lock = threading.Lock()
latest_frame = None

class AprilTagNode(Node):
    def __init__(self):
        super().__init__('april_tag_node')

        self.detector = Detector(
            families="tagStandard41h12",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False
        )

        # 카메라 초기화
        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다. 경로를 확인하세요.")

        # ROS2 Publisher
        self.publisher = self.create_publisher(PoseStamped, '/april_tag_pose', 10)

        # 실제 카메라 Intrinsic 파라미터
        self.camera_matrix = np.array([
            [978.548555, 0.0, 293.112691],
            [0.0, 981.781244, 163.100487],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([-0.481485, 0.586547, -0.000026, 0.005891, 0.0])
        self.tag_size = 0.028  # 28mm (meter)

        # 주기적으로 프레임 처리
        self.timer = self.create_timer(0.05, self.process_frame)

    def process_frame(self):
        global latest_frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not received.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        if len(detections) == 0:
            return
        
        if len(detections) > 0:
            for det in detections:
                corners = np.array(det.corners, dtype=np.float32)

            # 실제 태그 3D 좌표 (단위: m)
            obj_points = np.array([
                [-self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2,  self.tag_size/2, 0],
                [-self.tag_size/2,  self.tag_size/2, 0]
            ], dtype=np.float32)

            # solvePnP로 pose 계산
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            if success:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "jetcocam"

                # 위치
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[1][0]
                pose_msg.pose.position.z = tvec[2][0]

                # 회전 (Rodrigues → Quaternion)
                R, _ = cv2.Rodrigues(rvec)
                qw = np.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2.0
                qx = (R[2,1] - R[1,2]) / (4*qw)
                qy = (R[0,2] - R[2,0]) / (4*qw)
                qz = (R[1,0] - R[0,1]) / (4*qw)

                pose_msg.pose.orientation.x = float(qx)
                pose_msg.pose.orientation.y = float(qy)
                pose_msg.pose.orientation.z = float(qz)
                pose_msg.pose.orientation.w = float(qw)

                self.publisher.publish(pose_msg)
                self.get_logger().info(f"Published pose for tag ID {det.tag_id}")


        # 최신 프레임을 Flask 스트리밍용으로 저장
        with frame_lock:
            latest_frame = frame.copy()


# ---------------- Flask Streaming ---------------- #
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

# ---------------- Main ---------------- #
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