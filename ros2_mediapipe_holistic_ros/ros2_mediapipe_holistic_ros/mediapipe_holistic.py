
#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros2_mediapipe_holistic_ros_msg.msg import MediaPipeHolistic, MediaPipePose
import mediapipe as mp


PARAMS = {
    "output_image_topic": "/mediapipe_holistic/output_image",
    "output_mediapipe_topic": "/mediapipe_holistic/landmarks",
    "pub_landmark_output": True,
    "pub_image_output": False,
    "view_image": True,
    "image_width": 640,
    "image_height": 480
}

class HolisticMediaPipe(Node):

    def __init__(self):
        super().__init__('holistic_mediapipe')
        self.publisher_output_image = self.create_publisher(Image, PARAMS["output_image_topic"], 10)
        self.publisher_output_mediapipe = self.create_publisher(MediaPipeHolistic, PARAMS["output_mediapipe_topic"], 10)
        self.mp_holistic = mp.solutions.holistic
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, PARAMS["image_width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PARAMS["image_height"])

    def process_image(self, image):
        with self.mp_holistic.Holistic(
                enable_segmentation=False,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as holistic:
            image.flags.writeable = False
            results = holistic.process(image)
            return results

    def draw_landmarks(self, image, results):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert from BGR to RGB
        self.mp_drawing.draw_landmarks(
            image, results.face_landmarks, self.mp_holistic.FACEMESH_CONTOURS, landmark_drawing_spec=None,
            connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style())
        self.mp_drawing.draw_landmarks(
            image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
        self.mp_drawing.draw_landmarks(
            image, results.right_hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
            self.mp_drawing_styles.get_default_hand_landmarks_style(),
            self.mp_drawing_styles.get_default_hand_connections_style())
        self.mp_drawing.draw_landmarks(
            image, results.left_hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
            self.mp_drawing_styles.get_default_hand_landmarks_style(),
            self.mp_drawing_styles.get_default_hand_connections_style())
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert back from RGB to BGR
        return image

    def publish_results(self, results):
        if PARAMS["pub_landmark_output"]:
            landmarks = MediaPipeHolistic()

            if results.face_landmarks:
                self.populate_landmarks(results.face_landmarks, landmarks.face_landmarks)

            if results.left_hand_landmarks:
                self.populate_landmarks(results.left_hand_landmarks, landmarks.left_hand_landmarks)

            if results.right_hand_landmarks:
                self.populate_landmarks(results.right_hand_landmarks, landmarks.right_hand_landmarks)

            if results.pose_landmarks:
                self.populate_landmarks(results.pose_landmarks, landmarks.pose_landmarks)

            if results.pose_world_landmarks:
                self.populate_landmarks(results.pose_world_landmarks, landmarks.pose_world_landmarks)

            self.publisher_output_mediapipe.publish(landmarks)

    def populate_landmarks(self, landmarks_data, landmarks_msg):
        for i, landmark in enumerate(landmarks_data.landmark):
            pose = MediaPipePose()
            pose.id = i
            pose.x = landmark.x
            pose.y = landmark.y
            pose.z = landmark.z
            pose.visibility = landmark.visibility
            landmarks_msg.append(pose)

    def main(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error reading frame from camera.")
                break

            results = self.process_image(frame)
            if PARAMS["pub_image_output"] or PARAMS["view_image"]:
                frame = self.draw_landmarks(frame, results)
                if PARAMS["view_image"]:
                    cv2.imshow('Operator Image', cv2.flip(frame, 1))
                    if cv2.waitKey(5) & 0xFF == 27:
                        print("OK")
            self.publish_results(results)

def main(args=None):
    rclpy.init(args=args)
    node = HolisticMediaPipe()
    node.main()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
