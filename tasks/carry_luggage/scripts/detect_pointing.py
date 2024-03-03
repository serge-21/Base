#!/usr/bin/env python3

import cv2
import rospy
import mediapipe as mp
from sensor_msgs.msg import Image
from lasr_vision_msgs.srv import YoloDetectionRequest, YoloDetectionResponse, YoloDetection
from carry_luggage.srv import PointingService, PointingServiceResponse, PointingServiceRequest

'''using this rather than openpose because it is faster and more accurate'''
class PointingDetecor:
    def __init__(self):
        self.detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.service = rospy.Service('pointing_detection_service', PointingService, self.excute)

        # Load MediaPipe Pose model
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()

    def excute(self):
        resp = PointingServiceResponse()
        people, img = self.detection()

        if people:
            img_width = img.width
            img_height = img.height
            for person in people:
                keypoints = self.detect_keypoints(img)  # Detect keypoints using MediaPipe
                direction = self.determine_pointing_direction(person, img_width, img_height, keypoints)
                print("Person detected pointing:", direction)

                resp.direction = direction
        
        resp.direction = "Err"
        return resp

    def detect_keypoints(self, img):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.pose.process(img_rgb)
        
        keypoints = []
        if results.pose_landmarks:
            for landmark in results.pose_landmarks.landmark:
                x = int(landmark.x * img.width)
                y = int(landmark.y * img.height)
                keypoints.append((x, y))

        return keypoints
    
    def detection(self):
        msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)

        request = YoloDetectionRequest()
        request.image_raw = msg                # sensor_msgs/Image
        request.dataset = "yolov8n-seg.pt"     # YOLOv8 model, auto-downloads
        request.confidence = 0.7               # minimum confidence to include in results
        request.nms = 0.4                      # non maximal supression

        # send request
        response = self.detect_service(request)
        
        result = []
        for detection in response.detected_objects:
            if detection.name == "person":
                # cords of person in image
                result.appent(detection.xywh)

        return result, msg

    def determine_pointing_direction(self, person_bbox, img_width, img_height, keypoints):
        x, y, w, h = person_bbox
        # Calculate the center of the person's bounding box
        center_x = x + w // 2
        
        # Ensure keypoints are available
        if len(keypoints) >= 7:  # Ensure we have at least 7 keypoints for the upper body
            # Extract relevant keypoints for shoulders and wrists
            left_shoulder = keypoints[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
            right_shoulder = keypoints[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
            left_wrist = keypoints[self.mp_pose.PoseLandmark.LEFT_WRIST.value]
            right_wrist = keypoints[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]
            
            # Ensure all keypoints are detected
            if left_shoulder and right_shoulder and left_wrist and right_wrist:
                # Calculate the x-coordinate difference between shoulders and wrists
                left_diff = left_wrist[0] - left_shoulder[0]
                right_diff = right_shoulder[0] - right_wrist[0]
                
                # Determine pointing direction based on the difference in x-coordinates
                if left_diff > right_diff:
                    return "Left"
                elif right_diff > left_diff:
                    return "Right"
        
        # Default: Determine direction based on the relative position to the center of the image
        if center_x < img_width // 3:
            return "Left"
        elif center_x > 2 * img_width // 3:
            return "Right"
        else:
            return "Front"

if __name__ == '__main__':
    rospy.init_node("pointing_detector")
    pointer = PointingDetecor()
    rospy.loginfo("Pointing Detector is running")
    rospy.spin()