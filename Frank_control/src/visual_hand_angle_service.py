#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from Frank_control.srv import GetHandAngles, GetHandAnglesResponse
import cv2
import mediapipe as mp
import numpy as np
import threading
import yaml
import time
# 你需要将orbbec_camera.py等依赖文件也放到合适位置
# from camera.orbbec_camera import OrbbecCamera, get_serial_numbers, close_connected_cameras

def load_config(config_path='config.yaml'):
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        angle_map_config = config.get('angle_mapping', {})
        mediapipe_config = config.get('mediapipe_hands_settings', {})
        orbbec_config = config.get('orbbec_camera_settings', {})
        return {
            'VISUAL_THUMB_BEND_INPUT_MIN': angle_map_config.get('visual_thumb_bend_input_min', 0),
            'VISUAL_THUMB_BEND_INPUT_MAX': angle_map_config.get('visual_thumb_bend_input_max', 65),
            'VISUAL_OTHER_FINGERS_BEND_INPUT_MIN': angle_map_config.get('visual_other_fingers_bend_input_min', 0),
            'VISUAL_OTHER_FINGERS_BEND_INPUT_MAX': angle_map_config.get('visual_other_fingers_bend_input_max', 65),
            'VISUAL_THUMB_ROT_INPUT_MIN': angle_map_config.get('visual_thumb_rot_input_min', -90),
            'VISUAL_THUMB_ROT_INPUT_MAX': angle_map_config.get('visual_thumb_rot_input_max', 90),
            'MP_MIN_DETECTION_CONFIDENCE': mediapipe_config.get('min_detection_confidence', 0.8),
            'MP_MIN_TRACKING_CONFIDENCE': mediapipe_config.get('min_tracking_confidence', 0.5),
            'MP_MAX_NUM_HANDS': mediapipe_config.get('max_num_hands', 1),
            'ORBBEC_CAMERA_SERIAL_NUMBER': orbbec_config.get('serial_number', ''),
        }
    except Exception as e:
        print(f"加载配置失败: {e}")
        return {}

def translate_angle(value, visual_min, visual_max, ohand_min, ohand_max):
    if visual_max == visual_min:
        return ohand_min if value <= visual_min else ohand_max
    visual_span = visual_max - visual_min
    ohand_span = ohand_max - ohand_min
    value_scaled = float(value - visual_min) / float(visual_span)
    translated_value = ohand_min + (value_scaled * ohand_span)
    return max(min(translated_value, ohand_max), ohand_min)

def compute_thumb_rotation_angle(hand_landmarks):
    WRIST = 0
    THUMB_CMC = 1
    THUMB_MCP = 2
    THUMB_TIP = 4
    INDEX_MCP = 5
    if not hand_landmarks:
        return None
    vec_mcp_tip_2d = np.array([hand_landmarks.landmark[THUMB_TIP].x, hand_landmarks.landmark[THUMB_TIP].y]) - \
                     np.array([hand_landmarks.landmark[THUMB_MCP].x, hand_landmarks.landmark[THUMB_MCP].y])
    vec_wrist_index_mcp_2d = np.array([hand_landmarks.landmark[INDEX_MCP].x, hand_landmarks.landmark[INDEX_MCP].y]) - \
                             np.array([hand_landmarks.landmark[WRIST].x, hand_landmarks.landmark[WRIST].y])
    if np.linalg.norm(vec_mcp_tip_2d) == 0 or np.linalg.norm(vec_wrist_index_mcp_2d) == 0:
        return 0
    dot_product = np.dot(vec_mcp_tip_2d, vec_wrist_index_mcp_2d)
    norm_product = np.linalg.norm(vec_mcp_tip_2d) * np.linalg.norm(vec_wrist_index_mcp_2d)
    value_for_acos = np.clip(dot_product / norm_product, -1.0, 1.0)
    angle_rad = np.arccos(value_for_acos)
    angle_deg = np.degrees(angle_rad)
    cross_product = vec_mcp_tip_2d[0] * vec_wrist_index_mcp_2d[1] - vec_mcp_tip_2d[1] * vec_wrist_index_mcp_2d[0]
    if cross_product < 0:
        angle_deg = -angle_deg
    return int(angle_deg)

def compute_finger_angles_from_visual(results, joint_list, config):
    visual_bend_angles = []
    visual_thumb_rot_angle = 0
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            for i, joint_indices in enumerate(joint_list):
                a_coords = np.array([
                    hand_landmarks.landmark[joint_indices[0]].x, hand_landmarks.landmark[joint_indices[0]].y])
                b_coords = np.array([
                    hand_landmarks.landmark[joint_indices[1]].x, hand_landmarks.landmark[joint_indices[1]].y])
                c_coords = np.array([
                    hand_landmarks.landmark[joint_indices[2]].x, hand_landmarks.landmark[joint_indices[2]].y])
                radians = np.arctan2(c_coords[1] - b_coords[1], c_coords[0] - b_coords[0]) - \
                          np.arctan2(a_coords[1] - b_coords[1], a_coords[0] - b_coords[0])
                angle_deg = np.abs(radians * 180.0 / np.pi)
                if angle_deg > 180.0:
                    angle_deg = 360.0 - angle_deg
                processed_angle = 0
                if i == 0:
                    clamped_angle_deg = max(90, min(angle_deg, 180))
                    processed_angle = np.interp(clamped_angle_deg, [90, 180], [0, 200])
                    processed_angle = min(config['VISUAL_THUMB_BEND_INPUT_MAX'], processed_angle)
                    processed_angle = max(config['VISUAL_THUMB_BEND_INPUT_MIN'], processed_angle)
                else:
                    clamped_angle_deg = max(30, min(angle_deg, 180))
                    processed_angle = np.interp(clamped_angle_deg, [30, 180], [0, 180])
                    processed_angle = min(config['VISUAL_OTHER_FINGERS_BEND_INPUT_MAX'], processed_angle)
                    processed_angle = max(config['VISUAL_OTHER_FINGERS_BEND_INPUT_MIN'], processed_angle)
                visual_bend_angles.append(int(processed_angle))
            temp_rot_angle = compute_thumb_rotation_angle(hand_landmarks)
            if temp_rot_angle is not None:
                visual_thumb_rot_angle = temp_rot_angle
            break
    return visual_bend_angles, visual_thumb_rot_angle

class HandAngleServiceNode(object):
    def __init__(self):
        self.config = load_config('../config.yaml')
        self.latest_angles = [0, 0, 0, 0, 0, 0]  # [thumb, index, middle, ring, pinky, thumb_rot]
        self.hand_detected = False
        self.lock = threading.Lock()
        self.running = True
        self.joint_list = [
            [4, 3, 2],
            [8, 6, 5],
            [12, 10, 9],
            [16, 14, 13],
            [20, 18, 17]
        ]
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=self.config['MP_MAX_NUM_HANDS'],
            min_detection_confidence=self.config['MP_MIN_DETECTION_CONFIDENCE'],
            min_tracking_confidence=self.config['MP_MIN_TRACKING_CONFIDENCE'])
        self.thread = threading.Thread(target=self.capture_loop)
        self.thread.start()
        self.service = rospy.Service('get_hand_angles', GetHandAngles, self.handle_get_hand_angles)
        rospy.loginfo('Hand angle service ready.')

    def capture_loop(self):
        cap = cv2.VideoCapture(0)
        while self.running and not rospy.is_shutdown():
            ret, color_image = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            image = cv2.flip(image, 1)
            image.flags.writeable = False
            results = self.hands.process(image)
            image.flags.writeable = True
            visual_bend_angles, visual_thumb_rot_angle = compute_finger_angles_from_visual(results, self.joint_list, self.config)
            with self.lock:
                if len(visual_bend_angles) == 5:
                    self.latest_angles = [
                        float(visual_bend_angles[0]),
                        float(visual_bend_angles[1]),
                        float(visual_bend_angles[2]),
                        float(visual_bend_angles[3]),
                        float(visual_bend_angles[4]),
                        float(visual_thumb_rot_angle)
                    ]
                    self.hand_detected = True
                else:
                    self.hand_detected = False
            time.sleep(0.03)
        cap.release()

    def handle_get_hand_angles(self, req):
        with self.lock:
            resp = GetHandAnglesResponse()
            resp.thumb_bend = self.latest_angles[0]
            resp.index_bend = self.latest_angles[1]
            resp.middle_bend = self.latest_angles[2]
            resp.ring_bend = self.latest_angles[3]
            resp.pinky_bend = self.latest_angles[4]
            resp.thumb_rot = self.latest_angles[5]
            resp.detected = self.hand_detected
            return resp

    def shutdown(self):
        self.running = False
        self.thread.join()

if __name__ == '__main__':
    rospy.init_node('visual_hand_angle_service')
    node = HandAngleServiceNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin() 