# -*- coding: utf-8 -*-

import os
import random
import math
from controller import Supervisor
import cv2
import numpy as np

NUM_IMAGES_TO_GENERATE = 20000
DATASET_BASE_PATH = os.path.expanduser('~/webots_ws_new/src/mavic2_webots/dataset/')
MIN_CONTOUR_AREA = 0.4
MAX_SIMULATION_STEPS = 2000000

images_dir = os.path.join(DATASET_BASE_PATH, 'images_2')
labels_dir = os.path.join(DATASET_BASE_PATH, 'labels_2')
os.makedirs(images_dir, exist_ok=True)
os.makedirs(labels_dir, exist_ok=True)


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

target_drone_node = robot.getFromDef("TARGET_DRONE")
if not target_drone_node:
    print("FATAL ERROR: Could not find the target drone. Make sure its DEF name is set to 'TARGET_DRONE'.")
    exit()

camera = robot.getDevice('camera')
camera.enable(timestep)

saved_image_count = 15240
print(f"Starting data generation. Target: {NUM_IMAGES_TO_GENERATE} images.")
print(f"Dataset will be saved in: {DATASET_BASE_PATH}")

for i in range(MAX_SIMULATION_STEPS):

    follower_node = robot.getSelf()
    if follower_node is None:
        print("Could not get the supervisor node. Aborting.")
        break
    follower_position = follower_node.getPosition()

    MIN_DISTANCE = 5.5
    MAX_DISTANCE = 13.0
    
    relative_distance = random.uniform(MIN_DISTANCE, MAX_DISTANCE)

    relative_angle = random.uniform(-math.pi/4, -3*math.pi/4)

    target_x = follower_position[0] + relative_distance * math.cos(relative_angle)
    target_y = follower_position[1] + relative_distance * math.sin(relative_angle)
    
    target_z = random.uniform(0.5, 9.0)
    target_yaw = random.uniform(-math.pi, math.pi)

    target_drone_node.getField("translation").setSFVec3f([target_x, target_y, target_z])
    target_drone_node.getField("rotation").setSFRotation([0, 0, 1, target_yaw])

    if robot.step(timestep) == -1:
        break

    frame_raw = camera.getImage()
    if frame_raw:
        frame_bgr = np.frombuffer(frame_raw, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))[:, :, :3]

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 150, 150])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)

            print(f"Contour detected with area: {cv2.contourArea(largest_contour)}")

            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                x_b, y_b, w, h = cv2.boundingRect(largest_contour)
                
                image_path = os.path.join(images_dir, f'image_{saved_image_count}.png')
                label_path = os.path.join(labels_dir, f'image_{saved_image_count}.txt')
                
                cv2.imwrite(image_path, frame_bgr)
                
                x_center_norm = (x_b + w / 2) / camera.getWidth()
                y_center_norm = (y_b + h / 2) / camera.getHeight()
                width_norm = w / camera.getWidth()
                height_norm = h / camera.getHeight()
                
                with open(label_path, 'w') as f:
                    f.write(f"0 {x_center_norm:.6f} {y_center_norm:.6f} {width_norm:.6f} {height_norm:.6f}\n")
                    
                saved_image_count += 1

    if (i + 1) % 500 == 0:
        print(f"Simulation step {i + 1}/{MAX_SIMULATION_STEPS}... Found and saved {saved_image_count} images so far.")
    
    if saved_image_count >= NUM_IMAGES_TO_GENERATE:
        print(f"Successfully generated {saved_image_count} images. Target met.")
        break

print(f"Data generation complete. Total images saved: {saved_image_count}")
robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)

