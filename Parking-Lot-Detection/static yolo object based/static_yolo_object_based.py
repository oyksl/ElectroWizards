import cv2 as cv
import numpy as np
import torch
import pathlib
import matplotlib.pyplot as plt
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO('yolov8m.pt')

# Load and resize the input image
input_image_path = 'stop15_100cm.jpg'
input_image = cv.imread(input_image_path)
input_image_resized = cv.resize(input_image, (2252, 4000))  # Resize to 640x480 pixels

results = model.predict(input_image_resized)

# Extracting bounding box coordinates
result = results[0]

class_id= result.boxes.cls

ind = 0

for i in class_id:
    if i == 11:
        box_coordinates = result.boxes.xyxy[ind]
        object_width_in_pixels = box_coordinates[2] - box_coordinates[0]
        object_height_in_pixels = box_coordinates[3] - box_coordinates[1]
        ind+= 1
    else:
        ind+= 1
        
# Draw bounding box
cv.rectangle(input_image_resized, (int(box_coordinates[0]), int(box_coordinates[1])), (int(box_coordinates[2]), int(box_coordinates[3])), (255,0,0), 10)

# Constants for a reference object
KNOWN_DISTANCE = 100  # Distance from camera to object in centimeters in the reference image
KNOWN_WIDTH = 13   # Actual width of the object in centimeters
KNOWN_HEIGHT = 13 # Actual height of the object in centimeters

# Functions for focal length, distance, and angle estimation
def calculate_focal_length(ref_image_path, known_width, known_distance):
    ref_image = cv.imread(ref_image_path)
    ref_image_resized = cv.resize(ref_image, (2252, 4000))
    ref_results = model.predict(ref_image_resized)
    ref_class_id = ref_results[0].boxes.cls
    ind = 0
    for i in ref_class_id:
        if i == 11:
            ref_box = ref_results[0].boxes.xyxy[ind]
            object_width_in_pixels = ref_box[2] - ref_box[0]
            object_height_in_pixels = ref_box[3] - ref_box[1]
            ind+= 1
        else:
            ind += 1

    # Calculate width in pixels
    ref_width_in_pixels = ref_box[2] - ref_box[0]
    return (ref_width_in_pixels * known_distance) / known_width

# def distance_finder(focal_length, real_object_width, object_width_in_frame):
    # return (real_object_width * focal_length) / object_width_in_frame
    

def distance_finder(focal_length, real_object_height, object_height_in_pixel, real_object_width,object_width_in_pixel):
    return (((real_object_height*focal_length)/object_height_in_pixels) +((real_object_width*focal_length)/object_width_in_pixels)) / 2
    
    
def getAngle(fittedWidth, fittedHeight, knownWidth, knownHeight):
    ratio = knownWidth / knownHeight
    w2 = fittedHeight * ratio
    angle = (1 - (fittedWidth / w2)) * 190 if w2 > fittedWidth else 0  
    angle = (angle -35)/52*45+35
    return angle

# Main script
# Load or capture the reference image
reference_image_path = 'stop0_100cm.jpg'

# Calculate focal length using the reference image
focal_length_value = calculate_focal_length(reference_image_path, KNOWN_WIDTH, KNOWN_DISTANCE)

# Estimate distance and angle
Distance = distance_finder(focal_length_value, KNOWN_HEIGHT, object_height_in_pixels,KNOWN_WIDTH, object_width_in_pixels)
Angle = getAngle(object_width_in_pixels, object_height_in_pixels, KNOWN_WIDTH, KNOWN_HEIGHT)

# Convert Tensors to floats and round
Distance = Distance.item() if torch.is_tensor(Distance) else Distance
Angle = Angle.item() if torch.is_tensor(Angle) else Angle

distance_text = f"Distance: {round(Distance, 2)} CM"
angle_text = f"Angle: {round(Angle, 5)} degrees"

# Add text to the image
cv.putText(input_image_resized, f"{distance_text}, {angle_text}", (100, 100), cv.FONT_HERSHEY_SIMPLEX, 2, (255  , 255,  255), 10)


# Save the processed image
output_image_path = 'detected2_stop15_100cm.jpg'
cv.imwrite(output_image_path, input_image_resized)


cv.imshow("Processed Target Image", input_image_resized)
cv.waitKey(0)
cv.destroyAllWindows()
