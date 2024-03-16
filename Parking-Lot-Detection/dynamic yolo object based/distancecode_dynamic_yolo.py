import cv2 as cv
import numpy as np
import torch
from ultralytics import YOLO
from picamera2 import Picamera2

# Constants for a reference object
KNOWN_DISTANCE = 100  # Distance from camera to object in centimeters in the reference image
KNOWN_WIDTH = 13   # Actual width of the object in centimeters
KNOWN_HEIGHT = 13 # Actual height of the object in centimeters

# Functions for focal length, distance, and angle estimation
def calculate_focal_length(ref_image_path, known_width, known_distance, model):
    ref_image = cv.imread(ref_image_path)
    ref_image_resized = cv.resize(ref_image, (2252, 4000))
    ref_results = model.predict(ref_image_resized)
    ref_class_id = ref_results[0].boxes.cls
    for ind, i in enumerate(ref_class_id):
        if i == 11:  # Assuming 11 is the class ID for stop signs
            ref_box = ref_results[0].boxes.xyxy[ind]
            ref_width_in_pixels = ref_box[2] - ref_box[0]
            return (ref_width_in_pixels * known_distance) / known_width
    return None

def distance_finder(focal_length, real_object_width, object_width_in_frame):
    return (real_object_width * focal_length) / object_width_in_frame

def getAngle(fittedWidth, fittedHeight, knownWidth, knownHeight):
    ratio = knownWidth / knownHeight
    w2 = fittedHeight * ratio
    angle = (1 - (fittedWidth / w2)) * 90 if w2 > fittedWidth else 0
    return angle

# Load YOLOv8 model
model = YOLO('yolov8m.pt')  # Ensure you have the correct path to the model

# Load or capture the reference image
reference_image_path = 'stop0_100cm.jpg'  # Ensure you have this image in the correct path

# Calculate focal length using the reference image
focal_length_value = calculate_focal_length(reference_image_path, KNOWN_WIDTH, KNOWN_DISTANCE, model)
if focal_length_value is None:
    raise ValueError("Reference object not found in the reference image.")

# Initialize Picamera2
camera = Picamera2()
preview_config = camera.create_preview_configuration(main={"size": (2252, 4000)})
camera.configure(preview_config)
camera.start()

while True:
    # Capture the frame
    frame = camera.capture_array()

    # Check and convert the image to 3 channels if necessary
    if len(frame.shape) == 3 and frame.shape[2] == 4:  # Check if the image has 4 channels
        frame = cv.cvtColor(frame, cv.COLOR_RGBA2RGB)  # Convert from RGBA to RGB
    elif len(frame.shape) == 2:  # Check if the image is grayscale
        frame = cv.cvtColor(frame, cv.COLOR_GRAY2RGB)  # Convert from grayscale to RGB

    # Resize the frame
    frame_resized = cv.resize(frame, (2252, 4000))

    # Object detection
    results = model.predict(frame_resized)

    # Extracting bounding box coordinates
    result = results[0]
    class_id = result.boxes.cls

    for ind, cls_id in enumerate(class_id):
        if cls_id == 11:  # Assuming 11 is the class ID for stop signs
            box_coordinates = result.boxes.xyxy[ind]
            object_width_in_pixels = box_coordinates[2] - box_coordinates[0]
            object_height_in_pixels = box_coordinates[3] - box_coordinates[1]

            # Draw bounding box
            cv.rectangle(frame_resized, (int(box_coordinates[0]), int(box_coordinates[1])), 
                         (int(box_coordinates[2]), int(box_coordinates[3])), (255, 0, 0), 2)

            # Estimate distance and angle
            Distance = distance_finder(focal_length_value, KNOWN_WIDTH, object_width_in_pixels)
            Angle = getAngle(object_width_in_pixels, object_height_in_pixels, KNOWN_WIDTH, KNOWN_HEIGHT)

            # Convert Tensors to floats and round
            Distance = Distance.item() if torch.is_tensor(Distance) else Distance
            Angle = Angle.item() if torch.is_tensor(Angle) else Angle

            distance_text = f"Distance: {round(Distance, 2)} CM"
            angle_text = f"Angle: {round(Angle, 2)} degrees"

            # Add text to the frame
            cv.putText(frame_resized, distance_text, (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 10)
            cv.putText(frame_resized, angle_text, (50, 200), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 10)

    # Display the processed frame
    cv.imshow("Processed Target Image", frame_resized)

    # Break the loop when 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

camera.stop()
cv.destroyAllWindows()
