#bu kodda statik bir şekilde color based detection yaptık. referans 
#görüntüsü veriyoruz ona göre target fotodaki distance bulunuyor
#sonraki versiyonlarda açı bilgisi de işlenecek. şu andaki performans şu
#şekilde : 100cm -> 100, 150 -> 148, 200 -> 198, 250 -> 248, 300->307

import cv2 as cv
import numpy as np

# Constants for a reference object
KNOWN_DISTANCE = 100.0  # Distance from camera to object in centimeters  in the reference image
KNOWN_WIDTH = 17.0     # Actual width of the object in centimeters
KNOWN_HEIGHT = 50.0    # Actual height of the object in centimeters

# Functions for focal length, distance, and angle estimation
def calculate_focal_length(ref_image_path, known_width, known_distance):
    # Load reference image
    ref_image = cv.imread(ref_image_path)
    if ref_image is None:
        raise ValueError("Reference image not found or unable to open.")

    # Process reference image to find object width in pixels
    ref_width_in_pixels, _, _ = find_object_dimensions_in_pixels(ref_image)
    if ref_width_in_pixels is None:
        raise ValueError("Object not found in reference image.")

    # Calculate focal length
    return (ref_width_in_pixels * known_distance) / known_width

def find_object_dimensions_in_pixels(image):
    # Define HSV color range for blue
    low_H, low_S, low_V = 100, 150, 0
    high_H, high_S, high_V = 140, 255, 255

    # Process image to find object
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, np.array([low_H, low_S, low_V]),  np.array([high_H, high_S, high_V]))
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)

    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if cnts:
        c = max(cnts, key=cv.contourArea)
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        width = np.linalg.norm(box[0] - box[1])
        height = np.linalg.norm(box[1] - box[2])
        return width, height, box
    return None, None, None

def distance_finder(focal_length, real_object_width, object_width_in_frame):
    return (real_object_width * focal_length) / object_width_in_frame

def getAngle(fittedWidth, fittedHeight, knownWidth, knownHeight):
    ratio = knownWidth / knownHeight
    w2 = fittedHeight * ratio
    angle = (1 - (fittedWidth / w2)) * 90 if w2 > fittedWidth else 0
    return angle

# Main script
# Load or capture the reference image
reference_image_path = 'renk0_100cm.jpg'  # Replace with  your reference image path

# Calculate focal length using the reference image
focal_length_value = calculate_focal_length(reference_image_path,  KNOWN_WIDTH, KNOWN_DISTANCE)

# Process the target image
target_image_path = 'renk0_250cm.jpg'  # Replace with your  target image path
target_image = cv.imread(target_image_path)
if target_image is None:
    print("Error: Target image not found or unable to open.")
    exit()

# Find object dimensions in pixels in the target image
object_width_in_pixels, object_height_in_pixels, box =  find_object_dimensions_in_pixels(target_image)
if object_width_in_pixels is not None and object_height_in_pixels is not None:
    # Estimate distance
    Distance = distance_finder(focal_length_value, KNOWN_WIDTH,  object_width_in_pixels)
    Angle = getAngle(object_width_in_pixels, object_height_in_pixels,  KNOWN_WIDTH, KNOWN_HEIGHT)
    cv.putText(target_image, f"Distance: {round(Distance, 2)} CM,  Angle: {round(Angle, 2)} degrees", (50, 50), cv.FONT_HERSHEY_SIMPLEX,  1, (255, 255, 255), 2)
    cv.drawContours(target_image, [box], 0, (0, 255, 0), 2)

cv.imshow("Processed Target Image", target_image)
cv.waitKey(0)
cv.destroyAllWindows()

