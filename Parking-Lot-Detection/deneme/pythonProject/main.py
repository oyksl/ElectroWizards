import math
import cv2
import numpy as np


img = cv2.imread("duvar.jpg")
img = cv2.resize(img,(600,600))
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
equ = cv2.equalizeHist(gray)
blur = cv2.GaussianBlur(equ,(5,5),0)
dst = cv2.Canny(blur, 50, 200, None, 3)
ret, thresh = cv2.threshold(blur, 190, 255, cv2.THRESH_BINARY) # Find Contours
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Draw contours
# Use the original image or a copy for drawing contours if you want to preserve the original
cv2.drawContours(img, contours, -1, (255, 0, 0), 3)  # Draw contours on the original/resized image

drawing = np.zeros(img.shape, np.uint8)

lines = cv2.HoughLinesP(dst, cv2.HOUGH_PROBABILISTIC, np.pi/180, 25, minLineLength = 10, maxLineGap = 40)

# Draw the lines
if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        cv2.line(dst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)


# Display the image (optional, for debugging)
#print(lines)
cv2.imshow("Contours", dst)
cv2.waitKey(0)
cv2.destroyAllWindows()