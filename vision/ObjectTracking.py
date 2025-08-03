import cv2
import numpy as np

# Load your images
image1 = cv2.imread('../i_1.jpg')
image2 = cv2.imread('../i_2.jpg')

# Assume `detections1` and `detections2` hold YOLO results for each image
# You need to extract keypoints for each object detection in both images
# For simplicity, let's assume (x, y) is the center of a detected object

# Find keypoints and descriptors with SIFT
sift = cv2.SIFT_create()
keypoints1, descriptors1 = sift.detectAndCompute(image1, None)
keypoints2, descriptors2 = sift.detectAndCompute(image2, None)

# FLANN based Matcher
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)   # or pass an empty dictionary

flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(descriptors1, descriptors2, k=2)

# Filter matches using the Lowe's ratio test
good_matches = []
for m, n in matches:
    if m.distance < 0.7*n.distance:
        good_matches.append(m)

# Assuming you have the camera intrinsic parameters and the relative pose
# between image1 and image2, you can now triangulate points to get 3D coordinates
# (This part highly depends on your camera setup and calibration)

# Note: This is a simplified example, you would need to integrate your detections
# and possibly adjust for the camera's intrinsic and extrinsic parameters.
