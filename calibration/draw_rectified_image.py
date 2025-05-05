import argparse

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml

# create an arugment parser that takes in a path to a YAML and a path to a jpeg
parser = argparse.ArgumentParser()
parser.add_argument("--intrinsics_calibration", help="path to YAML file containing camera intrinsics")
parser.add_argument("--jpeg_path", help="path to a jpeg image taken by the camera")
args = parser.parse_args()

yaml_string = open(args.intrinsics_calibration).read()
intrinsics = yaml.safe_load(yaml_string)

img = cv2.imread(args.jpeg_path)
image_name = args.jpeg_path.split("/")[-1]

if img is None:
    print("Error: could not load image")
    exit(1)

print("image size", img.shape)


camera_matrix = np.array(intrinsics["camera_matrix"]["data"]).reshape(3, 3)
dist_coeffs = np.array(intrinsics["distortion_coefficients"]["data"])


if intrinsics["distortion_model"] == "equidistant" or intrinsics["distortion_model"] == "fisheye":
    # Display image before and after rectification
    fig, ax = plt.subplots(1, 2, figsize=(10, 5))
    plt.suptitle(image_name)

    # adjust the FOV of the post-rectification image
    Knew = camera_matrix.copy()
    Knew[0, 0] = 0.5 * Knew[0, 0]
    Knew[1, 1] = 0.5 * Knew[1, 1]

    # Rectify image using the fisheye model (since fisheye = equidistant)
    rectified_img = cv2.fisheye.undistortImage(img, K=camera_matrix, D=dist_coeffs[:4], Knew=Knew)

    # OpenCV uses BGR color space, so we convert to RGB for proper display with matplotlib
    ax[0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ax[0].set_title("Original Image")

    ax[1].imshow(cv2.cvtColor(rectified_img, cv2.COLOR_BGR2RGB))
    ax[1].set_title("Rectified Image")

    plt.show()
elif intrinsics["distortion_model"] == "rational_polynomial":
    # Display image before and after rectification
    fig, ax = plt.subplots(1, 2, figsize=(10, 5))
    plt.suptitle(image_name)

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, img.shape[1::-1], 1, img.shape[1::-1]
    )
    rectified_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

    # OpenCV uses BGR color space, so we convert to RGB for proper display with matplotlib
    ax[0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ax[0].set_title("Original Image")

    ax[1].imshow(cv2.cvtColor(rectified_img, cv2.COLOR_BGR2RGB))
    ax[1].set_title("Rectified Image")

    plt.show()
elif intrinsics["distortion_model"] == "plumb_bob":
    # Display image before and after rectification
    plt.suptitle(f"{image_name}\nOriginal Image (This linear camera requires no rectification)")

    # OpenCV uses BGR color space, so we convert to RGB for proper display with matplotlib
    if not len(dist_coeffs) == 0:
        print("This script assumes plumb_bob cameras are linear")
        exit(1)
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.show()
