"""
scanning.py: Image acquisition and preprocessing for AQUA Manufacturing 3D scanning.
Implements Subtask 6.1 of Project 1, Task 6, capturing images from Raspberry Pi Camera
Module 3 and isolating the laser line using OpenCV. Adapted from FabScan [1].
"""
import cv2
import numpy as np
try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None  # Fallback for Windows development

def capture_image(use_camera=True, image_path=None):
    """
    Capture an image from the Pi Camera 3 or load a synthetic image.
    Designed for AQUA's scanning pipeline, supporting integration with Z-axis (camera),
    X-axis (nozzle), and Y-axis/rotation (bed).
    Args:
        use_camera (bool): If True, capture from camera; else, load from file.
        image_path (str): Path to image file if use_camera is False.
    Returns:
        numpy.ndarray: RGB image (480x640).
    Raises:
        ValueError: If image_path is None when use_camera is False.
        FileNotFoundError: If image_path is invalid.
    Source: Adapted from FabScan [1] and picamera2 documentation [4].
    """
    if use_camera:
        if Picamera2 is None:
            raise ImportError("picamera2 not available; use image_path for testing")
        picam2 = Picamera2()
        picam2.configure(picam2.create_still_configuration(main={"size": (640, 480)}))
        picam2.start()
        image = picam2.capture_array("main")  # RGB image as NumPy array
        picam2.stop()
        return image
    else:
        if image_path is None:
            raise ValueError("image_path must be provided if use_camera is False")
        image = cv2.imread(image_path)
        if image is None:
            raise FileNotFoundError(f"Image not found at {image_path}")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB

def preprocess_image(image):
    """
    Preprocess an image to isolate the laser line for 3D scanning.
    Steps: Grayscale conversion, 5x5 Gaussian blur, fixed thresholding (200).
    Optimized for Raspberry Pi 5 efficiency and AQUA’s 50-micron resolution goal.
    Args:
        image (numpy.ndarray): Input RGB image (480x640).
    Returns:
        numpy.ndarray: Binary image with laser line at 255, background at 0.
    Source: OpenCV documentation [6], laser line detection techniques [7].
    """
    # Convert to grayscale to focus on intensity
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # Apply 5x5 Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # Apply fixed thresholding to isolate laser line
    _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
    return thresh