import cv2
import numpy as np
from scanning import capture_image, preprocess_image

def generate_synthetic_image():
    """
    Generate a synthetic image with a vertical red laser line and Gaussian noise.
    Simulates Pi Camera 3 output for testing on Windows.
    Returns:
        numpy.ndarray: RGB image with laser line.
    Source: Adapted from FabScan's synthetic testing approach [1].
    """
    height, width = 480, 640
    image = np.zeros((height, width, 3), dtype=np.uint8)
    for row in range(100, 380):
        for col in range(width):
            intensity = 255 * np.exp(-((col - 320) ** 2) / (2 * 2 ** 2))  # Gaussian profile
            image[row, col] = [intensity, 0, 0]  # Red laser line
    # Add Gaussian noise to simulate real-world conditions
    noise = np.random.normal(0, 10, image.shape).astype(np.int16)
    image = np.clip(image + noise, 0, 255).astype(np.uint8)
    cv2.imwrite('data/synthetic_laser.jpg', image)
    return image

# Generate and save synthetic image
synthetic_image = generate_synthetic_image()

# Test image acquisition and preprocessing
image = capture_image(use_camera=False, image_path='data/synthetic_laser.jpg')
thresh = preprocess_image(image)

# Save thresholded image for visual inspection
cv2.imwrite('data/thresh_laser.jpg', thresh)