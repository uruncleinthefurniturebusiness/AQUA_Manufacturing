import cv2
import numpy as np
import time
from scanning import capture_image, preprocess_image

def generate_synthetic_image():
    """
    Generate a synthetic image with a vertical red laser line and Gaussian noise.
    Simulates Pi Camera 3 output for testing on Windows, using BGR for correct red color.
    Returns:
        numpy.ndarray: RGB image (height, width, 3).
    Source: Adapted from FabScan's synthetic testing [1].
    """
    height, width = 480, 640
    image = np.zeros((height, width, 3), dtype=np.uint8)
    for row in range(100, 380):
        for col in range(width):
            intensity = 255 * np.exp(-((col - 320) ** 2) / (2 * 2 ** 2))  # Gaussian profile
            image[row, col] = [0, 0, intensity]  # Red in BGR for cv2.imwrite
    # Add Gaussian noise
    noise = np.random.normal(0, 10, image.shape).astype(np.int16)
    image = np.clip(image + noise, 0, 255).astype(np.uint8)
    cv2.imwrite('data/synthetic_laser.jpg', image)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for processing

def compute_snr(thresh, gray_image):
    """
    Compute signal-to-noise ratio (SNR) for the thresholded image.
    Handles edge cases to avoid division by zero.
    Args:
        thresh (numpy.ndarray): Binary thresholded image.
        gray_image (numpy.ndarray): Grayscale input image.
    Returns:
        float: SNR in dB or -inf if no laser pixels detected.
    Source: Based on signal processing principles [4].
    """
    laser_pixels = gray_image[thresh == 255]
    background_pixels = gray_image[thresh == 0]
    mean_signal = np.mean(laser_pixels) if laser_pixels.size > 0 else 0
    std_noise = np.std(background_pixels) if background_pixels.size > 0 else 1
    return 20 * np.log10(mean_signal / std_noise) if mean_signal > 0 and std_noise > 0 else float('-inf')

def compute_false_positive_rate(thresh, ground_truth):
    """
    Compute false positive rate for non-laser pixels incorrectly thresholded.
    Args:
        thresh (numpy.ndarray): Binary thresholded image.
        ground_truth (numpy.ndarray): Ground truth binary image.
    Returns:
        float: False positive rate.
    Source: Adapted from image processing metrics [2].
    """
    false_positives = np.sum((thresh == 255) & (ground_truth == 0))
    total_non_laser = np.sum(ground_truth == 0)
    return false_positives / total_non_laser if total_non_laser > 0 else 0

# Generate synthetic image
synthetic_image = generate_synthetic_image()

# Test image acquisition and preprocessing
start_time = time.time()
image = capture_image(use_camera=False, image_path='data/synthetic_laser.jpg')
thresh = preprocess_image(image)
end_time = time.time()

# Generate ground truth for false positive rate
ground_truth = np.zeros((480, 640), dtype=np.uint8)
ground_truth[100:380, 318:323] = 255  # Approximate laser line width

# Compute metrics
gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
snr = compute_snr(thresh, gray_image)
fpr = compute_false_positive_rate(thresh, ground_truth)
execution_time = end_time - start_time

# Log results
print(f"SNR: {snr:.2f} dB")
print(f"False Positive Rate: {fpr:.4f}")
print(f"Execution Time: {execution_time:.4f} seconds")

# Save thresholded image
cv2.imwrite('data/thresh_laser.jpg', thresh)