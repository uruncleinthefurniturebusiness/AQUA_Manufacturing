import cv2 as cv
from camera_manager import CameraManager

# Initialize camera manager
try:
    cm = CameraManager()
except Exception as e:
    print(f"❌ CameraManager initialization failed: {e}")
    exit(1)

while True:
    frame0, frame1 = cm.getFrame()

    # Decide what to display
    if frame0 is not None and frame1 is not None:
        combined = cv.hconcat([frame0, frame1])
    elif frame0 is not None:
        combined = frame0
    elif frame1 is not None:
        combined = frame1
    else:
        print("No frames available from either camera.")
        break

    # Optional: Detection logic here
    '''
    warning = detect_spaghetti(combined)
    if warning:
        print("⚠️ Spaghetti detected!")
    '''

    cv.imshow("3D Printer Monitor", combined)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cm.release()
cv.destroyAllWindows()
