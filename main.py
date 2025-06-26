import cv2 as cv
from camera_manager import CameraManager 

cm = CameraManager()

while True:
    frame0, frame1 = cm.get_frames()
    combined = cv.hconcat([frame0, frame1])
    
    ''''Run detection logic here
    warning = detect_spaghetti(combined)
    if warning:
        print(" Spaghetti detected!")
    '''

    cv.imshow("3D Printer Monitor", combined)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cm.release()
cv.destroyAllWindows()


