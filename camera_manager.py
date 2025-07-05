import cv2 as cv

class CameraManager:
    def __init__(self, cam1_index=0, cam2_index=1): 
        self.cam1 = cv.VideoCapture(cam1_index)
        self.cam2 = cv.VideoCapture(cam2_index)
        self.setResolution(720, 540)

    def setResolution(self, width, height):  
        for camera in [self.cam1, self.cam2]:
            camera.set(cv.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv.CAP_PROP_FRAME_HEIGHT, height)

    def getFrame(self):
        ret1, frame1 = self.cam1.read()
        ret2, frame2 = self.cam2.read()

        # Return None if a frame failed
        if not ret1:
            frame1 = None
        if not ret2:
            frame2 = None

        return frame1, frame2 

    def release(self):
        if self.cam1 and self.cam1.isOpened():
            self.cam1.release()
        if self.cam2 and self.cam2.isOpened():
            self.cam2.release()
