import cv2 as cv

class CameraManager:
    def _init_(self, cam1_index = 0, cam2_index = 1):
        self.cam1 = cv.VideoCapture(cam1_index)
        self.cam2 = cv.VideoCapture(cam2_index)
        self.setResolution(1280,720)

    def setResolution(self, height, width):
        for camera in [self.cam1, self.cam2]:
            camera.set(cv.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    
    def getFrame(self):
        frame1 = self.cam1.read()
        frame2 = self.cam2.read()

        return frame1,frame2
    
    def release(self):
        self.cam1.release()
        self.cam2.release()


    

    