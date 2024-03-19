# optical_flow.py

import numpy as np
import cv2

class LucasKanade(object):
    def __init__(self):
        self.cap = cv2.VideoCapture(0)  # Open the first camera connected to the computer.

        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100,
                            qualityLevel = 0.3,
                            minDistance = 7,
                            blockSize = 7 )

        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15,15),
                        maxLevel = 2,
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Create some random colors
        self.color = np.random.randint(0,255,(100,3))

        self.p0 = None
        while self.p0 is None:
            self.set_reference_points()

        self.cam_toggle_bool = True
        self.dot_toggle_bool = True

        cv2.namedWindow("Frame")
        cv2.createButton("Camera",self.cam_toggle,None,cv2.QT_PUSH_BUTTON,1)
        cv2.createButton("Dots",self.dot_toggle,None,cv2.QT_PUSH_BUTTON,1)
        cv2.createButton("Exit",self.exit_button,None,cv2.QT_PUSH_BUTTON,1)

    def set_reference_points(self):
        # Take first frame and find corners in it
        ret, self.old_frame = self.cap.read()
        self.old_gray = cv2.cvtColor(self.old_frame, cv2.COLOR_BGR2GRAY)
        self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)

    def cam_toggle(self, *args):
        self.cam_toggle_bool = not self.cam_toggle_bool

    def dot_toggle(self, *args):
        self.dot_toggle_bool = not self.dot_toggle_bool

    def exit_button(self, *args):
        exit()

    def next_frame(self):

        ret, frame = self.cap.read()
        if ret:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if not self.cam_toggle_bool:
                frame[...] = 0

            if len(self.p0) == 0:
                self.set_reference_points()
                return
                
            # calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
            # Select good points
            if p1 is not None:
                good_new = p1[st==1]
                good_old = self.p0[st==1]
            else:
                return

            if self.dot_toggle_bool:
                # draw the dots
                for i, (new, old) in enumerate(zip(good_new, good_old)):
                    a, b = new.ravel()
                    frame = cv2.circle(frame, (int(a), int(b)), 5, self.color[i].tolist(), -1)

            cv2.imshow('Frame', frame)

            # Now update the previous frame and previous points
            self.old_gray = frame_gray.copy()
            self.p0 = good_new.reshape(-1, 1, 2)



if __name__ == "__main__":
    lucaskanade = LucasKanade()
    while cv2.waitKey(1) < 0:
        lucaskanade.next_frame()