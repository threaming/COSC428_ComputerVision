import numpy as np
import cv2
import matplotlib.pyplot as plt


class MeanShiftDemo(object):
    def __init__(self):
        self.drawing = False

        self.cap = cv2.VideoCapture(0)

        self.roi_hist = np.zeros((180, 1), dtype=np.float32)

        self.track_window = [0, 0, 1, 1]

        cv2.namedWindow("Window")
        cv2.setMouseCallback("Window", self.draw)

        # Setup the termination criteria, either 10 iteration or move by at least 1 pt
        self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

    def draw(self, event, ix, iy, flags, params):
        if(event==1):
            self.drawing = True
            self.track_window[0] = ix
            self.track_window[1] = iy

        if(event==4 and self.drawing):
            self.drawing = False

            self.track_window[2] = ix - self.track_window[0]
            self.track_window[3] = iy - self.track_window[1]

            x, y, w, h = self.track_window

            # set up the ROI for tracking
            roi = self.frame[y:y+h, x:x+w]
            hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
            self.roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])

            cv2.normalize(self.roi_hist,self.roi_hist,0,255,cv2.NORM_MINMAX)
        

    def next_frame(self):
        ret, self.frame = self.cap.read()
        if ret == True:
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1)

            # apply meanshift to get the new location
            ret, track_window = cv2.meanShift(dst, self.track_window, self.term_crit)
            # Draw it on image
            x,y,w,h = track_window

            roi2 = self.frame[y:y+h, x:x+w]
            hsv_roi2 =  cv2.cvtColor(roi2, cv2.COLOR_BGR2HSV)
            mask2 = cv2.inRange(hsv_roi2, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
            roi_hist2 = cv2.calcHist([hsv_roi2],[0],mask2,[180],[0,180])
            cv2.normalize(roi_hist2,roi_hist2,0,255,cv2.NORM_MINMAX)

            # Note: I think the colours are inverted due to maplotlib using BGR not RGB
            fig, ax = plt.subplots(ncols=1, nrows=2)
            ax[0].bar(np.arange(180), self.roi_hist[:,0], color="blue")
            ax[0].set_ylabel("normalized frequency")
            ax[0].set_xlabel("hue value bin")
            ax[0].legend(["target distribution"])

            ax[1].bar(np.arange(180), roi_hist2[:,0], color="red")
            ax[1].set_ylabel("normalized frequency")
            ax[1].set_xlabel("hue value bin")
            ax[1].legend(["current distribution"])

            plt.tight_layout()

            fig.canvas.draw()
            X = np.array(fig.canvas.renderer.buffer_rgba())[..., :3]
            plt.close()

            if not self.drawing:
                img2 = cv2.rectangle(self.frame, (x,y), (x+w,y+h), 255,2)
            else:
                img2 = self.frame

            img3 = np.concatenate([img2, X], axis=1)
            cv2.imshow('Window',img3)


if __name__ == "__main__":
    mean_shift_demo = MeanShiftDemo()
    while cv2.waitKey(1) < 0:
        mean_shift_demo.next_frame()