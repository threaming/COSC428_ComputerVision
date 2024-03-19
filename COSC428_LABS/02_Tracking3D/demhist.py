import numpy as np
import cv2
import matplotlib.pyplot as plt


class DemHist(object):
    def __init__(self):

        self.cap = cv2.VideoCapture(0)

        self.brightness = 100
        self.contrast = 100

        cv2.namedWindow("Window")
        cv2.createTrackbar("brightness", "Window", self.brightness, 200, self.update_brightness)
        cv2.createTrackbar("contrast", "Window", self.contrast, 200, self.update_contrast)

    def update_brightness(self, val):
        self.brightness = val

    def update_contrast(self, val):
        self.contrast = val

    def next_frame(self):
        ret, frame = self.cap.read()
        if ret:

            # Adjust brightness
            frame_norm = frame.astype(np.float32) / 256
            frame_norm = frame_norm * self.brightness / 100
            frame  = frame_norm * 256
            frame = np.clip(frame, 0, 255)
            frame = frame.astype(np.uint8)
            
            # Adjust contrast
            frame_norm = frame.astype(np.float32) / 128 - 1
            frame_norm = frame_norm * self.contrast / 100
            frame  = (frame_norm + 1) * 128
            frame = np.clip(frame, 0, 255)
            frame = frame.astype(np.uint8)

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            hist = cv2.calcHist([frame_gray], [0], None, [256], [0, 256]) / frame_gray.size

            fig, ax = plt.subplots()
            ax.bar(np.arange(256), hist[:,0], color="blue")
            ax.set_ylabel("normalized frequency")
            ax.set_xlabel("pixel intensity bin")

            fig.canvas.draw()
            X = np.array(fig.canvas.renderer.buffer_rgba())[..., :3]
            plt.close()

            img = np.concatenate([frame, X], axis=1)

            cv2.imshow('Window', img)


if __name__ == "__main__":
    demhist = DemHist()
    while cv2.waitKey(1) < 0:
        demhist.next_frame()