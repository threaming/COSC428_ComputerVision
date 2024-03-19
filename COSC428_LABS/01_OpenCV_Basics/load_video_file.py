# load_video_file.py

import cv2

# Load the video file.
cap = cv2.VideoCapture('./output.avi')

while cv2.waitKey(30) < 0:
    ret, frame = cap.read()  # Read an frame from the video file.

    # If we cannot read any more frames from the video file, then exit.
    if not ret:
        break
    
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert the frame to grayscale.
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Luv)  # Convert the frame to luv.
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2XYZ)  # Convert the frame to xyz.
    cv2.imshow('frame', frame)  # Display the grayscale frame on the screen.

# Release the video file, and close the GUI.
cap.release()
cv2.destroyAllWindows()
