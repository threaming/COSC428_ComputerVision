# webcam_to_file.py

import cv2

# Open the first camera connected to the computer.
cap = cv2.VideoCapture(-1)

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

while cv2.waitKey(1) < 0:
    ret, frame = cap.read()  # Read an frame from the webcam.
    frame = cv2.resize(frame, (640, 480), interpolation = cv2.INTER_AREA)
    out.write(frame)  # Write the frame to the output file.
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change display colour (uncomment for BW)
    cv2.imshow('frame', frame)  # While we're here, we might as well show it on the screen.

# Release the camera device and output file, and close the GUI.
cap.release()
out.release()
cv2.destroyAllWindows()
