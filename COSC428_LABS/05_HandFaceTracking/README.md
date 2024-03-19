# COSC428 Lab 5 - Example projects

This lab is a collection of interesting computer vision projects. Completion of each section is optional, but feel free to use any of them as inspiration for your own project.

Note: For this lab, you will be creating your own virtual environments for each project. As these can be quite large, we highly recommend using the `/local` directory on your lab PC.

To do this use cd

`cd /local`

## Lab content

- [Real-time face replacement](#real-time-face-replacement)
- [Musical hand tracking](#musical-hand-tracking)

## Real-Time Face Replacement

This is a 2022 COSC428 project by Scott (Gonzai) Li. The project detects a set of keypoints for each of two faces within a video frame and then uses them to superimpose each person's face onto the other. The project uses the connected webcam and generates the altered footage in real-time.

Abstract: This paper proposes a method for real-time face replacement using a webcam. Most of the current researches on face replacement techniques are based on static face replacement techniques. Those studies focused on how to achieve face replacement between people in two photographs. This paper is devoted to achieving real-time face replacement based on the static face replacement technique. Ultimately, real-time face replacement technique has been implemented by merging the algorithms of face recognition, convex hull, Delaunay triangulation, and Poisson equation. In addition, there is also face colour contrast after real-time face replacement was performed. The colour difference between the central part of the new face and the original face was significant in comparison to the colour difference between the boundaries of both the new face and original face. The results also show that there are some limitations of the real-time face replacement function.The limitations include differences in face size, facial occlusions, and the rotation angle of the face in whichthey have a great impact on the outcome.

### To do:
- `cd FaceReplace`
- `mkdir env`
- `virtualenv --python /usr/bin/python3.9 ./env/face-replace`
- `source ./env/face-replace/bin/activate`
- `pip install -r requirements.txt`
- `python main.py`

## Musical Hand Tracking

This is a 2022 COSC428 project by Josh Tait. The project detects the pose of a hand as a 2D coordinate within the image and then uses it to control pitch and volume for each dimension, respectively.

Abstract—A hand tracking package, ‘MediaPipe Hands’ was used to create a novel live musical experience for a user. Previous attempts at producing a similar application included the use of algorithms such as morphological closing, contour detection, and Gaussian background subtraction. The proposed method in this report utilised a method based on a single shot detector model which utilised convolution neural networks and landmark prediction to produce a precise location of a hand within a webcam frame. The pydub module was then utilised with the ‘play()’ method to produce sounds of different pitch and amplitude which depended on the position of the hand in 2D space. The program was run on a 2017 Macbook Pro on a 3.1 GHz Dual-Core Intel Core i5 processor using python. The program resulted in sounds being produced at approximately 4 beeps per second. The display refreshed at less than 1 fps due to inefficiencies in code. Compared to similar past attempts, the SSD hand detection method was determined to be highly reliable without affecting the frame rate too greatly at 30fps. Future improvements would include multithread processing to allow the sound to run in parallel with the main code, or otherwise, the use of a better sound module to provide continuous feedback to the user

### To do:
- `cd MusicalHandTracking`
- `mkdir env`
- `virtualenv --python /usr/bin/python3.9 ./env/hand-track`
- `source ./env/hand-track/bin/activate`
- `pip install -r requirements.txt`
- `python main.py`


