"""
Author: Josh Tait

Code should run and display a live webcam feed where the user's hand position
within the frame will determine the pitch and amplitude that is delivered through
the interface's speakers

"""

"""
Question: Can this code be run such that the values can can be read from another program in parallel?
The Other program would read the values from the CV program, and export them to a continous synthesizer.
This would allow extra features to be easily implemented.

"""


import cv2
import mediapipe as mp
import time
import thinkdsp
from pydub import AudioSegment
from playsound import playsound
from pydub.playback import play
import numpy as np
import threading


filename = "sound.wav"

cap = cv2.VideoCapture(0)  # Open the first camera connected to the computer.

mpHands = mp.solutions.hands
hands = mpHands.Hands() #Default parameters selected
mpDraw = mp.solutions.drawing_utils

"""-----------------------------------------------------------------------------
-------------------------Initialising Constants------------------------------"""

minFreq = 130.8
maxFreq = 523.35
freqRange = maxFreq - minFreq

minAmp = 0
maxAmp = 0.5
ampRange = maxAmp - minAmp


pTime = 0
cTime = 0
fps = 0
duration = 1
normalPitch = 440
normalAmplitude = 0.5
numLms = 20
soundDelay = 10
xMovAverage = 0
yMovAverage = 0
xOut = 0
yOut = 0
BUFFSIZE = 5
targetID = 20



def writeWave(filename, normalPitch=0, normalAmplitude=0, duration=0): 
    """Function that converts a integers into a note containing pitch, 
                      amplitude and duration"""    
    #default amplitude is 0.5. So maybe normalise between 0.0 and 0.5
    
    sin_sig = thinkdsp.SinSignal(freq=normalPitch, amp=normalAmplitude, offset=0)
    wave = sin_sig.make_wave(duration, start=0, framerate=44100)
    wave.write(filename)
    audio = AudioSegment.from_wav(filename)
    
    return audio 



"""MAIN WHILE LOOP BEGINS HERE"""

while True:
    ret, frame = cap.read()  # Read an image from the frame.
    imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)
    isHand = results.multi_hand_landmarks
    handedness = results.multi_handedness
    
    if isHand:
                

        normalPitch = minFreq + yOut* freqRange 
        normalAmplitude = minAmp + xOut * ampRange
        
        print(normalPitch, normalAmplitude)
        
        audio = writeWave(filename, normalPitch, normalAmplitude, duration=0.1)
        
        #playsound('sound.wav', block=False)     
        
        play(audio)
        
        
        xOut = 0
        yOut = 0
        
        handLms = isHand[0] #isHand is a list with one item? 
        
        desLm = handLms.landmark[targetID] #.landmark opens an iterable list of all landmarks
        h, w, c = frame.shape
        cy, cx = int(desLm.x*w), int(desLm.y*h)
        yOut = desLm.x
        xOut = desLm.y     
        cv2.circle(frame, (cy, cx), 25, (255, 0, 255), cv2.FILLED)
            
        
        mpDraw.draw_landmarks(frame, handLms, mpHands.HAND_CONNECTIONS)
            

    cTime = time.time() #extension is just a clock
    fps = 1/(cTime-pTime) #
    pTime = cTime
    duration = 1/fps

    
    cv2.putText(frame, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 
                3, (255, 0, 255), 3)
    
    cv2.imshow('frame', frame)  # Show the image on the display.
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Close the script when q is pressed.
        break

# Release the camera device and close the GUI.
cap.release()
cv2.destroyAllWindows()




