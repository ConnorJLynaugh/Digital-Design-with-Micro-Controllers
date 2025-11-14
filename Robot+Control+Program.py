#Using Keyboard to remote control the robot 
#it is able to take pictures, preview video, and record video
#The shell will display what the robot is doing

import movementlibrary as ML
import pygame

from picamera import PiCamera
from time import sleep

import sys

camera = PiCamera()
camera.rotation = 180

pygame.init()
win = pygame.display.set_mode((1,1))

print('Please wait for 10 seconds')
ML.StandUp()
print('I am Ready!')

x = True
p = 1 #number of pictures taken
v = 1 #recorded video number
c = False #preview on or off variable
r = False #record on or off varible

while x == True:
    for eve in pygame.event.get():pass
    keyInput = pygame.key.get_pressed()
    
    if keyInput [pygame.K_w]:
        print('Going Forward')
        ML.Forward()
        
    elif keyInput [pygame.K_s]:
        print('Going Backward')
        ML.Backward()
        
    elif keyInput [pygame.K_e]:
        print('Going Right')
        ML.Right()
        
    elif keyInput [pygame.K_q]:
        print('Going Left')
        ML.Left()
        
    elif keyInput [pygame.K_a]:
        print('Going Counterclockwise')
        ML.CCW()
        
    elif keyInput [pygame.K_d]:
        print('Going Clockwise')
        ML.CW()
        
    elif keyInput [pygame.K_t]:
        print('Creep Forward')
        ML.LegPositionFB(1,4,0)
        sleep(0.1)
        ML.C_F()
        
    elif keyInput [pygame.K_g]:
        print('Creep Backward')
        ML.LegPositionFB(1,2,0)
        sleep(0.1)
        ML.C_B()
        
    elif keyInput [pygame.K_SPACE]:
        print('Saying Hi')
        ML.Hi()
        
    elif keyInput [pygame.K_c]:
        print('Shuffling')
        ML.Shuffle()
        
    elif keyInput [pygame.K_v]:
        print('Humping')
        ML.Humping()
        
    elif keyInput [pygame.K_b]:
        print('Squading from all sides')
        ML.Squads()
        
    elif keyInput [pygame.K_j]: #Press J to take picture
        camera.start_preview(fullscreen=False,window=(200,5,960,540))
        sleep(1)
        camera.capture('/home/pi/Desktop/image'+str(p)+'.jpg')
        print('Picture ' +str(p)+ ' taken')
        p = p + 1
        c = True
        
    elif keyInput [pygame.K_i]: #Press I to preview or camera on
        camera.start_preview(fullscreen=False,window=(200,5,960,540))
        print('Camera start preview')
        c = True
        sleep(5)
        
    elif keyInput [pygame.K_k]: #Stop preview
        camera.stop_preview()
        print('Camera stop preview')
        c = False
        sleep(5)
        
    elif keyInput [pygame.K_o]: # Press o to record the video
        if c == False :
            camera.start_preview(fullscreen=False,window=(200,5,960,540))
            c = True
        
        camera.start_recording('/home/pi/Desktop/video'+str(v)+'.h264', )
        print('Camera start recording video ' +str(v))
        r = True
        sleep(5)
        v=v+1

    elif keyInput [pygame.K_l]: #Stop recording
        camera.stop_recording()
        print('Camera stop recording')
        r = False
        sleep(5)

    elif keyInput [pygame.K_BACKSPACE]: #Press Backspace to end program
        print('Ending Program')
        if r == True:
            camera.stop_recording()
            print('Camera stop recording')
            sleep(5)
        if p == True:
            camera.stop_preview()
            print('Camera stop preview')
            sleep(5)
        ML.Sit()
        sleep(1)
        ML.LegsUp()
        print('Please! I don\'t want to go')
        x = False
        
    else:
        ML.Xposition()
        
    pygame.display.update()

sys.exit('Goodbye')
