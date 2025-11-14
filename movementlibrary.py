#This program has functions that let the robot go forward, backward, right, left, CW, and CCW. 

from __future__ import division
import time
import Adafruit_PCA9685
import math
pwm = Adafruit_PCA9685.PCA9685()

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
    
pwm.set_pwm_freq(60)
    
def AnglesToPWM(x, H, L, Ha, La): #x = angle, H = High PWM value, L = Low PWM value
    r = abs((x - La)/(Ha - La))
    y = int(r * (H - L) + L)
    if x > Ha:
        y = H
    elif x < La:
        y = L
    return y


def Joint_1(A):
    pwm.set_pwm(10, 0, AnglesToPWM(A, 500, 260, 135, 45))  #1
def Joint_2(A):
    pwm.set_pwm(5, 0, AnglesToPWM(A, 260, 500, 135, 45)) #2
def Joint_3(A):
    pwm.set_pwm(2, 0, AnglesToPWM(A, 500, 260, 135, 45)) #3
def Joint_4(A):
    pwm.set_pwm(13, 0, AnglesToPWM(A, 260, 500, 135, 45))  #4
    
def Thigh_1(A):
    pwm.set_pwm(11, 0, AnglesToPWM(A, 620, 380, 180, 90))  #1
def Thigh_2(A):
    pwm.set_pwm(4, 0, AnglesToPWM(A, 140, 380, 180, 90)) #2
def Thigh_3(A):
    pwm.set_pwm(1, 0, AnglesToPWM(A, 620, 380, 180, 90)) #3
def Thigh_4(A):
    pwm.set_pwm(14, 0, AnglesToPWM(A, 140, 380, 180, 90))  #4

def Calf_1(A):
    pwm.set_pwm(12, 0, AnglesToPWM(A, 620, 230, 180, 35))  #1
def Calf_2(A):
    pwm.set_pwm(3, 0, AnglesToPWM(A, 140, 530, 180, 35)) #2
def Calf_3(A):
    pwm.set_pwm(0, 0, AnglesToPWM(A, 620, 230, 180, 35)) #3
def Calf_4(A):
    pwm.set_pwm(15, 0, AnglesToPWM(A, 140, 530, 180, 35))  #4

def Joints(A):
    Joint_1(A)
    Joint_2(A)
    Joint_3(A)
    Joint_4(A)

def Thighs(A):
    Thigh_1(A)
    Thigh_2(A)
    Thigh_3(A)
    Thigh_4(A)

def Calfs(A):
    Calf_1(A)
    Calf_2(A)
    Calf_3(A)
    Calf_4(A)

def LegsUp():
    time.sleep(1)
    Joints(90)
    time.sleep(1)
    Thighs(180)
    Calfs(180)
    time.sleep(1)

def SetUp():
    print("Assemble the servo horns") 
    time.sleep(1)
    Joints(90)
    time.sleep(1)
    Thighs(90)
    time.sleep(1)
    Calfs(180)
    time.sleep(1)
    
def Sit():
    Calfs(30) #legs touching the ground
    time.sleep(1)
    Thighs(170)
    
#Stand up Program
def StandUp() :
    LegsUp()

    Calfs(30) #legs touching the ground
    time.sleep(1)
    Thighs(170)
    time.sleep(2)

    Thighs(135) #lift up the body
    Calfs(35)

def Xposition():
    time.sleep(0.2)
    Joints(90)
    Thighs(135)
    Calfs(35)
    time.sleep(0.2)

def ShiftTo(P):
    
    Xposition()
    
    if P == 1:
        Thigh_1(150)
        Calf_1(35)
        Thigh_2(120)
        Joint_2(120)
        Joint_4(50)
        time.sleep(1)
        
    elif P == 2:
        Thigh_2(150)
        Calf_2(35)
        Thigh_1(120)
        Joint_1(120)
        Joint_3(50)
        time.sleep(1)
        
    elif P == 3:
        Thigh_3(150)
        Calf_3(35)
        Thigh_4(120)
        Joint_4(120)
        Joint_2(50)
        time.sleep(1)
        
    elif P == 4:
        Thigh_4(150)
        Calf_4(35)
        Thigh_3(120)
        Joint_3(120)
        Joint_1(50)
        time.sleep(1)
    
    else:
        Xposition()
        

#Walking Method
'''It walks by lifting two legs and twist two separate joints
to move forward. I called it Lift Two Twist Two :P OR kind of like a dog trotting'''
def Forward():
    
    Calf_4(45)
    Calf_1(45)
     
    Thigh_1(160) #lifts leg 1 and 3 up at the sametime
    Thigh_3(160)
    Joint_1(90) #Move joint 1 and 3 to the orignal position for the next move
    Joint_3(90)
    time.sleep(0.1)
    Joint_2(120) #Move or "twist" joint 2 and 4 
    Joint_4(60)
    time.sleep(0.2)
    Thigh_1(135) #drop leg 1 and 3  
    Thigh_3(135)

    time.sleep(0.1)
    #Repeat the same steps but for the opposite legs
    Thigh_2(160) #lifts leg 2 and 4 up at the sametime
    Thigh_4(160)
    Joint_2(90) #Move joint 2 and 4 to the orignal position for the next move
    Joint_4(90)
    time.sleep(0.1)
    Joint_1(120) #Move or "twist" joint 1 and 3
    Joint_3(60)
    time.sleep(0.2)
    Thigh_2(135) #drop leg 2 and 4
    Thigh_4(135)

    time.sleep(0.1)
    

#Walk Backward
def Backward():
    
    Calf_4(45)
    Calf_1(45)
    
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    time.sleep(0.2)
    Joint_2(60)#
    Joint_4(120)#
    time.sleep(0.2)
    Thigh_1(135)
    Thigh_3(135)
    
    time.sleep(0.1)

    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    time.sleep(0.2)
    Joint_1(60)#
    Joint_3(120)#
    time.sleep(0.2)
    Thigh_2(135)
    Thigh_4(135)

    time.sleep(0.1)


#Rotate Counterclockwise
def CCW():

    Calf_4(45)
    Calf_1(45)
    
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    time.sleep(0.1)
    Joint_2(135)#
    Joint_4(135)#
    time.sleep(0.1)
    Thigh_1(135)
    Thigh_3(135)
    
    time.sleep(0.1)

    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    time.sleep(0.1)
    Joint_1(35)#
    Joint_3(35)#
    time.sleep(0.1)
    Thigh_2(135)
    Thigh_4(135)

    time.sleep(0.1)

#Rotate Clockwise
def CW():
    
    Calf_4(45)
    Calf_1(45)
    
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    time.sleep(0.1)
    Joint_2(35)#
    Joint_4(35)#
    time.sleep(0.1)
    Thigh_1(135)
    Thigh_3(135)
    
    time.sleep(0.1)

    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    time.sleep(0.1)
    Joint_1(135)#
    Joint_3(135)#
    time.sleep(0.1)
    Thigh_2(135)
    Thigh_4(135)

    time.sleep(0.1)

#Crab walk to the right
def Right():
    
    Calf_4(45)
    Calf_1(45)
    
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    time.sleep(0.2)
    Joint_2(50)#
    Joint_4(120)#
    time.sleep(0.2)
    Thigh_1(135)
    Thigh_3(135)
    
    time.sleep(0.2)
    
    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    time.sleep(0.2)
    Joint_1(120)#
    Joint_3(50)#
    time.sleep(0.2)
    Thigh_2(135)
    Thigh_4(135)

    time.sleep(0.2)

#Crab walk to the left
def Left():
    
    Calf_4(45)
    Calf_1(45)
    
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    time.sleep(0.2)
    Joint_2(120)#
    Joint_4(50)#
    time.sleep(0.2)
    Thigh_1(135)
    Thigh_3(135)
    
    time.sleep(0.2)
    
    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    time.sleep(0.2)
    Joint_1(50)#
    Joint_3(120)#
    time.sleep(0.2)
    Thigh_2(135)
    Thigh_4(135)

    time.sleep(0.2)

def Hi():
    print('Hello!! HUMAN!')
    Xposition()
    ShiftTo(3)
    Thigh_1(170)
    i=0
    while i<5:
        i=i+1
        time.sleep(0.2)
        Calf_1(90)
        time.sleep(0.2)
        Calf_1(160)
    Calf_1(50)
    Xposition()
  
def Shuffle():
    ShiftTo(1)
    ShiftTo(2)
    ShiftTo(3)
    ShiftTo(4)
    Xposition()
    
def Sit():
    Thighs(140)
    time.sleep(0.5)
    Thighs(150)
    time.sleep(0.5)
    Thighs(160)
    Calfs(30) #legs touching the ground
    time.sleep(0.5)
    Thighs(170)
    time.sleep(1)
    
def Humping():
    time.sleep(0.1)
    Joints(90)
    Thighs(135)
    Calfs(35)
    time.sleep(0.1)
    Thigh_3(160)
    Thigh_4(160)
    time.sleep(0.1)
    Thigh_3(135)
    Thigh_4(135)

def Squads():
    time.sleep(0.1)
    Joints(90)
    Thighs(135)
    Calfs(35)
    time.sleep(0.1)
    Thigh_3(160)
    Thigh_4(160)
    time.sleep(0.1)
    Thigh_3(135)
    Thigh_4(135)
    
    time.sleep(0.1)
    Joints(90)
    Thighs(135)
    Calfs(35)
    time.sleep(0.1)
    Thigh_1(160)
    Thigh_2(160)
    time.sleep(0.1)
    Thigh_1(135)
    Thigh_2(135)

    time.sleep(0.1)
    Joints(90)
    Thighs(135)
    Calfs(35)
    time.sleep(0.1)
    Thigh_1(160)
    Thigh_4(160)
    time.sleep(0.1)
    Thigh_1(135)
    Thigh_4(135)    
    
    time.sleep(0.1)
    Joints(90)
    Thighs(135)
    Calfs(35)
    time.sleep(0.1)
    Thigh_2(160)
    Thigh_3(160)
    time.sleep(0.1)
    Thigh_2(135)
    Thigh_3(135)

'''This is a different walking method. Below these functions are for creep gait'''
def LegPositionFB(Y,L,S): #This function is for creep gait going forward and backward 
    c = 93
    T = 75
    h = 40
    Xs = 31.8
    Z = math.sqrt(Y**2 + Xs**2)
    #print(Z)
    w = math.sqrt(h**2 + Z**2)
    thetaH = math.degrees(math.atan(Z/h))
    
    thetaJ = 135 - math.degrees(math.atan(Y/Xs))
    #print(thetaJ)
    
    thetaT = math.degrees(math.acos((T**2 + w**2 - c**2)/(2*T*w))) + thetaH
    #print(thetaT)
    
    thetaC = math.degrees(math.acos((T**2 + c**2 - w**2)/(2*T*c)))
    #print(thetaC)
    
    if S == 0:
        if L == 1:
            #Thigh_3(160)
            Thigh_1(175)
            Joint_2(50)
        elif L == 2:
            #Thigh_4(160)
            Thigh_2(175)
            Joint_1(50)
        elif L == 3:
            #Thigh_1(160)
            Thigh_3(175)
            Joint_4(50)
        elif L == 4:
            #Thigh_2(160)
            Thigh_4(175)
            Joint_3(50)
            
        time.sleep(0.2)
        
    if L == 1:
        Joint_1(thetaJ)
        Thigh_1(thetaT)
        Calf_1(thetaC)
        Thigh_3(135)
        Joint_2(90)
    elif L == 2:
        Joint_2(thetaJ)
        Thigh_2(thetaT)
        Calf_2(thetaC)
        Thigh_4(135)
        Joint_1(90)
    elif L == 3:
        Joint_3(thetaJ)
        Thigh_3(thetaT)
        Calf_3(thetaC)
        Thigh_1(135)
        Joint_4(90)
    elif L == 4:
        Joint_4(thetaJ)
        Thigh_4(thetaT)
        Calf_4(thetaC)
        Thigh_2(135)
        Joint_3(90)
    else:
        print("L can only be in between 1 to 4")


def C_F(): #Walk forward using creep gait, walk by lifting one leg at a time
    LegPositionFB(80,1,0)
    time.sleep(0.1)
    LegPositionFB(31.8,1,1)
    LegPositionFB(31.8,4,1)
    LegPositionFB(1,2,1)
    LegPositionFB(80,3,1)
    time.sleep(0.1)
    LegPositionFB(1,3,0)
    
    time.sleep(0.2)
    
    LegPositionFB(80,2,0)
    time.sleep(0.1)
    LegPositionFB(31.8,2,1)
    LegPositionFB(1,1,1)
    LegPositionFB(31.8,3,1)
    LegPositionFB(80,4,1)
    time.sleep(0.1)
    LegPositionFB(1,4,0)

def C_B():
    LegPositionFB(80,3,0)
    time.sleep(0.1)
    LegPositionFB(31.8,2,1)
    LegPositionFB(31.8,3,1)
    LegPositionFB(1,4,1)
    LegPositionFB(80,1,1)
    time.sleep(0.1)
    LegPositionFB(1,1,0)
    
    time.sleep(0.2)
    
    LegPositionFB(80,4,0)
    time.sleep(0.1)
    LegPositionFB(31.8,4,1)
    LegPositionFB(1,3,1)
    LegPositionFB(31.8,1,1)
    LegPositionFB(80,2,1)
    time.sleep(0.1)
    LegPositionFB(1,2,0)

#Remember!!! to comment the line below before you run the robot control code
#SetUp()