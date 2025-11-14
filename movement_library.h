#ifndef MOVEMENT_LIBRARY_H
#define MOVEMENT_LIBRARY_H
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pca9685.h"
#include <math.h>



// Global PCA9685 instance
extern PCA9685 pwm;

// Servo configuration
#define SERVO_MIN 150
#define SERVO_MAX 600

// Function prototypes - Basic servo control
int angles_to_pwm(int angle, int h, int l, int ha, int la);
void set_servo_pulse(uint8_t channel, float pulse);

// Joint control functions
void joint_1(int angle);
void joint_2(int angle);
void joint_3(int angle);
void joint_4(int angle);
void joints(int angle);

void thigh_1(int angle);
void thigh_2(int angle);
void thigh_3(int angle);
void thigh_4(int angle);
void thighs(int angle);

void calf_1(int angle);
void calf_2(int angle);
void calf_3(int angle);
void calf_4(int angle);
void calfs(int angle);

// Basic positions
void legs_up(void);
void setup_servos(void);
void sit(void);
void stand_up(void);
void xposition(void);
void shift_to(int p);

// Movement functions
void forward(void);
void backward(void);
void ccw(void);
void cw(void);
void right(void);
void left(void);

// Special actions
void hi(void);
void shuffle(void);
void humping(void);
void squads(void);

// Creep gait functions
void leg_position_fb(float y, int l, int s);
void c_f(void);
void c_b(void);

#endif // MOVEMENT_LIBRARY_H
