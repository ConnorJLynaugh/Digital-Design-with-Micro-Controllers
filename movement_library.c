#include "movement_library.h"
#include <math.h>
#include <stdlib.h>


PCA9685 pwm;

// Convert angles to PWM values
int angles_to_pwm(int angle, int h, int l, int ha, int la) {
    float r = (float)abs(angle - la) / (float)(ha - la);
    int y = (int)(r * (h - l) + l);
    
    if (angle > ha) {
        y = h;
    } else if (angle < la) {
        y = l;
    }
    
    return y;
}

void set_servo_pulse(uint8_t channel, float pulse) {
    float pulse_length = 1000000.0f;  // 1,000,000 us per second
    pulse_length /= 60.0f;             // 60 Hz
    pulse_length /= 4096.0f;           // 12 bits of resolution
    pulse *= 1000.0f;
    pulse /= pulse_length;
    pca9685_set_pwm(&pwm, channel, 0, (uint16_t)pulse);
}

// Joint control functions
void joint_1(int angle) {
    pca9685_set_pwm(&pwm, 10, 0, angles_to_pwm(angle, 500, 260, 135, 45));
}

void joint_2(int angle) {
    pca9685_set_pwm(&pwm, 5, 0, angles_to_pwm(angle, 260, 500, 135, 45));
}

void joint_3(int angle) {
    pca9685_set_pwm(&pwm, 2, 0, angles_to_pwm(angle, 500, 260, 135, 45));
}

void joint_4(int angle) {
    pca9685_set_pwm(&pwm, 13, 0, angles_to_pwm(angle, 260, 500, 135, 45));
}

void joints(int angle) {
    joint_1(angle);
    joint_2(angle);
    joint_3(angle);
    joint_4(angle);
}

// Thigh control functions
void thigh_1(int angle) {
    pca9685_set_pwm(&pwm, 11, 0, angles_to_pwm(angle, 620, 380, 180, 90));
}

void thigh_2(int angle) {
    pca9685_set_pwm(&pwm, 4, 0, angles_to_pwm(angle, 140, 380, 180, 90));
}

void thigh_3(int angle) {
    pca9685_set_pwm(&pwm, 1, 0, angles_to_pwm(angle, 620, 380, 180, 90));
}

void thigh_4(int angle) {
    pca9685_set_pwm(&pwm, 14, 0, angles_to_pwm(angle, 140, 380, 180, 90));
}

void thighs(int angle) {
    thigh_1(angle);
    thigh_2(angle);
    thigh_3(angle);
    thigh_4(angle);
}

// Calf control functions
void calf_1(int angle) {
    pca9685_set_pwm(&pwm, 12, 0, angles_to_pwm(angle, 620, 230, 180, 35));
}

void calf_2(int angle) {
    pca9685_set_pwm(&pwm, 3, 0, angles_to_pwm(angle, 140, 530, 180, 35));
}

void calf_3(int angle) {
    pca9685_set_pwm(&pwm, 0, 0, angles_to_pwm(angle, 620, 230, 180, 35));
}

void calf_4(int angle) {
    pca9685_set_pwm(&pwm, 15, 0, angles_to_pwm(angle, 140, 530, 180, 35));
}

void calfs(int angle) {
    calf_1(angle);
    calf_2(angle);
    calf_3(angle);
    calf_4(angle);
}

// Basic position functions
void legs_up(void) {
    sleep_ms(1000);
    joints(90);
    sleep_ms(1000);
    thighs(180);
    calfs(180);
    sleep_ms(1000);
}

void setup_servos(void) {
    printf("Assemble the servo horns\n");
    sleep_ms(1000);
    joints(90);
    sleep_ms(1000);
    thighs(90);
    sleep_ms(1000);
    calfs(180);
    sleep_ms(1000);
}

void sit(void) {
    thighs(140);
    sleep_ms(500);
    thighs(150);
    sleep_ms(500);
    thighs(160);
    calfs(30);  // legs touching the ground
    sleep_ms(500);
    thighs(170);
    sleep_ms(1000);
}

void stand_up(void) {
    legs_up();
    
    calfs(30);  // legs touching the ground
    sleep_ms(1000);
    thighs(170);
    sleep_ms(2000);
    
    thighs(135);  // lift up the body
    calfs(35);
}

void xposition(void) {
    sleep_ms(200);
    joints(90);
    thighs(135);
    calfs(35);
    sleep_ms(200);
}

void shift_to(int p) {
    xposition();
    
    switch(p) {
        case 1:
            thigh_1(150);
            calf_1(35);
            thigh_2(120);
            joint_2(120);
            joint_4(50);
            sleep_ms(1000);
            break;
            
        case 2:
            thigh_2(150);
            calf_2(35);
            thigh_1(120);
            joint_1(120);
            joint_3(50);
            sleep_ms(1000);
            break;
            
        case 3:
            thigh_3(150);
            calf_3(35);
            thigh_4(120);
            joint_4(120);
            joint_2(50);
            sleep_ms(1000);
            break;
            
        case 4:
            thigh_4(150);
            calf_4(35);
            thigh_3(120);
            joint_3(120);
            joint_1(50);
            sleep_ms(1000);
            break;
            
        default:
            xposition();
            break;
    }
}

// Movement functions
void forward(void) {
    calf_4(45);
    calf_1(45);
    
    thigh_1(160);
    thigh_3(160);
    joint_1(90);
    joint_3(90);
    sleep_ms(100);
    joint_2(120);
    joint_4(60);
    sleep_ms(200);
    thigh_1(135);
    thigh_3(135);
    
    sleep_ms(100);
    
    thigh_2(160);
    thigh_4(160);
    joint_2(90);
    joint_4(90);
    sleep_ms(100);
    joint_1(120);
    joint_3(60);
    sleep_ms(200);
    thigh_2(135);
    thigh_4(135);
    
    sleep_ms(100);
}

void backward(void) {
    calf_4(45);
    calf_1(45);
    
    thigh_1(160);
    thigh_3(160);
    joint_1(90);
    joint_3(90);
    sleep_ms(200);
    joint_2(60);
    joint_4(120);
    sleep_ms(200);
    thigh_1(135);
    thigh_3(135);
    
    sleep_ms(100);
    
    thigh_2(160);
    thigh_4(160);
    joint_2(90);
    joint_4(90);
    sleep_ms(200);
    joint_1(60);
    joint_3(120);
    sleep_ms(200);
    thigh_2(135);
    thigh_4(135);
    
    sleep_ms(100);
}

void ccw(void) {
    calf_4(45);
    calf_1(45);
    
    thigh_1(160);
    thigh_3(160);
    joint_1(90);
    joint_3(90);
    sleep_ms(100);
    joint_2(135);
    joint_4(135);
    sleep_ms(100);
    thigh_1(135);
    thigh_3(135);
    
    sleep_ms(100);
    
    thigh_2(160);
    thigh_4(160);
    joint_2(90);
    joint_4(90);
    sleep_ms(100);
    joint_1(35);
    joint_3(35);
    sleep_ms(100);
    thigh_2(135);
    thigh_4(135);
    
    sleep_ms(100);
}

void cw(void) {
    calf_4(45);
    calf_1(45);
    
    thigh_1(160);
    thigh_3(160);
    joint_1(90);
    joint_3(90);
    sleep_ms(100);
    joint_2(35);
    joint_4(35);
    sleep_ms(100);
    thigh_1(135);
    thigh_3(135);
    
    sleep_ms(100);
    
    thigh_2(160);
    thigh_4(160);
    joint_2(90);
    joint_4(90);
    sleep_ms(100);
    joint_1(135);
    joint_3(135);
    sleep_ms(100);
    thigh_2(135);
    thigh_4(135);
    
    sleep_ms(100);
}

void right(void) {
    calf_4(45);
    calf_1(45);
    
    thigh_1(160);
    thigh_3(160);
    joint_1(90);
    joint_3(90);
    sleep_ms(200);
    joint_2(50);
    joint_4(120);
    sleep_ms(200);
    thigh_1(135);
    thigh_3(135);
    
    sleep_ms(200);
    
    thigh_2(160);
    thigh_4(160);
    joint_2(90);
    joint_4(90);
    sleep_ms(200);
    joint_1(120);
    joint_3(50);
    sleep_ms(200);
    thigh_2(135);
    thigh_4(135);
    
    sleep_ms(200);
}

void left(void) {
    calf_4(45);
    calf_1(45);
    
    thigh_1(160);
    thigh_3(160);
    joint_1(90);
    joint_3(90);
    sleep_ms(200);
    joint_2(120);
    joint_4(50);
    sleep_ms(200);
    thigh_1(135);
    thigh_3(135);
    
    sleep_ms(200);
    
    thigh_2(160);
    thigh_4(160);
    joint_2(90);
    joint_4(90);
    sleep_ms(200);
    joint_1(50);
    joint_3(120);
    sleep_ms(200);
    thigh_2(135);
    thigh_4(135);
    
    sleep_ms(200);
}

// Special actions
void hi(void) {
    printf("Hello!! HUMAN!\n");
    xposition();
    shift_to(3);
    thigh_1(170);
    
    for(int i = 0; i < 5; i++) {
        sleep_ms(200);
        calf_1(90);
        sleep_ms(200);
        calf_1(160);
    }
    
    calf_1(50);
    xposition();
}

void shuffle(void) {
    shift_to(1);
    shift_to(2);
    shift_to(3);
    shift_to(4);
    xposition();
}

void humping(void) {
    sleep_ms(100);
    joints(90);
    thighs(135);
    calfs(35);
    sleep_ms(100);
    thigh_3(160);
    thigh_4(160);
    sleep_ms(100);
    thigh_3(135);
    thigh_4(135);
}

void squads(void) {
    sleep_ms(100);
    joints(90);
    thighs(135);
    calfs(35);
    sleep_ms(100);
    thigh_3(160);
    thigh_4(160);
    sleep_ms(100);
    thigh_3(135);
    thigh_4(135);
    
    sleep_ms(100);
    joints(90);
    thighs(135);
    calfs(35);
    sleep_ms(100);
    thigh_1(160);
    thigh_2(160);
    sleep_ms(100);
    thigh_1(135);
    thigh_2(135);
    
    sleep_ms(100);
    joints(90);
    thighs(135);
    calfs(35);
    sleep_ms(100);
    thigh_1(160);
    thigh_4(160);
    sleep_ms(100);
    thigh_1(135);
    thigh_4(135);
    
    sleep_ms(100);
    joints(90);
    thighs(135);
    calfs(35);
    sleep_ms(100);
    thigh_2(160);
    thigh_3(160);
    sleep_ms(100);
    thigh_2(135);
    thigh_3(135);
}

// Creep gait functions
void leg_position_fb(float y, int l, int s) {
    float c = 93.0f;
    float t = 75.0f;
    float h = 40.0f;
    float xs = 31.8f;
    float z = sqrtf(y * y + xs * xs);
    float w = sqrtf(h * h + z * z);
    float theta_h = atan2f(z, h) * 180.0f / M_PI;
    
    float theta_j = 135.0f - atan2f(y, xs) * 180.0f / M_PI;
    
    float theta_t = acosf((t * t + w * w - c * c) / (2.0f * t * w)) * 180.0f / M_PI + theta_h;
    
    float theta_c = acosf((t * t + c * c - w * w) / (2.0f * t * c)) * 180.0f / M_PI;
    
    if (s == 0) {
        switch(l) {
            case 1:
                thigh_1(175);
                joint_2(50);
                break;
            case 2:
                thigh_2(175);
                joint_1(50);
                break;
            case 3:
                thigh_3(175);
                joint_4(50);
                break;
            case 4:
                thigh_4(175);
                joint_3(50);
                break;
        }
        sleep_ms(200);
    }
    
    switch(l) {
        case 1:
            joint_1((int)theta_j);
            thigh_1((int)theta_t);
            calf_1((int)theta_c);
            thigh_3(135);
            joint_2(90);
            break;
        case 2:
            joint_2((int)theta_j);
            thigh_2((int)theta_t);
            calf_2((int)theta_c);
            thigh_4(135);
            joint_1(90);
            break;
        case 3:
            joint_3((int)theta_j);
            thigh_3((int)theta_t);
            calf_3((int)theta_c);
            thigh_1(135);
            joint_4(90);
            break;
        case 4:
            joint_4((int)theta_j);
            thigh_4((int)theta_t);
            calf_4((int)theta_c);
            thigh_2(135);
            joint_3(90);
            break;
    }
}

void c_f(void) {
    leg_position_fb(80.0f, 1, 0);
    sleep_ms(100);
    leg_position_fb(31.8f, 1, 1);
    leg_position_fb(31.8f, 4, 1);
    leg_position_fb(1.0f, 2, 1);
    leg_position_fb(80.0f, 3, 1);
    sleep_ms(100);
    leg_position_fb(1.0f, 3, 0);
    
    sleep_ms(200);
    
    leg_position_fb(80.0f, 2, 0);
    sleep_ms(100);
    leg_position_fb(31.8f, 2, 1);
    leg_position_fb(1.0f, 1, 1);
    leg_position_fb(31.8f, 3, 1);
    leg_position_fb(80.0f, 4, 1);
    sleep_ms(100);
    leg_position_fb(1.0f, 4, 0);
}

void c_b(void) {
    leg_position_fb(80.0f, 3, 0);
    sleep_ms(100);
    leg_position_fb(31.8f, 2, 1);
    leg_position_fb(31.8f, 3, 1);
    leg_position_fb(1.0f, 4, 1);
    leg_position_fb(80.0f, 1, 1);
    sleep_ms(100);
    leg_position_fb(1.0f, 1, 0);
    
    sleep_ms(200);
    
    leg_position_fb(80.0f, 4, 0);
    sleep_ms(100);
    leg_position_fb(31.8f, 4, 1);
    leg_position_fb(1.0f, 3, 1);
    leg_position_fb(31.8f, 1, 1);
    leg_position_fb(80.0f, 2, 1);
    sleep_ms(100);
    leg_position_fb(1.0f, 2, 0);
}
