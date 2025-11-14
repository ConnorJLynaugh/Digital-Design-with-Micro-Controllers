/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
// Include custom libraries
#include "vga16_graphics_v2.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// character array
char screentext[40];
char plot_text[40];
char buf[1000];

// draw speed
int threshold = 38 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;
// ISR activity counter for diagnostics
volatile int isr_count = 0;

// ========= PWM Set Up ======================================
#define WRAPVAL 5000
static volatile float CLKDIV = 25.0f; 
// GPIO we're using for PWM
#define PWM_OUT 4
// Variable to hold PWM slice number
uint slice_num ;
// PWM duty cycle
volatile int control ;
volatile int old_control ;


// Default system variables on start up (editable in serial terminal)
volatile fix15 desired_angle = int2fix15(90);
volatile fix15 error = int2fix15(0);
volatile fix15 kp = int2fix15(100); // Max at start up is 3000 PWM / 90 degrees
volatile fix15 ki = int2fix15(5);
volatile fix15 integral_sum = int2fix15(0); // Accumulative sum (error * dt)
static fix15 dt = float2fix15(0.001);
volatile fix15 kd = int2fix15(250);
volatile fix15 delta_error = int2fix15(0); 

// // Sliding Integeral Sum
// # define integral_size 500
// volatile fix15 buffer[integral_size] = {int2fix15(0)};
// volatile int head = 0;
// volatile fix15 integral_sum = int2fix15(0);

// Accelerometer Filter
// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];
int acc_filt[3] = {0, 0, 0};
static fix15 complementary_angle = int2fix15(0);

/////////////// low pass filter on accelerometer and complimentary filter /////////////
static void FilteredAccel(void) {
    // Uses a first-order IIR filter as the source website said
    // Equation: y[n] = y[n-1] + α * (x[n] - y[n-1])
    // Here α = 1/64 because of the ">>6" shift.
    // -------------------------------------------------------------
    static int32_t acc_filt[3] = {0, 0, 0};   // filtered (smoothed) accel values

    static fix15 accel_angle = 0;
    static fix15 gyro_angle_delta = 0;

    for (int i = 0; i < 3; i++) {
        // IIR low-pass filter keeps slow (low-frequency) motion, removes fast (high-frequency) vibration/noise.
        acc_filt[i] = acc_filt[i] + ((acceleration[i] - acc_filt[i]) >> 6); // shift 6 from course website
    }

    // Complementary Filter for Angle Estimation
    fix15 filtered_ay = acc_filt[1];
    fix15 filtered_az = acc_filt[2];

    // this is from the complemntary filter example on the course website
    // SMALL ANGLE APPROXIMATION
    // accel_angle = multfix15(divfix(acceleration[0], acceleration[1]), oneeightyoverpi);

    // NO SMALL ANGLE APPROXIMATION
    float fay = fix2float15(filtered_ay);
    float faz = fix2float15(filtered_az);
    fix15 PI = 3.1415926535897932384; 
    accel_angle = multfix15(float2fix15(atan2f(-faz, -fay) + PI), oneeightyoverpi);

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    gyro_angle_delta = multfix15(gyro[2], zeropt001);

    // Complementary angle (degrees - 15.16 fixed point)
    //change these weights as necessary
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, float2fix15(0.5))
                        + multfix15(accel_angle, float2fix15(0.5));

    // //// print the accelerometer and complementary angles to VGA --same format as lab 2 
    // char buf[1000];
    // setTextSize(1);
    // setTextColor2(WHITE, BLACK);
    // setCursor(0, 22);

    // // snprintf(buf, sizeof(buf),
    //          "    Accel(filt): X:%ld Y:%ld Z:%ld  CompAng:%d deg  desired angle:%d  PWM:%d    ",
    //          (long)acc_filt[0], (long)acc_filt[1], (long)acc_filt[2],
    //          fix2int15(complementary_angle), fix2int15(desired_angle), (int)control);
    // writeString(buf);

    // setCursor(0, 34);

    // snprintf(buf, sizeof(buf),
    //          "    Kp:%.2f  Ki:%.2f Kd:%.2f             ",
    //          fix2float15(kp), fix2float15(ki), fix2float15(kd));
    // writeString(buf);

    // setCursor(0, 34);
}

// ========= PID Control ======================================

// // Proportional Control
// void proportional_control (void) {
//     error = desired_angle - complementary_angle;
//     control = fix2int15(multfix15(kp, error));
// }
// // Integral Control
// void integral_control (void) {
//     error = desired_angle - complementary_angle;
//     integral_sum += multfix15(error, dt);
//     control = fix2int15(multfix15(kp, error) + (ki * integral_sum));
// }

// PID Control
void derivative_control (void) {
    fix15 prior_error = error;
    error = desired_angle - complementary_angle;
    delta_error = error - prior_error;
    integral_sum += multfix15(error, dt);
    // // Integral Sliding Window Buffer Logic
    // if (head == integral_size) {head = 0;}
    // integral_sum -= buffer[head];
    // buffer[head] = multfix15(error, dt);
    // integral_sum += buffer[head];
    // head += 1;


    // Clamp integral contribution to avoid windup
    fix15 integral_contribution = multfix15(ki, integral_sum);
    if (integral_contribution > int2fix15(1500)) {
        integral_contribution = int2fix15(1500);
    } else if (integral_contribution < int2fix15(-1500)) {
        integral_contribution = int2fix15(-1500);
    }
    control = fix2int15(multfix15(kp, error) + integral_contribution + multfix15(kd, gyro[2])); // kd times rate gyro (deg/s)
}
    
///////// ======== Button ==========
#define BUTTON_PIN 15
#define DEBOUNCE_US 15000

// Button State
typedef enum {
  NotPressed = 0,
  Pressed = 1
} ButtonState;
static volatile ButtonState button_mode = NotPressed;

static void button_init(void) {
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN);  // internal ~50k pull-up
}

// Debouncing Variables used in update_button()
//static volatile bool button_event = false;
static bool button_state = true; // 0 for Pressed, 1 for released
static bool button_last_raw = true;
static uint32_t button_last_change_us = 0;

// Timed Button Release Function
void button_release(void) {
    // Get current time
    uint32_t current_time = time_us_32();
    // Set angle to 90 for 5 seconds
    desired_angle = int2fix15(90);
    while ((time_us_32() - current_time) < (5000000)) {
        // Do Nothing
    }
    // Set angle to 120 for 5 seconds
    integral_sum = int2fix15(0);
    desired_angle = int2fix15(120);
    while ((time_us_32() - current_time) < (10000000)) {
        // Do Nothing
    }
    // Set angle to 60 for 5 seconds
    integral_sum = int2fix15(0);
    desired_angle = int2fix15(60);
    while ((time_us_32() - current_time) < (15000000)) {
        // Do Nothing
    }
    // Set angle to 90
    integral_sum = int2fix15(0);
    desired_angle = int2fix15(90);
}

// ========= Interrupt Service Routine ======================================
// Triggered each time PWM counter WRAPs (1kHz)
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));

    // Update duty cycle
    if (control!= old_control) {
        if (control > 3000) {
            control = 3000;
        } else if (control < 0) {
            control = 0;
        }
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    FilteredAccel();

    // mark that ISR ran
    isr_count++;

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

    //proportional_control();
    //integral_control(); // Not Yet Implemented
    derivative_control(); // Not Yet Implemented
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    printf("vga thread started\n"); fflush(stdout);

    // We will start drawing at column 81
    static int xcoord = 81 ;

    // Variables to store previous plot points for smooth lines -- added this
    static int prev_pwm_y = 355 ;      // Initialize to middle of PWM plot
    static int prev_angle_y = 155 ;    // Initialize to middle of angle plot

    // Low pass filter for PWM display - similar to accelerometer filter
    static int pwm_filtered = 1500 ;   // Initialize to middle PWM value

    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor2(WHITE, BLACK);

    // Draw bottom plot - PWM Control Output
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;

    // Y-axis labels for bottom plot - moved back to left side
    snprintf(plot_text, sizeof(plot_text), "1500");
    setCursor(35, 350) ;
    writeString(plot_text) ;

    snprintf(plot_text, sizeof(plot_text), "3000");
    setCursor(35, 280) ;
    writeString(plot_text) ;

    snprintf(plot_text, sizeof(plot_text), "0");
    setCursor(50, 425) ;
    writeString(plot_text) ;

    // Bottom plot title - moved back to left side
    snprintf(plot_text, sizeof(plot_text), "PWM");
    setCursor(15, 300) ;
    writeString(plot_text) ;

    snprintf(plot_text, sizeof(plot_text), "Control");
    setCursor(15, 315) ;
    writeString(plot_text) ;

    // Draw top plot - Complementary Angle (degrees)
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;

    // Y-axis labels for top plot - moved back to left side
    snprintf(plot_text, sizeof(plot_text), "%.1f", 90.);
    setCursor(50, 150) ;
    writeString(plot_text) ;

    snprintf(plot_text, sizeof(plot_text), "%.1f", 180.);
    setCursor(45, 75) ;
    writeString(plot_text) ;

    snprintf(plot_text, sizeof(plot_text), "%.1f", 0.);
    setCursor(50, 225) ;
    writeString(plot_text) ;

    // Top plot title - moved back to left side
    snprintf(plot_text, sizeof(plot_text), "Angle");
    setCursor(15, 100) ;
    writeString(plot_text) ;

    snprintf(plot_text, sizeof(plot_text), "(deg)");
    setCursor(10, 115) ;
    writeString(plot_text) ;


    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // diagnostic: show ISR activity when VGA thread is signaled
        //  printf("vga: semaphore signaled, isr_count=%d\n", isr_count); fflush(stdout);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) {
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Apply low pass filter to PWM control output for smoother display
            // Using same filter approach as accelerometer: y[n] =y[n-1] + α * (x[n] - y[n-1])
            // α = 1/8 (>>3 shift) for moderate filtering
            pwm_filtered = pwm_filtered + ((control - pwm_filtered) >> 3);

            // Draw bottom plot - PWM Control Output (0-3000 range mapped to 150 pixels)
            // Use filtered value for display instead of raw control value
            int pwm_plot_y = 430 - (int)((pwm_filtered * 150) / 3000) ;
            if (pwm_plot_y < 280) pwm_plot_y = 280 ;  // Clamp to plot area
            if (pwm_plot_y > 430) pwm_plot_y = 430 ;

            // Draw line from previous point to current point for smooth plotting
            if (xcoord > 81) {
                drawLine(xcoord-1, prev_pwm_y, xcoord, pwm_plot_y, YELLOW) ;
            }
            prev_pwm_y = pwm_plot_y ;  // Store current point for next iteration

            // Draw top plot - Complementary Angle (0-180 degrees mapped to 150 pixels)
            int angle_plot_y = 230 - (int)((fix2int15(complementary_angle) * 150) / 180) ;
            if (angle_plot_y < 80) angle_plot_y = 80 ;   // Clamp to plot area
            if (angle_plot_y > 230) angle_plot_y = 230 ;

            // Draw line from previous point to current point for smooth plotting
            if (xcoord > 81) {
                drawLine(xcoord-1, prev_angle_y, xcoord, angle_plot_y, WHITE) ;
            }
            prev_angle_y = angle_plot_y ;  // Store current point for next iteration

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
                // Reset previous points when wrapping around
                prev_pwm_y = pwm_plot_y ;
                prev_angle_y = angle_plot_y ;
            }

/////////////moved
            setTextSize(1);
            setTextColor2(WHITE, BLACK);
            setCursor(0, 22);

            snprintf(buf, sizeof(buf),
                    "    Accel(filt): X:%ld Y:%ld Z:%ld  CompAng:%d deg  desired angle:%d  PWM:%d    ",
                    (long)acc_filt[0], (long)acc_filt[1], (long)acc_filt[2],
                    fix2int15(complementary_angle), fix2int15(desired_angle), (int)control);
            writeString(buf);

            setCursor(0, 34);

            snprintf(buf, sizeof(buf),
                    "    Kp:%.2f  Ki:%.2f Kd:%.2f             ",
                    fix2float15(kp), fix2float15(ki), fix2float15(kd));
            writeString(buf);

            setCursor(0, 34);

        }
    }
    // Indicate end of thread
    PT_END(pt);
}


// Button Press Thread
static PT_THREAD (protothread_button(struct pt *pt)) {
    PT_BEGIN(pt);

    bool raw = gpio_get(BUTTON_PIN);  // 1 = released, 0 = pressed
    uint32_t now = time_us_32();

    // Button DEBOUNCING:
    // edge on raw input? start debounce timer
    if (raw != button_last_raw) {
      button_last_raw = raw;
      button_last_change_us = now;
    }

    // stable longer than debounce window? accept as new state
    if ((now - button_last_change_us) > DEBOUNCE_US) {
        if (raw != button_state) {
        button_state = raw;
        //printf("button pressed");
        // falling edge (pressed)
        if (button_state == false) {
            // Set positon to 0
            integral_sum = int2fix15(0);
            desired_angle = int2fix15(0);
        }
        // rising edge (released)
        if (button_state == true) {
            // NotPressed -> Pressed
            printf("button released");
            button_release();
        }
      }
    }

    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    printf("serial thread started\n"); fflush(stdout);
    static char classifier ;
    static int test_in ;
    static float float_in ;
    static int ch;
    static int idx;
    while(1) {
        // Prompt using stdio so it appears on the same USB serial as printf
        printf("input a command: ");
        fflush(stdout);

        // Read a line (non-blocking) into pt_serial_in_buffer using getchar_timeout_us
        idx = 0;
        memset(pt_serial_in_buffer, 0, pt_buffer_size);
        // wait for first character
        do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            while (ch != '\r' && ch != '\n') {
                // echo
                putchar(ch);
                fflush(stdout);
            if (ch == pt_backspace) {
                if (idx > 0) idx--;
            } else if (idx < pt_buffer_size - 1) {
                pt_serial_in_buffer[idx++] = (char)ch;
            }
            // get next char
            do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
        }
            putchar('\n');
            fflush(stdout);

        // debug: show the raw line we received before parsing
        printf("DBG: line='%s'\n", pt_serial_in_buffer);
        fflush(stdout);

        // parse classifier (skip leading whitespace)
        sscanf(pt_serial_in_buffer, " %c", &classifier);
        printf("DBG: classifier='%c' (0x%02x)\n", classifier, (unsigned char)classifier);
        fflush(stdout);

        if (classifier == 't') {
            printf("timestep: "); fflush(stdout);
            // read line
            idx = 0; memset(pt_serial_in_buffer, 0, pt_buffer_size);
            do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            while (ch != '\r' && ch != '\n') {
                putchar(ch);
                if (ch == pt_backspace) { if (idx > 0) idx--; }
                else if (idx < pt_buffer_size - 1) pt_serial_in_buffer[idx++] = (char)ch;
                do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            }
            putchar('\n');
            sscanf(pt_serial_in_buffer, "%d", &test_in);
            if (test_in > 0) threshold = test_in;
        }

        if (classifier == 'p') {
            printf("pwm (int): "); fflush(stdout);
            idx = 0; memset(pt_serial_in_buffer, 0, pt_buffer_size);
            do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            while (ch != '\r' && ch != '\n') {
                putchar(ch);
                if (ch == pt_backspace) { if (idx > 0) idx--; }
                else if (idx < pt_buffer_size - 1) pt_serial_in_buffer[idx++] = (char)ch;
                do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            }
            putchar('\n');
            sscanf(pt_serial_in_buffer, "%d", &test_in);
            if (test_in > 5000) continue;
            else if (test_in < 0) continue;
            else control = test_in;
        }

        if (classifier == 'a') {
            printf("desired angle (float): "); fflush(stdout);
            idx = 0; memset(pt_serial_in_buffer, 0, pt_buffer_size);
            do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            while (ch != '\r' && ch != '\n') {
                putchar(ch);
                if (ch == pt_backspace) { if (idx > 0) idx--; }
                else if (idx < pt_buffer_size - 1) pt_serial_in_buffer[idx++] = (char)ch;
                do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            }
            putchar('\n');
            sscanf(pt_serial_in_buffer, " %f", &float_in);
            if (float_in > 180) continue;
            else if (float_in < 0) continue;
            else {
                desired_angle = float2fix15(float_in);
                integral_sum = int2fix15(0);
            }
        }

        if (classifier == 'k') {
            printf("kp (float): "); fflush(stdout);
            idx = 0; memset(pt_serial_in_buffer, 0, pt_buffer_size);
            do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            while (ch != '\r' && ch != '\n') {
                putchar(ch);
                if (ch == pt_backspace) { if (idx > 0) idx--; }
                else if (idx < pt_buffer_size - 1) pt_serial_in_buffer[idx++] = (char)ch;
                do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            }
            putchar('\n');
            sscanf(pt_serial_in_buffer, " %f", &float_in);
            if (float_in < 0) continue;
            else kp = float2fix15(float_in);
        }

        if (classifier == 'i') {
            printf("ki (float): "); fflush(stdout);
            idx = 0; memset(pt_serial_in_buffer, 0, pt_buffer_size);
            do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            while (ch != '\r' && ch != '\n') {
                putchar(ch);
                if (ch == pt_backspace) { if (idx > 0) idx--; }
                else if (idx < pt_buffer_size - 1) pt_serial_in_buffer[idx++] = (char)ch;
                do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            }
            putchar('\n');
            sscanf(pt_serial_in_buffer, " %f", &float_in);
            if (float_in < 0) continue;
            else ki = float2fix15(float_in);
        }

        if (classifier == 'd') {
            printf("kd (float): "); fflush(stdout);
            idx = 0; memset(pt_serial_in_buffer, 0, pt_buffer_size);
            do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            while (ch != '\r' && ch != '\n') {
                putchar(ch);
                if (ch == pt_backspace) { if (idx > 0) idx--; }
                else if (idx < pt_buffer_size - 1) pt_serial_in_buffer[idx++] = (char)ch;
                do { PT_YIELD(pt); ch = getchar_timeout_us(0); } while (ch == PICO_ERROR_TIMEOUT);
            }
            putchar('\n');
            sscanf(pt_serial_in_buffer, " %f", &float_in);
            if (float_in < 0) continue;
            else kd = float2fix15(float_in);
        }
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    // Run both VGA and serial protothreads on core 1
    pt_add_thread(protothread_vga);
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_button);
    // Inform host that core1 is starting the protothreads
    printf("core1: starting protothreads (vga + serial)\n"); fflush(stdout);
    pt_schedule_start;
}



int main() {

    // Overclock
 //   set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();
    printf("\n\nHello IMU World!\n");
    PT_SEM_INIT(&vga_semaphore, 0);

    // Initialize VGA
    initVGA() ;
    printf("VGA initialized\n"); fflush(stdout);

    // Initialize Button
    button_init();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
   // gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
   // pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);
    printf("core1 launched\n"); fflush(stdout);

    // Main (core 0) has no protothreads to schedule now; idle here.
    while (true) {
        sleep_ms(1000);
    }

}