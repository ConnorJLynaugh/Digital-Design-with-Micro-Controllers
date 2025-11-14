# Quadruped Robot Control for Raspberry Pi Pico W

This project ports a Python-based quadruped robot control system to C for the Raspberry Pi Pico W. The robot uses a PCA9685 PWM controller to control 12 servo motors (4 legs with 3 joints each).

## Hardware Requirements

- Raspberry Pi Pico W
- PCA9685 PWM/Servo Driver Board
- 12 servo motors
- Power supply for servos (typically 5-6V)

## Wiring

### I2C Connection (PCA9685 to Pico W)
- SDA: GPIO 4 (Pin 6)
- SCL: GPIO 5 (Pin 7)
- VCC: 3.3V
- GND: GND

**Note:** The servo motors should be powered separately from the Pico (use the PCA9685's power input).

### Servo Connections to PCA9685
```
Leg 1 (Front Right):
- Channel 10: Joint
- Channel 11: Thigh
- Channel 12: Calf

Leg 2 (Front Left):
- Channel 5: Joint
- Channel 4: Thigh
- Channel 3: Calf

Leg 3 (Back Right):
- Channel 2: Joint
- Channel 1: Thigh
- Channel 0: Calf

Leg 4 (Back Left):
- Channel 13: Joint
- Channel 14: Thigh
- Channel 15: Calf
```

## Building the Project

### Prerequisites

1. Install the Raspberry Pi Pico SDK:
   ```bash
   git clone https://github.com/raspberrypi/pico-sdk.git
   cd pico-sdk
   git submodule update --init
   ```

2. Set the PICO_SDK_PATH environment variable:
   ```bash
   export PICO_SDK_PATH=/path/to/pico-sdk
   ```

3. Install build tools:
   ```bash
   sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
   ```

### Build Instructions

1. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```

2. Configure and build:
   ```bash
   cmake ..
   make
   ```

3. This will generate two programs:
   - `robot_control.uf2` - Main robot control program
   - `servo_test.uf2` - Servo testing program

### Flashing to Pico W

1. Hold the BOOTSEL button on the Pico W while plugging it into USB
2. The Pico will appear as a USB mass storage device
3. Copy the `.uf2` file to the device
4. The Pico will automatically reboot and run the program

## Usage

### Servo Test Program

The servo test program cycles through all servos one at a time, moving them between min and max positions. This is useful for:
- Verifying servo connections
- Checking servo range of motion
- Identifying which servo is connected to which channel

Connect to the Pico via serial terminal (115200 baud) to see status messages.

### Robot Control Program

The main control program provides keyboard control over USB serial. After flashing, connect to the Pico using a serial terminal (115200 baud).

#### Commands

**Movement:**
- `w` - Move forward
- `s` - Move backward
- `a` - Rotate counterclockwise
- `d` - Rotate clockwise
- `q` - Move left (crab walk)
- `e` - Move right (crab walk)
- `t` - Creep forward (slower gait)
- `g` - Creep backward (slower gait)

**Special Actions:**
- `h` - Wave/say hi
- `c` - Shuffle through positions
- `v` - Humping motion
- `b` - Squats (all sides)

**Positions:**
- `x` - Neutral X position
- `1-4` - Shift weight to position 1-4
- `i` - Sit down
- `u` - Stand up
- `l` - Legs up position

**Other:**
- `m` - Show menu
- `p` - Exit program and sit down

## Serial Terminal Setup

### Windows
Use PuTTY, TeraTerm, or similar:
- Baud rate: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

### Linux/Mac
```bash
screen /dev/ttyACM0 115200
```
or
```bash
minicom -D /dev/ttyACM0 -b 115200
```

## Project Structure

```
.
├── CMakeLists.txt          # Build configuration
├── pca9685.h               # PCA9685 driver header
├── pca9685.c               # PCA9685 driver implementation
├── movement_library.h      # Robot movement functions header
├── movement_library.c      # Robot movement functions implementation
├── main.c                  # Main robot control program
├── servo_test.c            # Servo testing program
└── README.md               # This file
```

## Customization

### Adjusting Servo Range

If your servos don't move to the correct positions, you may need to adjust the PWM values in `movement_library.c`. The `angles_to_pwm()` function maps angles to PWM values for each servo.

### Changing I2C Pins

Edit these defines in `main.c` and `servo_test.c`:
```c
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
```

### Modifying Movement Patterns

All movement functions are in `movement_library.c`. You can adjust timing, angles, and sequences to customize the robot's behavior.

## Troubleshooting

1. **Servos don't move:**
   - Check power supply to PCA9685
   - Verify I2C connections
   - Run the servo test program to identify issues

2. **Erratic servo movement:**
   - Ensure adequate power supply for servos
   - Check for loose connections
   - Verify PWM values are within servo range

3. **Can't connect to serial:**
   - Make sure USB is properly connected
   - Try a different USB cable
   - Check device permissions (Linux: add user to dialout group)

4. **Build errors:**
   - Verify PICO_SDK_PATH is set correctly
   - Ensure all SDK submodules are initialized
   - Check that all required tools are installed

## Original Python Version

This C port is based on the original Python implementation that used:
- Adafruit PCA9685 library
- Pygame for keyboard input
- PiCamera for camera functions

The camera functionality has been removed in this port, focusing purely on servo control.

## License

This code is based on the original work by Tony DiCola (Public Domain) and has been adapted for Raspberry Pi Pico W.
