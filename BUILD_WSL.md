# Building on WSL (Windows Subsystem for Linux)

You have WSL installed, which is perfect! You can use the Linux build process.

## Setup (One-time)

### 1. Install build tools in WSL:
```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

### 2. Set environment variable in WSL:
```bash
echo 'export PICO_SDK_PATH=/mnt/c/Users/sgb14/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

### 3. Verify it's set:
```bash
echo $PICO_SDK_PATH
```
Should show: `/mnt/c/Users/sgb14/pico-sdk`

---

## Build Your Project

### 1. Navigate to your project folder:
```bash
cd /mnt/c/Users/sgb14/Documents/robot_project
```
(Put all your .c, .h, and CMakeLists.txt files here first)

### 2. Create build directory:
```bash
mkdir build
cd build
```

### 3. Configure and build:
```bash
cmake ..
make
```

### 4. Your .uf2 files are ready:
```bash
ls -lh *.uf2
```
You should see:
- `robot_control.uf2`
- `servo_test.uf2`

---

## Flash to Pico W

### From WSL:
The Pico will appear as a drive in Windows. From WSL:

```bash
# Copy to Pico drive (adjust drive letter if needed)
cp servo_test.uf2 /mnt/d/
```

Or just use Windows File Explorer:
1. Hold BOOTSEL on Pico + plug USB
2. Pico appears as drive (usually D: or E:)
3. Copy `servo_test.uf2` from your project folder to the Pico drive

---

## Serial Terminal from WSL

### Option 1: Using screen
```bash
sudo apt install screen
sudo screen /dev/ttyACM0 115200
```
Exit with: Ctrl+A then K, then Y

### Option 2: Using minicom
```bash
sudo apt install minicom
sudo minicom -D /dev/ttyACM0 -b 115200
```
Exit with: Ctrl+A then X

### Option 3: Using Python
```bash
sudo apt install python3-serial
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```
Exit with: Ctrl+]

---

## Complete Build Process (Copy-Paste)

```bash
# One-time setup
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
echo 'export PICO_SDK_PATH=/mnt/c/Users/sgb14/pico-sdk' >> ~/.bashrc
source ~/.bashrc

# Navigate to project
cd /mnt/c/Users/sgb14/Documents/robot_project

# Build
mkdir build
cd build
cmake ..
make

# Flash: Use Windows File Explorer to copy .uf2 file to Pico drive

# Connect serial
sudo apt install screen
sudo screen /dev/ttyACM0 115200
```

---

## Finding Serial Port in WSL

If `/dev/ttyACM0` doesn't work, find your port:
```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

Or use Windows Device Manager to see the COM port, then:
- COM3 = /dev/ttyS3
- COM4 = /dev/ttyS4
- etc.

---

## Troubleshooting

**"Permission denied" on serial port:**
```bash
sudo usermod -a -G dialout $USER
```
Then log out and back in.

**Can't find serial device:**
Make sure Pico is plugged in and running (not in BOOTSEL mode).

**Build errors about PICO_SDK_PATH:**
```bash
echo $PICO_SDK_PATH
```
Should show: `/mnt/c/Users/sgb14/pico-sdk`

If empty, run:
```bash
export PICO_SDK_PATH=/mnt/c/Users/sgb14/pico-sdk
```
