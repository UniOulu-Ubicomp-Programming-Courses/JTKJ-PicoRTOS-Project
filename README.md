# TKJHAT project
This project includes the TKJHAT SDK and a set of use examples for the University of Oulu course [Computer Systems](https://lovelace.oulu.fi/tietokonej%C3%A4rjestelm%C3%A4t/tietokonej%C3%A4rjestelm%C3%A4t/). 

## Changelogs


### Project changelog
* **0.84** *(11-11-2025)* Updated the README.md files including changelog and list of examples. Version 0.83 of TKJHAT. Created hat_imu_display (not in course repo)
* **0.83** *(28-10-2025)* Added serial client examples: hello_serial_bidirectional_client and hello_serial_client. 
* **0.82** *(22-10-2025)* TKJHat version 0.82. Added example hat_imu_ex and hat_imu_cdc_ex
* **0.81** *(13-10-2025)* Added hat examples: hello_hat, hat_example. Solved problem in toolchain with Ninja path. Version 0.81 of the TKJHAT
* **0.80** *(05-10-2025)* First version ready.

### TKJHAT changelog
* **0.83** *(11-11-2025)*: Restructured documentation. Including names for pins. Restructured the topics section. 
* **0.82** *(22-10-2025)*: Solved bug that under certain circunstances did not allow to init correctly the IMU, making the I2C get stuck and without possibility of being used again. Corrected some links in documentation.
* **0.81** *(13-10-2025)*: Refactor IMU function to start sampling with default values. Hat initialization now includes I2C.
* **0.80** *(05-10-2025)*: First official version.

### usb-serial-debug changelog

* **0.80** *(05-10-2025)*:Initial working version

## Important links
[TKJHAT library](https://unioulu-ubicomp-programming-courses.github.io/JTKJ-PicoRTOS-Project/)

## List of examples

The **examples folder** contains a list of test programs that you can use to learn how to use different aspects of FreeRTOS and the TKJHAT SDK. The following list contains a description of each example and the target necessary to compile and run the code between parenthesis.

### Hello Pico

* **hello_pico** (*hello_pico*): Another Hello World app for the Raspberry Pi Pico that does not need either FreeRTOS or the TKJHAT SDK. Just print in terminal periodically.
* **hello_blink** (*hello_blink*): Example application to run without the need of the TKJHAT. It blinks the Raspberry Pi Pico LED. 

### Hello FreeRTOS

* **hello_freertos** (*hello_freertos*): Hello World application for using FreeRTOS. TKJHAT not needed
* **hello_blink_freertos** (*hello_blink_freertos*): Same as before but the blinking happens inside a FreeRTOS task.

### Hello JTKJHAT

* **hello_hat** (*hello_hat*): Hello World application for the JTKHAT SDK. It tests the buzzer, device LED and the LCD screen 
* **hat_example** (*hat_example*): Code not finished yet. Few examples to test running different sensors using different tasks. Do not use yet.
* **hello_dual_cdc** (*hello_dual_cdc*): Example of using the usb-serial-debug to open two different serial ports via USB using TinyUSB library. One of the ports uses the helpers of the usb-serial-debug library instead of ```printf``` to send string to the terminal. 
* **hat_imu_ex** (*hat_imu_ex*): Example on how to use the IMU sensor in a FreeRTOS task to collect acceleration and gyroscope data and printing it in the terminal. 
* **hat_imu_display** (*hat_imu_display*): Same as before but module of acceleration data is presented in the LCD display. Two FreeRTOS tasks in use: one to collect data and other to print datat in the LCD.
* **hat_imu_cdc_ex**(*hat_imu_cdc_ex*): Another example of collecting data using the IMU. In this case data is sent to two different terminals using the usb-serial-debug library. 
* **hello_microphone** (*test_microphone*): Application that configures and sets up the microphone using the JTKJSDK api. Collects microphone sample. PCM samples are sent to the terminal. The script located at *tools/record_audio.sh* can be used to collect the samples and added to a .wav file that can be played. It needs to have Sox as dependency.  The file *tools/play_stream_audio.sh* plays directly the audio, storing it first in a buffer. 

### Computer System Course specific examples

* **compilation_errors** (*compilation_error*): This code is a broken code that is used in the lab session 2 of the course [Computer Systems](https://lovelace.oulu.fi/tietokonej%C3%A4rjestelm%C3%A4t/tietokonej%C3%A4rjestelm%C3%A4t/)
* **hello_serial_client** (*hello_serial_client*): It uses the protocol of the [https://github.com/Gr701/serial_client|Serial-client] application, used to translate ```.``` and ```-``` to Morse code. It only sends data to the application.  
* **hello_serial_bidirectional_client** (*hello_serial_client*): It uses the protocol of the [https://github.com/Gr701/serial_client|Serial-client] application, used to translate ```.``` and ```-``` to Morse code. It sends data to the application and is able to process received data and echo it as a comment to the Serial-client application.  


## Installation in Linux with VSCode extension

This section explains how to install and configure a **development environment for Raspberry Pi Pico (and Pico 2)** using **Lubuntu 24.04**.  
The setup includes **C/C++ compilation tools**, **VS Code**, **FreeRTOS**, and **USB access configuration**.  

---

### Step 1: Virtual Machine Setup (Optional)

Create a virtual machine with the following specifications:

- **Name:** `ComputerSystems_Pico`  
- **Memory:** 4 GB  
- **Hard Disk:** 25 GB  
- **Display:** 128 MB  

Once the VM is created and Lubuntu 24.04 is installed:

1. **Install Guest Additions** (from the VirtualBox menu).  

---

### Step 2: Basic Tools Installation

First, update the package index:
```bash
sudo apt update
```

Then install the essential tools for C programming and general development:
```bash
sudo apt -y install build-essential gcc gdb make git cmake
```

Install dependencies needed for [Pico SDK](https://github.com/raspberrypi/pico-sdk), debugging, and Python tools:
```bash
sudo apt install python3 python-is-python3 tar gdb-multiarch
```

Install additional libraries for **[OpenOCD](https://openocd.org/)** (for debugging):
```bash
sudo apt install libusb-1.0-0-dev libhidapi-dev
```

Install **Firefox** (optional but useful):
```bash
sudo apt install firefox
```

---

### Step 3: USB Permissions for Pico and OpenOCD


Allow USB serial and debugging devices access for your current user:
   ```bash
   sudo usermod -aG dialout,plugdev $USER
   ```

Add udev rules so you can use the Pico without `sudo`:

1. Download and copy the following files to `/etc/udev/rules.d`:
   - [60-picotool.rules](https://github.com/raspberrypi/picotool/blob/master/udev/60-picotool.rules)
   - [60-openocd.rules](https://github.com/raspberrypi/openocd/blob/sdk-2.0.0/contrib/60-openocd.rules)

2. Reload the udev rules:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

---

### Step 4: Installing Visual Studio Code

Install VS Code using Snap:
```bash
sudo snap install code --classic
```

Then, open VS Code and install the following extensions from the **Marketplace**:
- **C/C++** (by Microsoft)
- **CMake Tools**
- **Raspberry Pi Pico** (by Raspberry Pi)

---

### Step 5: Create a Test Project

Create a simple workspace for your C projects:
```bash
mkdir -p ~/c-playground/helloworld
cd ~/c-playground/helloworld
```

You can later use this folder to test your setup with simple “Hello, World” programs.

-----


### Step 6: Setting Up FreeRTOS

Install and configure a local copy of the FreeRTOS Kernel so that your CMake configuration can link it to your Pico project.

1. Create a hidden folder for FreeRTOS and clone the kernel
```bash
mkdir ~/.freertos
cd ~/.freertos
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git .
git checkout V11.2.0
git submodule update --init portable/ThirdParty/Community-Supported-Ports
```

2. Download the **CMake import file**:  
- [FreeRTOS_Kernel_import.cmake](https://github.com/FreeRTOS/FreeRTOS-Kernel-Community-Supported-Ports/blob/main/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)

Place it later in the **root of your project**.

3. Add FreeRTOS Path to environment variable. 
You can either:

**Option 1 – Add to `.bashrc`:**
```bash
echo 'export FREERTOS_KERNEL_PATH="$HOME/.freertos"' >> ~/.bashrc
source ~/.bashrc
```

**Option 2 – Add to VS Code `settings.json`: inside your project vscode folder** 
```json
"cmake.configureEnvironment": {
  "FREERTOS_KERNEL_PATH": "${env:HOME}/.freertos"
},
"terminal.integrated.env.linux": {
  "FREERTOS_KERNEL_PATH": "${env:HOME}/.freertos"
}
```

4. Download and save the **FreeRTOSConfig.h** file from:  
[FreeRTOSConfig_examples_common.h](https://github.com/raspberrypi/pico-examples/blob/master/freertos/FreeRTOSConfig_examples_common.h)

It generally goes in the ```config``` folder of your project. 

---

### Step 7: (Optional) Installing Doxygen 

If you plan to generate documentation from your source code (for example, from comments written in Doxygen format), you can install **Doxygen** as follows:

```bash
sudo apt update
sudo apt install doxygen graphviz
```

---

### Step 8: Modify Your CMakeLists.txt

[Check this project CMakeLists.txt](./CMakeLists.txt) as good example. You can use this file as a reference for setting up your own project structure.

Include the Pico SDK and FreeRTOS kernel:
```cmake
# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Include FreeRTOS Kernel libraries
include(FreeRTOS_Kernel_import.cmake)

# Include config only for FreeRTOS target. 
target_include_directories(FreeRTOS-Kernel INTERFACE
${CMAKE_CURRENT_LIST_DIR}/config
)
```

Then, define your executable and link libraries:
```cmake
# Add executable
add_executable(blink
  src/main.c
)

# Link libraries
target_link_libraries(blink
  pico_stdlib
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)
```

---

### Step 9: Adjust VS Code Settings

Once the project is created, set the following configuration in `settings.json`:
```json
"raspberry-pi-pico.cmakeAutoConfigure": false,
"raspberry-pi-pico.useCmakeTools": true,
"cmake.configureOnEdit": true,
"cmake.automaticReconfigure": true,
"cmake.configureOnOpen": true
```

This ensures you’re using the CMake Tools extension rather than the Pico extension for building projects.

NOTE: At some point the Ninja stop working inside the project (CMake could not find Ninja).
Hence I had to add it by hand in `settings.json` //ADD RIGHT PATH TO NINJA. GENERALLY NINJA IS AUTOMATICALLY DETECTED, BUT FOR SOME REASON NOT ANYMORE. 
    ```json
    "cmake.configureArgs": [
        "-DCMAKE_MAKE_PROGRAM:FILEPATH=${userHome}/.pico-sdk/ninja/v1.12.1/ninja"
    ]
    ```

---

### Step 10: Verify Installation

To verify everything is working:
1. Plug in your Raspberry Pi Pico.
2. In VS Code, open the **Raspberry Pi Pico** extension.
3. Try to build and upload a simple example (like `blink`).
4. Check the output in the serial terminal.