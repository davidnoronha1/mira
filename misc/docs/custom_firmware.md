# Custom Firmware 

This repository contains the custom ArduSub firmware and thruster configurations for our unique thruster configuration. We are using **ArduSub 4.5.7** compiled for the Pixhawk 2.4.8 `fmuv3`.

## Step 1: Install System Dependencies and setup environment
Before touching the codebase, install the raw C-libraries required to build the MAVLink telemetry generators. 

Open your terminal and run:

    sudo apt update
    sudo apt install -y libxml2-dev libxslt1-dev zlib1g-dev build-essential

`waf` mostly likely uses a different version of Python than what you would have on your system. To isolate our Python version and prevent breaking our main operating system's Python setup, we will use **`uv`**.

Run the following in your terminal to install `uv`:

    curl -LsSf https://astral.sh/uv/install.sh | sh

*(Note: Close and reopen your terminal after this finishes so the `uv` command is recognized).*

## Step 2: Clone the Repository and Sync Submodules

    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot

    git fetch --all --tags 

We are currently using the latest stable version of ArduSub (4.5.7) change the version accordingly.

    git checkout ArduSub-4.5.7

**Do not skip this!!**

    git submodule update --init --recursive --force

## Step 3: Lock the Python environment and install dependencies.

We will use `uv` to instantly download Python 3.9 and create an isolated virtual environment (`.venv`) just for this folder.

    uv venv --python 3.9

Activate the environment (you must do this every time you open a new terminal to compile):

    source .venv/bin/activate

verify 

    python --version

**Install the MAVLink and text-generation dependencies:**

The `waf` compiler explicitly needs these older packages to generate the C++ telemetry code before it can build the firmware. `uv pip` installs these almost instantly:

    uv pip install future lxml empy==3.3.4

**We need exactly 3.3.4 version of empy**

## Step 4: Custom Thruster Matrix

**What is a Motor Matrix?**
A motor matrix tells the flight controller exactly how the thrusters are physically arranged on the vehicle. It defines how much force each individual motor contributes to the 6 degrees of freedom (roll, pitch, yaw, forward/surge, lateral/sway, and vertical/heave) so the drone moves correctly when given a command.

You have two ways to apply our custom motor matrix to the codebase before compiling:

### Option A: Use the Patch File (Recommended)
Applying the patch file automatically edits the codebase for you, preventing copy-paste errors. Ensure you are in the root `ardupilot` directory, then apply the patch (adjust the path if you saved the patch elsewhere in the repo):

    git apply ../mira_thrusters.patch

*(If you used the patch, skip Option B and go straight to Step 5).*

### Option B: Manual Edit
If you prefer to edit the code manually, change the `AP_Motors6DOF.cpp` file.

    cd libraries/AP_Motors/

    code AP_Motors6DOF.cpp

Now locate SUB_FRAME_CUSTOM (just do `ctrl+f` then search for custom). This the correct matrix that the bot is running with currently: 

    _frame_class_string = "DNT-VTC";
            add_motor_raw_6dof(AP_MOTORS_MOT_1,    -1.0f,         -1.0f,         -1.0f,               1.0f,              1.0f,                1.0f,              1);
            add_motor_raw_6dof(AP_MOTORS_MOT_2,     1.0f,          1.0f,          -1.0f,             -1.0f,              1.0f,                1.0f,             2);
            add_motor_raw_6dof(AP_MOTORS_MOT_3,    -1.0f,          1.0f,          1.0f,               1.0f,             -1.0f,                1.0f,              3);
            add_motor_raw_6dof(AP_MOTORS_MOT_4,     1.0f,         -1.0f,          1.0f,              -1.0f,             -1.0f,                1.0f,              4);
            add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,         -1.0f,          1.0f,               1.0f,              1.0f,               -1.0f,              5);
            add_motor_raw_6dof(AP_MOTORS_MOT_6,    -1.0f,          1.0f,          1.0f,              -1.0f,              1.0f,               -1.0f,              6);
            add_motor_raw_6dof(AP_MOTORS_MOT_7,     1.0f,          1.0f,         -1.0f,               1.0f,             -1.0f,               -1.0f,              7);
            add_motor_raw_6dof(AP_MOTORS_MOT_8,    -1.0f,         -1.0f,         -1.0f,              -1.0f,             -1.0f,               -1.0f,              8);
            break;

paste this inside the field. Navigate back to the ardupilot directory:

    cd ../..

## Step 5: Configure and Compile Custom Firmware
*(Make sure your `.venv` is still active!)*

    ./waf configure --board fmuv3 
    ./waf sub

**Our compiled firmware is here `build/fmuv3/bin/ardusub.apj`**

Just copy the firmware out in a directory you want so its easier to find when you wanna flash the pixhawk , i just copy it to my Desktop

    cp build/fmuv3/bin/ardusub.apj ~/Desktop/newFirmware.apj

## Step 6: Flash the Pixhawk with our custom firmware

1. Connect the pixhawk to your laptop.
2. Open QGC , vehicle settings -> firmware
3. Unplug and plug the pix to start the firmware upgrade 
4. From advanced options select custom firmware.
5. select the firmware file you compiled earlier.
6. boom you are done pix is flashed.