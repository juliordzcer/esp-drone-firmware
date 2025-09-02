## IDF Install
### Step 1. Install Prerequisites
To compile using ESP-IDF, you need to get the following packages. The command to run depends on which distribution of Linux you are using:
```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### Step 2. Get ESP-IDF
To get ESP-IDF, navigate to your installation directory and clone the repository with `git clone`, following instructions below specific to your operating system.

Open Terminal, and run the following commands:
```
mkdir -p ~/esp
cd ~/esp
git clone -b v5.0 --recursive https://github.com/espressif/esp-idf.git
```

ESP-IDF is downloaded into `~/esp/esp-idf`.

### Step 3. Set up the Tools
Aside from the ESP-IDF, you also need to install the tools used by ESP-IDF, such as the compiler, debugger, Python packages, etc, for projects supporting ESP32.
```
cd ~/esp/esp-idf
./install.sh esp32
```

The above commands install tools for ESP32 only. If you intend to develop projects for more chip targets then you should list all of them and run for example:
```
cd ~/esp/esp-idf
./install.sh esp32,esp32s2
```

In order to install tools for all supported targets please run the following command:
```
cd ~/esp/esp-idf
./install.sh all
```

### Step 4. Set up the Environment Variables

The installed tools are not yet added to the PATH environment variable. To make the tools usable from the command line, some environment variables must be set. ESP-IDF provides another script which does that.

In the terminal where you are going to use ESP-IDF, run:
```
. $HOME/esp/esp-idf/export.sh
```

If you plan to use esp-idf frequently, you can create an alias for executing `export.sh`:

1. Copy and paste the following command to your shell's profile (.profile, .bashrc, .zprofile, etc.)
```
alias get_idf='. $HOME/esp/esp-idf/export.sh'
```
2. Refresh the configuration by restarting the terminal session or by running `source [path to profile]`, for example, `source ~/.bashrc`.

Now you can run `get_idf` to set up or refresh the esp-idf environment in any terminal session.

Technically, you can add `export.sh` to your shell's profile directly; however, it is not recommended. Doing so activates IDF virtual environment in every terminal session (including those where IDF is not needed), defeating the purpose of the virtual environment and likely affecting other software.