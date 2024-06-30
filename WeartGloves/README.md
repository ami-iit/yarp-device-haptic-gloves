# Instruction on how to install and use the Weart Glove Device

## Build and Install

1. Install the `WeArtGloveLib` following the instruction [here](https://github.com/ami-iit/WEART-SDK-Cpp/tree/devel_ami-iit#ami-iit).
2. clone the repo `git clone https://github.com/ami-iit/yarp-device-haptic-gloves.git`
3. `git checkout devel_WeartGlove`
4. `mkdir && cd build`
5. `cmake -G"Visual Studio 17 2022" ..`
6. Notice that the files are going to be installed to `C:\Program Files (x86)\YOUR_PROJECT_NAME\bin`
    - If you want to change `CMAKE_INSTALL_PREFIX`, run `ccmake ..`
    - In our case: `C:<PATH_TO_ROBOTOLOGYSUPERBUILD>\build\install`
7. `cmake --build . --target INSTALL --config Release`
8. Download and Install the latest WEARTMiddleware [link](https://weart.it/developer-guide/).

### Parameters Manuals

1. If you want to enable the haptic feedback, change the parameters below to `true`:

```text
use_force_feedback
use_temperature_feedback 
use_texture_feedback
```

2. In order to change the smoothing factor:

```text
smoothing_factor
```

Please keep in mind that the accepted value for this parameter is between 0 and 1 (_0 is not acceptable_)

3. Tracking Algorithm (TA):
It is possible to choose the type of tracking algorithm with the `tracking_algorithm` paramater. The available TAs are:
    - **Simple Closure-based Tracking Algorithm (SCTA)**: This TA uses only closure information reading from a time of flight (ToF) sensor.

    - **Advanced Sensor-based Tracking Algorithm (ASTA)**: This TA uses raw sensory information reading from Inertial Measurement Unit (IMU) sensor beside the ToF sensor.
