# Instruction on how to install and use the Weart Glove Device

## Build and Install

1. Install the `WeArtGloveLibRawData`
2. clone the repo
3. `mkdir && cd build`
4. `cmake -G"Visual Studio 17 2022" ..`
5. `cmake --build . --target INSTALL --config Release`
6. Download and Install the latest WEARTMiddleware [link](https://weart.it/developer-guide/).

### Parameters Manuals

1. If you want to use the actuators, change the parameters below to `true`:

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
