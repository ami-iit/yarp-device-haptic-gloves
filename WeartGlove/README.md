# Instruction on how to install and use the Weart Glove Device

⚠️ This device can only be installed on Windows.

## Dependencies

1. Install the `WeArtGloveLib` following the instruction [here](https://github.com/ami-iit/WEART-SDK-Cpp#installation-cmake).
2. Download and Install the latest WEARTMiddleware [link](https://weart.it/developer-guide/).
   - Latest tested version: v.2.3.0
3. Install `bipedallocomotionframework` following the instruction in [this link](https://github.com/ami-iit/bipedal-locomotion-framework/?tab=readme-ov-file#package-install-with-conda-recommended)
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
It is possible to choose the type of tracking algorithm with the `tracking_algorithm` paramater to one of `SCTA` or `ASTA`.
    - **Simple Closure-based Tracking Algorithm (SCTA)**: This TA uses only closure information reading from a time of flight (ToF) sensor.

    - **Advanced Sensor-based Tracking Algorithm (ASTA)**: This TA uses raw sensory information reading from Inertial Measurement Unit (IMU) sensor beside the ToF sensor.
      ⚠️ When using ASTA, the URDF model is required. Please make sure to add the path to the directory ``yarp-device-haptic-gloves/WeartGlove/models`` to the `YARP_DATA_DIRS` environmental variable.
