## HapticGlove

Here we are exposing the [SenseGlove SDK](https://github.com/Adjuvo/SenseGlove-API) as a wearable device. This device expose the interfaces in order to stream hand motion data and setting the haptic feedback to the glove through the YARP netwrok.

The following interfaces are exposed for each hand/glove:

- human hand joint angles (computed by SenseGlove SDK) `[20 DoF (each finger 4 DoF), rad]`
- glove fingertip poses attached to the human fingertip wrt human frame `5 pose vectors [<position, quaternion>]`
- glove hand pose (using the hand IMU data) wrt human hand inertial frame`Pose [<position, quaternion>]`
- hand fingertip vibrotactile feedbacks `[5 values: 0.0 - 1.0]`
- hand fingertip forceFeedback feedbacks `[5 values: 0.0 - 1.0]`
- hand palm vibrotactile feedback `[1 value: 0.0 - 1.0]`


### Dependencies

- Sense Glove SDK
```
git clone https://github.com/Adjuvo/SenseGlove-API
cd SenseGlove-API
git checkout v2.102.1
```
In a Linux machine add following environment variable:

```
export SenseGlove_DIR= <path tho the SenseGlove-API Folder>
export PATH=${PATH}:${SenseGlove_DIR}/lib/linux/v22/x86-64/<build type(Release or Debug)> 
```
In a Windows machine following environment variable:

```
set SenseGlove_DIR= <path tho the SenseGlove-API Folder>
set PATH=${PATH}:${SenseGlove_DIR}/lib/win64/<msvc version>/<build type(Release or Debug)> 
```
where `<msvc version>` is the the Visual Studio version you are using:
- `msvc142` for Visual Studio 2019.
- `msvc143` for Visual Studio 2022.

### BUILD

Enable the option `YARP_DEVICE_SENSE_GLOVES_ENABLE`.

### RUN

After building and installing the project.

- Run the SenseCom executable as indicated in the README on the SenseCom folder of the SenseGlove-API repository.
    
- Then run:
```
yarprobotinterface --config SenseGlove.xml
```

**N.B. when `SenseCom` runs the glove colors in GUI should be blue. If it is not, in Linux, try to do:**
``` 
sudo adduser $USER dialout
```
