# Instruction on how to install and use the Weart Glove Device

## Build and Install

1. Install the `WeArtGloveLibRawData`
2. clone the repo
3. `mkdir && cd build`
4. `cmake -G"Visual Studio 17 2022" ..`
5. `cmake --build . --target INSTALL --config Release`
6. Install the WEARTMiddleware V2.1.10 [link](https://istitutoitalianotecnologia.sharepoint.com/:f:/r/sites/DynamicInteractionControl/Documenti%20condivisi/Telexistence/WEART%20Haptic%20Glove/Middleware?csf=1&web=1&e=sbJScy)

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

3. Please make sure `Send sensors data to client` is checked in the `Show UI test panel` of the WEARTMiddleware
