/**
 * @file WeArtGlove.h
 * @authors Ehsan Ranjbari <ehsan.ranjbari@iit.it>
 * @copyright 2022 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2022
 */

#ifndef WEART_GLOVE_H
#define WEART_GLOVE_H

#include <Wearable/IWear/IWear.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/ISerialDevice.h>
#include <yarp/os/PeriodicThread.h>

namespace wearable {
    namespace devices {
        class WeArtGlove;
    } // namespace devices
} // namespace wearable

class wearable::devices::WeArtGlove
    : public yarp::dev::DeviceDriver
    , public yarp::os::PeriodicThread
    , public yarp::dev::IPreciselyTimed
    , public wearable::IWear
{
private:
    class WeArtGloveImpl;
    std::unique_ptr<WeArtGloveImpl> pImpl;

public:
    WeArtGlove();
    ~WeArtGlove() override;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

    // ================
    // IPRECISELY TIMED
    // ================

    yarp::os::Stamp getLastInputStamp() override;

    // =====
    // IWEAR
    // =====

    // GENERIC
    // -------

    WearableName getWearableName() const override;

    WearStatus getStatus() const override;

    TimeStamp getTimeStamp() const override;

    SensorPtr<const sensor::ISensor> getSensor(const sensor::SensorName name) const override;

    VectorOfSensorPtr<const sensor::ISensor> getSensors(const sensor::SensorType) const override;

    ElementPtr<const actuator::IActuator>
    getActuator(const actuator::ActuatorName name) const override;

    VectorOfElementPtr<const actuator::IActuator>
    getActuators(const actuator::ActuatorType type) const override;

    // IMPLEMENTED SENSORS
    // -------------------

    SensorPtr<const sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const sensor::SensorName name) const override;
    SensorPtr<const sensor::IVirtualJointKinSensor>
    getVirtualJointKinSensor(const sensor::SensorName name) const override;

    // IMPLEMENTED ACTUATORS
    // ---------------------

    ElementPtr<const actuator::IHaptic>
    getHapticActuator(const actuator::ActuatorName name) const override;

    // UNIMPLEMENTED SENSORS
    // ---------------------

    inline SensorPtr<const sensor::IForce3DSensor>
    getForce3DSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ITorque3DSensor>
    getTorque3DSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IMagnetometer>
    getMagnetometer(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IOrientationSensor>
    getOrientationSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IPoseSensor>
    getPoseSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IPositionSensor>
    getPositionSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IAccelerometer>
    getAccelerometer(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IEmgSensor>
    getEmgSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IGyroscope>
    getGyroscope(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ISkinSensor>
    getSkinSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ITemperatureSensor>
    getTemperatureSensor(const sensor::SensorName /*name*/) const override;

    // UNIMPLEMENTED ACTUATORS
    // -----------------------

    inline ElementPtr<const actuator::IHeater>
    getHeaterActuator(const actuator::ActuatorName) const override;

    inline ElementPtr<const actuator::IMotor>
    getMotorActuator(const actuator::ActuatorName) const override;
};

inline wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
wearable::devices::WeArtGlove::getForce3DSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
wearable::devices::WeArtGlove::getTorque3DSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
wearable::devices::WeArtGlove::getForceTorque6DSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
wearable::devices::WeArtGlove::getFreeBodyAccelerationSensor(
    const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IMagnetometer>
wearable::devices::WeArtGlove::getMagnetometer(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
wearable::devices::WeArtGlove::getOrientationSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IPoseSensor>
wearable::devices::WeArtGlove::getPoseSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IPositionSensor>
wearable::devices::WeArtGlove::getPositionSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
wearable::devices::WeArtGlove::getVirtualSphericalJointKinSensor(
    const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IAccelerometer>
wearable::devices::WeArtGlove::getAccelerometer(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IEmgSensor>
wearable::devices::WeArtGlove::getEmgSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IGyroscope>
wearable::devices::WeArtGlove::getGyroscope(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ISkinSensor>
wearable::devices::WeArtGlove::getSkinSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
wearable::devices::WeArtGlove::getTemperatureSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IHeater>
wearable::devices::WeArtGlove::getHeaterActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IMotor>
wearable::devices::WeArtGlove::getMotorActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

#endif // WEART_GLOVE_H
