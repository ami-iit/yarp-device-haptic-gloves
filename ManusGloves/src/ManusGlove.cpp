/**
 * @file ManusGlove.cpp
 * @authors Ehsan Ranjbari <ehsan.ranjbari@iit.it>
 * @copyright 2024 AMI Lab - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2024
 */

#include <ManusGlove.h>
#include <ManusSDK.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ResourceFinder.h>

#include <assert.h>
#include <cmath>
#include <map>
#include <mutex>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstdlib>

#include <Eigen/Dense>
#include <Eigen/Core>

const std::string DeviceName = "ManusGlove";
const std::string LogPrefix = DeviceName + wearable::Separator;

using namespace wearable;
using namespace wearable::devices;

class ManusGlove::ManusGloveImpl
{
public:
    mutable std::mutex mutex;

    WearableName wearableName;

    TimeStamp timeStamp;

    double period;

    bool handSide;
    std::string handSideLogPrefix;

    bool hostType;

    std::vector<double> humanJointState;

    //Pointer to the glove object.
    std::unique_ptr<manusGlove::ManusGloveHelper> pGlove; /**< Pointer to the glove object. */

    // Vector of the human joint names
    std::vector<std::string> humanJointNameList;

    // Vector of the human finger names
    std::vector<std::string> humanFingerNames;

    // The name of the human hand link name
    std::string humanHandLinkName;

    // Link Sensor
    std::string linkSensorPrefix;
    class ManusGloveVirtualLinkKinSensor;
    std::vector<SensorPtr<ManusGloveVirtualLinkKinSensor>> manusGloveVirtualLinkKinSensor;

    // Joint Sensor
    std::string jointSensorPrefix;
    class ManusGloveVirtualJointKinSensor;
    std::vector<SensorPtr<ManusGloveVirtualJointKinSensor>> manusGloveJointSensorVector;

    // Constructor
    ManusGloveImpl();

    bool update();

    bool open(yarp::os::Searchable& config);

    bool close();

    // Utilities Methods


};

ManusGlove::ManusGloveImpl::ManusGloveImpl() {}

bool ManusGlove::ManusGloveImpl::open(yarp::os::Searchable& config)
{
    this->pGlove = std::make_unique<manusGlove::ManusGloveHelper>();
    // Parameters from xml file
    period = config.check("period", yarp::os::Value(0.01)).asFloat64();

    // Assigning the correct handside
    if (!config.check("is_right_hand"))
    {
        yError()<<"Cannot find the is_right_hand parameter";
        return false;
    }

    bool isRightHand = config.find("is_right_hand").asBool();
    handSide = isRightHand;
    if (handSide)
    {
        handSideLogPrefix = "Right";
    }
    else
    {
        handSideLogPrefix = "Left";
    }
    

    // Choose the host type: Remote/Local
    if (!config.check("is_local_host"))
    {
        yError()<<"Cannot find the is_local_host parameter";
        return false;
    }

    bool isLocalHost = config.find("is_local_host").asBool();
    hostType = isLocalHost;

    // Get Robot Joint limit
    // MIN
    yarp::os::Bottle* robotJointLimitMinYarp;
    if (!(config.check("min_joint_limit")) || !config.find("min_joint_limit").isList())
    {
        yError()<<LogPrefix<<"couldn't find robot joint limits, check the xml file";
        return false;
    }
    robotJointLimitMinYarp = config.find("min_joint_limit").asList();

    // MAX
    yarp::os::Bottle* robotJointLimitMaxYarp;
    if (!(config.check("max_joint_limit")) || !config.find("max_joint_limit").isList())
    {
        yError()<<LogPrefix<<"couldn't find robot joint limits, check the xml file";
        return false;
    }
    robotJointLimitMaxYarp = config.find("max_joint_limit").asList();

    // Get Human Joint Names
    yarp::os::Bottle* jointListYarp;
    if (!(config.check("human_joint_list") && config.find("human_joint_list").isList())) {
        yError() << LogPrefix << "Unable to find human_joint_list in the config file.";
        return false;
    }
    jointListYarp = config.find("human_joint_list").asList();

    for (size_t i = 0; i < jointListYarp->size(); i++) {
        humanJointNameList.push_back(jointListYarp->get(i).asString());
    }
    yInfo() << LogPrefix << "human joint names: " << humanJointNameList;

    // Get human hand link name
    if (!(config.check("hand_link") && config.find("hand_link").isString())) {
        yError() << LogPrefix << "Unable to find hand_link in the config file.";
        return false;
    }
    humanHandLinkName = config.find("hand_link").asString();
    yInfo() << LogPrefix << "human hand link name: " << humanHandLinkName;

    // Get human hand finger names
    yarp::os::Bottle* fingerListYarp;
    if (!(config.check("human_finger_list") && config.find("human_finger_list").isList()))
    {
        yError() << LogPrefix << "Unable to find human_finger_list in the config file.";
        return false;
    }
    fingerListYarp = config.find("human_finger_list").asList();

    for (size_t i = 0; i < fingerListYarp->size(); i++)
    {
        humanFingerNames.push_back(fingerListYarp->get(i).asString());
    }
    yInfo() << LogPrefix << "human finger names: " << humanFingerNames;

// TODO: Add a check if there is no glove connected!
    pGlove->Initialize(hostType);
    humanJointState.resize(humanJointNameList.size(), 0);

    return true;
}

bool ManusGlove::ManusGloveImpl::update()
{
    std::lock_guard<std::mutex> lock(mutex);


    pGlove->getHandJointPosition(humanJointState, handSide);

    for (size_t i = 0; i < humanJointState.size(); i++)
    {
        humanJointState[i] = humanJointState[i] * EIGEN_PI / 180;
    }

    return true;
}

bool ManusGlove::ManusGloveImpl::close()
{
    pGlove->ShutDown();
    yInfo() << LogPrefix << "Core closed successfully.";
    return true;
}

// Constructor
ManusGlove::ManusGlove()
    : PeriodicThread(0.01)
    , pImpl{std::make_unique<ManusGloveImpl>()}
{}

// Destructor
ManusGlove::~ManusGlove() = default;

bool ManusGlove::open(yarp::os::Searchable& config)
{

    // Check the device name
    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yInfo() << LogPrefix << "Using default wearable name ManusGlove";
        pImpl->wearableName = DeviceName;
    }
    else {
        pImpl->wearableName = config.find("wearableName").asString();
        yInfo() << LogPrefix << "Using the wearable name " << pImpl->wearableName;
    }

    // Configure the implementation class
    if (!pImpl->open(config)) {
        yError() << LogPrefix << "Cannot configure the implementation class";
        return false;
    }

    // Create Virtual Joint Sensors
    pImpl->jointSensorPrefix = getWearableName() + sensor::IVirtualJointKinSensor::getPrefix();

    for (int i = 0; i < pImpl->humanJointNameList.size(); i++)
    {
        pImpl->manusGloveJointSensorVector.push_back(std::make_shared<ManusGloveImpl::ManusGloveVirtualJointKinSensor>(pImpl.get(), i, pImpl->jointSensorPrefix + pImpl->humanJointNameList[i]));
    }

    // Create Virtual Link Sensors
    pImpl->linkSensorPrefix = getWearableName() + sensor::IVirtualLinkKinSensor::getPrefix();
    pImpl->manusGloveVirtualLinkKinSensor.push_back(
        std::make_shared<ManusGloveImpl::ManusGloveVirtualLinkKinSensor>(
            pImpl.get(), pImpl->linkSensorPrefix + pImpl->humanHandLinkName));

    for (size_t i = 0; i < pImpl->humanFingerNames.size(); i++) {
        pImpl->manusGloveVirtualLinkKinSensor.push_back(
            std::make_shared<ManusGloveImpl::ManusGloveVirtualLinkKinSensor>(
                pImpl.get(),
                pImpl->linkSensorPrefix + pImpl->humanFingerNames[i]
                    + "::fingertip"));
    }

    setPeriod(pImpl->period);

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the period thread.";
        return false;
    }

    return true;
}

// =================================================
// ManusGlove implementation of VirtualLinkKinSensor
// =================================================

class ManusGlove::ManusGloveImpl::ManusGloveVirtualLinkKinSensor
    : public sensor::IVirtualLinkKinSensor
{
public:
    ManusGloveVirtualLinkKinSensor(ManusGlove::ManusGloveImpl* gloveImplPtr,
                                   const sensor::SensorName& name = {},
                                   const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
        : IVirtualLinkKinSensor(name, status)
        , gloveImpl(gloveImplPtr)
        , sensorName(name)
    {
        assert(gloveImpl != nullptr);
    }

    ~ManusGloveVirtualLinkKinSensor() override = default;

    bool getLinkAcceleration(Vector3& linear, Vector3& angular) const override
    {

        // we do not handle linear and angular accelerations in the current
        // implementation

        linear.fill(0.0);

        angular.fill(0.0);

        return true;
    }

    bool getLinkPose(Vector3& position, Quaternion& orientation) const override
    {

        // we do not handle position in the current implementation
        position = {0, 0, 0};

        orientation = {0, 0, 0, 0};

        return true;
    }

    bool getLinkVelocity(Vector3& linear, Vector3& angular) const override
    {

        // we do not handle linear and angular velocity

        angular.fill(0.0);

        linear.fill(0.0);

        return true;
    }

    inline void setStatus(const sensor::SensorStatus& status) { m_status = status; }

private:
    ManusGlove::ManusGloveImpl* gloveImpl{nullptr};
    std::string sensorName;
};

// ==================================================
// ManusGlove implementation of VirtualJointKinSensor
// ==================================================

class ManusGlove::ManusGloveImpl::ManusGloveVirtualJointKinSensor
    : public sensor::IVirtualJointKinSensor
{
public:
    ManusGloveVirtualJointKinSensor(ManusGlove::ManusGloveImpl* gloveImplPtr,
                                    const int humanJointIndex,
                                    const sensor::SensorName& name = {},
                                    const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
        : IVirtualJointKinSensor(name, status)
        , gloveImpl(gloveImplPtr)
        , humanJointIndex(humanJointIndex)
    {
        assert(gloveImpl != nullptr);
    }

    ~ManusGloveVirtualJointKinSensor() override = default;

    bool getJointPosition(double& position) const override
    {

        std::lock_guard<std::mutex> lock(gloveImpl->mutex);

        position = gloveImpl->humanJointState[humanJointIndex];

        return true;
    }

    bool getJointVelocity(double& velocity) const override
    {

        std::lock_guard<std::mutex> lock(gloveImpl->mutex);

        velocity = 0.0;

        return true;
    }

    bool getJointAcceleration(double& acceleration) const override
    {

        std::lock_guard<std::mutex> lock(gloveImpl->mutex);

        acceleration = 0.0;

        return true;
    }

    inline void setStatus(const sensor::SensorStatus& status) { m_status = status; }

private:
    ManusGlove::ManusGloveImpl* gloveImpl{nullptr};
    const int humanJointIndex;
};

// ==================================================

void ManusGlove::run()
{
    // Get timestamp
    pImpl->timeStamp.time = yarp::os::Time::now();

    // to implement
    pImpl->update();
}

bool ManusGlove::close()
{
    this->askToStop();
    while(this->isRunning())
    {
        yDebug()<<"Waiting for the thread to stop ";
        yarp::os::Time::delay(1);

    }    

    yDebug()<<"Thread stopped ";

    if (!pImpl->close()) {
        yError() << LogPrefix << "Cannot close correctly the manus glove implementation.";
        return false;
    }

    yDebug()<<"end of closing ";

    return true;
}

// =========================
// IPreciselyTimed interface
// =========================

yarp::os::Stamp ManusGlove::getLastInputStamp()
{
    // Stamp count should be always zero
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return yarp::os::Stamp(0, pImpl->timeStamp.time);
}

// ---------------------------------------
// Implement Sensors and Actuators Methods
// ---------------------------------------

wearable::WearableName ManusGlove::getWearableName() const
{
    return pImpl->wearableName + wearable::Separator;
}

wearable::WearStatus ManusGlove::getStatus() const
{
    wearable::WearStatus status = wearable::WearStatus::Ok;

    for (const auto& s : getAllSensors()) {
        if (s->getSensorStatus() != sensor::SensorStatus::Ok) {
            status = wearable::WearStatus::Error;
            // TO CHECK
            break;
        }
    }
    // Default return status is Ok
    return status;
}

wearable::TimeStamp ManusGlove::getTimeStamp() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return {pImpl->timeStamp.time, 0};
}

wearable::SensorPtr<const wearable::sensor::ISensor>
ManusGlove::getSensor(const wearable::sensor::SensorName name) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getAllSensors();
    for (const auto& s : sensors) {
        if (s->getSensorName() == name) {
            return s;
        }
    }
    yWarning() << LogPrefix << "User specified name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
ManusGlove::getSensors(const wearable::sensor::SensorType aType) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> outVec;
    switch (aType)
    {
        case sensor::SensorType::VirtualLinkKinSensor: {
            outVec.reserve(pImpl->manusGloveVirtualLinkKinSensor.size());
            for (const auto& manusgloveLinkSensor : pImpl->manusGloveVirtualLinkKinSensor)
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(manusgloveLinkSensor));
            break;
        }
        case sensor::SensorType::VirtualJointKinSensor: {
            outVec.reserve(pImpl->manusGloveJointSensorVector.size());
            for (const auto& element : pImpl->manusGloveJointSensorVector)
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(element));
            break;
        }
        default: {
            return {};
        }
    }

    return outVec;
}

wearable::ElementPtr<const wearable::actuator::IActuator>
ManusGlove::getActuator(const wearable::actuator::ActuatorName name) const
{
    return nullptr;
}

wearable::VectorOfElementPtr<const wearable::actuator::IActuator>
ManusGlove::getActuators(const wearable::actuator::ActuatorType aType) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
ManusGlove::getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
ManusGlove::getVirtualJointKinSensor(const wearable::sensor::SensorName name) const
{
    for (SensorPtr<ManusGloveImpl::ManusGloveVirtualJointKinSensor> &element : pImpl->manusGloveJointSensorVector)
    {
        if (name == element->getSensorName())
        {
            return element;
        }
    }

    return nullptr;
}

wearable::ElementPtr<const wearable::actuator::IHaptic>
ManusGlove::getHapticActuator(const actuator::ActuatorName name) const
{
    return nullptr;
}
// =========================
// Defintion of Utilities
// =========================
