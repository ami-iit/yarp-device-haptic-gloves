// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <limits>

#include <SenseGloveHelper.hpp>

// SenseGlove
#include <SenseGlove/Core/DeviceList.hpp>
#include <SenseGlove/Core/HandLayer.hpp>
#include <SenseGlove/Core/HandPose.hpp>
#include <SenseGlove/Core/Vect3D.hpp>
#include <SenseGlove/Core/Quat.hpp>

// Code inspired from https://github.com/Adjuvo/SenseGlove-API/blob/fc5b021386eb1de50ee9c790630127d6eb83e6de/examples/sgcore-client.cpp

using namespace senseGlove;

enum class FingerType : int32_t
{
    Thumb = 0,
    Index = 1,
    Middle = 2,
    Ring = 3,
    Pinky = 4
};

enum class FingerParsingFailure
{
    None,
    HandNotFound,
    FingerNotFound,
    JointNotFound,
};

std::pair<FingerType, FingerParsingFailure> getFingerType(bool isRightHand, const std::string& fingerName)
{
    std::string input = fingerName;
    FingerParsingFailure failure = FingerParsingFailure::None;
    FingerType fingerType = FingerType::Thumb;

    //check that the first two letters are "r_" or "l_" depending on the hand and remove them
    if (isRightHand && input.substr(0,2) == "r_")
    {
        input = input.substr(2);
    }
    else if (!isRightHand && input.substr(0, 2) == "l_")
    {
        input = input.substr(2);
    }
    else
    {
        failure = FingerParsingFailure::HandNotFound;
        return { fingerType, failure };
    }

    bool found = false;
    if (input.find("thumb") == 0)
    {
        fingerType = FingerType::Thumb;
        found = true;
    }
    if (input.find("index") == 0)
    {
        fingerType = FingerType::Index;
        found = true;
    }
    if (input.find("middle") == 0)
    {
        fingerType = FingerType::Middle;
        found = true;
    }
    if (input.find("ring") == 0)
    {
        fingerType = FingerType::Ring;
        found = true;
    }
    if (input.find("pinky") == 0 || input.find("little_finger") == 0)
    {
        fingerType = FingerType::Pinky;
        found = true;
    }

    if (!found)
    {
        failure = FingerParsingFailure::FingerNotFound;
    }

    return {fingerType, failure};
}

SGCore::EHapticLocation getHapticLocation(FingerType fingerType)
{
    switch (fingerType)
    {
    case FingerType::Thumb:
        return SGCore::EHapticLocation::ThumbTip;
    case FingerType::Index:
        return SGCore::EHapticLocation::IndexTip;
    case FingerType::Middle:
        return SGCore::EHapticLocation::MiddleTip;
    case FingerType::Ring:
        return SGCore::EHapticLocation::RingTip;
    case FingerType::Pinky:
        return SGCore::EHapticLocation::PinkyTip;
    }
    return SGCore::EHapticLocation::ThumbTip;
}

std::pair<HandJointIndex, FingerParsingFailure> getJointIndex(bool isRightHand, const std::string& jointName)
{
    HandJointIndex output;
    bool found = false;
    FingerParsingFailure failure = FingerParsingFailure::None;

    auto parsedFinger = getFingerType(isRightHand, jointName);
    if (parsedFinger.second != FingerParsingFailure::None)
    {
        return {output, parsedFinger.second };
    }
    std::string input = jointName;
    input = input.substr(2); // remove the hand prefix

    output.fingerIndex = static_cast<int32_t>(parsedFinger.first);

    if (jointName.find("_oppose") != std::string::npos || jointName.find("_abduction") != std::string::npos)
    {
        // Oppose (for the thumb) and abduction joints are on the first joint in the z axis
        output.jointIndex = 0;
        output.angleIndex = 2;
        found = true;
    }
    // All other joints are on the y axis
    if (jointName.find("_proximal") != std::string::npos)
    {
        output.jointIndex = 0;
        output.angleIndex = 1;
        found = true;
    }
    if (jointName.find("_middle") != std::string::npos)
    {
        output.jointIndex = 1;
        output.angleIndex = 1;
        found = true;
    }
    if (jointName.find("_distal") != std::string::npos)
    {
        output.jointIndex = 2;
        output.angleIndex = 1;
        found = true;
    }

    if (!found)
    {
        failure = FingerParsingFailure::JointNotFound;
    }

    return {output, failure};
}


SenseGloveHelper::SenseGloveHelper()
{
    yInfo() << LogPrefix << "SenseGloveHelper()";

    m_isReady = false;
}

bool SenseGloveHelper::configure(const yarp::os::Searchable& config)
{
    yInfo() << LogPrefix << "configure:: ";

    if (!(config.check("rightHand") && config.find("rightHand").isBool())) {
        yInfo() << LogPrefix << "Using default hand Sense Glove: Right hand";
        m_isRightHand = true;
    }
    else {
        m_isRightHand = config.find("rightHand").asBool();
        yInfo() << LogPrefix << "Using the right hand: " << m_isRightHand
                << "(if false, using left hand)";
    }
    LogPrefix = LogPrefix + (m_isRightHand ? "Right hand: " : "Left hand: ");

    // get human hand link name
    if (!(config.check("hand_link") && config.find("hand_link").isString())) {
        yError() << LogPrefix << "Unable to find hand_link in the config file.";
        return false;
    }
    m_humanHandLinkName = config.find("hand_link").asString();
    yInfo() << LogPrefix << "human hand link name: " << m_humanHandLinkName;

    // get human hand joint names
    yarp::os::Bottle* jointListYarp;
    if (!(config.check("human_joint_list") && config.find("human_joint_list").isList())) {
        yError() << LogPrefix << "Unable to find human_joint_list in the config file.";
        return false;
    }
    jointListYarp = config.find("human_joint_list").asList();

    for (size_t i = 0; i < jointListYarp->size(); i++) {
        std::string jointName = jointListYarp->get(i).asString();
        auto parsed = getJointIndex(m_isRightHand, jointName);
        if (parsed.second == FingerParsingFailure::HandNotFound)
        {
            yError() << LogPrefix << "Hand name not found or not correct in the joint name: " << jointName;
            return false;
        }
        if (parsed.second == FingerParsingFailure::FingerNotFound)
        {
            yError() << LogPrefix << "Unrecognized finger name in the joint name: " << jointName;
            return false;
        }
        if (parsed.second == FingerParsingFailure::JointNotFound)
        {
            yError() << LogPrefix << "Unrecognized joint name in the joint name: " << jointName;
            return false;
        }
        m_humanJointIndexMap[jointName] = parsed.first;
        m_humanJointNameList.push_back(jointName);
        yInfo() << LogPrefix << "Parsed joint name: " << jointName << "finger index:" << parsed.first.fingerIndex
                                                                   << "joint index:" << parsed.first.jointIndex
                                                                   << "angle index:" << parsed.first.angleIndex;
    }
    yInfo() << LogPrefix << "human joint names: " << m_humanJointNameList;

    // get human hand finger names
    yarp::os::Bottle* fingerListYarp;
    if (!(config.check("human_finger_list") && config.find("human_finger_list").isList())) {
        yError() << LogPrefix << "Unable to find human_finger_list in the config file.";
        return false;
    }
    fingerListYarp = config.find("human_finger_list").asList();

    for (size_t i = 0; i < fingerListYarp->size(); i++) {
        std::string fingerName = fingerListYarp->get(i).asString();

        auto parsed = getFingerType(m_isRightHand, fingerName);
        if (parsed.second == FingerParsingFailure::HandNotFound)
        {
            yError() << LogPrefix << "Hand name not found or not correct in the finger name: " << fingerName;
            return false;
        }
        if (parsed.second == FingerParsingFailure::FingerNotFound)
        {
            yError() << LogPrefix << "Unrecognized finger name in: " << fingerName;
            return false;
        }

        m_humanToGloveMap[fingerName] = {static_cast<int32_t>(parsed.first), getHapticLocation(parsed.first)};

        m_handOrientationEulerAngles.push_back(Eigen::Matrix3d::Zero()); //Each finger has three three-dimensional joints
        m_handPose.push_back(Eigen::MatrixXd::Zero(4, PoseSize)); //Each finger has four links
        m_humanFingerNameList.push_back(fingerName);
    }
    yInfo() << LogPrefix << "human finger names: " << m_humanFingerNameList;

    if (!setupGlove()) {
        yError() << LogPrefix << "Unable to set up the sense glove.";
        return false;
    }

    return true;
}

bool SenseGloveHelper::setupGlove()
{
    yInfo() << LogPrefix << "setupGlove()";

    if (!SGCore::DeviceList::SenseComRunning()) // Returns true if SenseComm is
                                                 // running.
    {
        yError() << LogPrefix << "SenseComm is not running. Please run SenseComm, then try again.";
        return false;
    }

    if (!SGCore::HandLayer::DeviceConnected(m_isRightHand)) {
        yError() << LogPrefix << "No SenseGlove connected.";
        return false;
    }

    yInfo() << LogPrefix << "Glove model:" << SGCore::SGDevice::ToString(SGCore::HandLayer::GetDeviceType(m_isRightHand));

    return true;
}

bool SenseGloveHelper::setFingersForceReference(const std::vector<double>& desiredValue)
{
    if (desiredValue.size() != m_humanFingerNameList.size()) {
        yError() << LogPrefix << "setFingersForceReference: "
                 << "Expected a force reference of dimension " << m_humanFingerNameList.size()
                 << "obtained" << desiredValue.size();
        return false;
    }

    for (size_t i = 0; i < m_humanFingerNameList.size(); i++) {
        float value = std::clamp(static_cast<float>(desiredValue[i]), 0.0f, 1.0f);
        int32_t finger = m_humanToGloveMap[m_humanFingerNameList[i]].first;
        SGCore::HandLayer::QueueCommand_ForceFeedbackLevel(m_isRightHand, finger, value, false);
    }
    SGCore::HandLayer::SendHaptics(m_isRightHand);

    return true;
}

bool SenseGloveHelper::setBuzzMotorsReference(const std::vector<double>& desiredValue)
{
    if (desiredValue.size() != m_humanFingerNameList.size()) {
        yError() << LogPrefix << "setBuzzMotorsReference: "
                 << "Expected a buzz motor reference of dimension " << m_humanFingerNameList.size()
                 << "obtained" << desiredValue.size();
        return false;
    }


    for (size_t i = 0; i < m_humanFingerNameList.size(); i++) {
        float value = std::clamp(static_cast<float>(desiredValue[i]), 0.0f, 1.0f);
        SGCore::EHapticLocation finger = m_humanToGloveMap[m_humanFingerNameList[i]].second;
        SGCore::HandLayer::QueueCommand_VibroLevel(m_isRightHand, finger, value, false);
    }
    SGCore::HandLayer::SendHaptics(m_isRightHand);

    return true;
}

bool senseGlove::SenseGloveHelper::setPalmForceFeedback(double desiredValue)
{
    if (!SGCore::HandLayer::SupportsWristSqueeze(m_isRightHand))
    {
        yWarningOnce() << LogPrefix << "The glove does not support wrist squeeze.";
        return true;
    }

    float value = std::clamp(static_cast<float>(desiredValue), 0.0f, 1.0f);
    SGCore::HandLayer::QueueCommand_WristSqueeze(m_isRightHand, value, true);
    return true;
}

bool SenseGloveHelper::setPalmBuzzFeedback(double desiredValue)
{
    float value = std::clamp(static_cast<float>(desiredValue), 0.0f, 1.0f);
    SGCore::HandLayer::QueueCommand_VibroLevel(m_isRightHand, SGCore::EHapticLocation::PalmIndexSide, value, false);
    SGCore::HandLayer::QueueCommand_VibroLevel(m_isRightHand, SGCore::EHapticLocation::PalmPinkySide, value, false);
    SGCore::HandLayer::SendHaptics(m_isRightHand);
    return true;
}

void SenseGloveHelper::updateHandLinksPose()
{
    SGCore::HandPose handPose;
    if (!SGCore::HandLayer::GetHandPose(m_isRightHand, handPose)) {
        yWarning() << LogPrefix << "Failed to get hand pose for reading link positions.";
        return;
    }

    const auto& jointPositions = handPose.GetJointPositions();
    const auto& jointRotations = handPose.GetJointRotations();

    // Info in https://senseglove.gitlab.io/SenseGloveDocs/native/handpose-core.html
    assert(jointPositions.size() == 5);
    assert(jointRotations.size() == 5);

    for (size_t i = 0; i < m_humanFingerNameList.size(); i++) {
        auto fingerIndex = m_humanToGloveMap[m_humanFingerNameList[i]].first;
        const auto& linkPositions = jointPositions[fingerIndex];
        const auto& linkRotations = jointRotations[fingerIndex];
        auto& fingerMatrix = m_handPose[i];

        assert(linkPositions.size() == 4);
        assert(linkRotations.size() == 4);
        for (size_t j = 0; j < linkPositions.size(); j++) {
            fingerMatrix(j, 0) = linkPositions[j].GetX();
            fingerMatrix(j, 1) = linkPositions[j].GetY();
            fingerMatrix(j, 2) = linkPositions[j].GetZ();

            // wrt to the origin frame
            fingerMatrix(j, 3) = linkRotations[j].GetW();
            fingerMatrix(j, 4) = linkRotations[j].GetX();
            fingerMatrix(j, 5) = linkRotations[j].GetY();
            fingerMatrix(j, 6) = linkRotations[j].GetZ();
        }
    }
}

bool SenseGloveHelper::getHandJointsAngles()
{
    SGCore::HandPose handPose;
    if (!SGCore::HandLayer::GetHandPose(m_isRightHand, handPose)) {
        yWarning() << LogPrefix << "Failed to get hand pose for reading joint angles.";
        return true; // to avoid stopping the device
    }

    const auto& handAngles = handPose.GetHandAngles();

    // Info in https://senseglove.gitlab.io/SenseGloveDocs/native/handpose-core.html
    assert(handAngles.size() == 5);

    for (size_t i = 0; i < m_humanFingerNameList.size(); i++) {
        auto fingerIndex = m_humanToGloveMap[m_humanFingerNameList[i]].first;
        const auto& fingerAngles = handAngles[fingerIndex];
        auto& fingerMatrix = m_handOrientationEulerAngles[i];

        assert(fingerAngles.size() == 3); // 3 joints for each finger (proximal, intermediate, distal)

        for (int j = 0; j < fingerAngles.size(); j++) {
            // Euler angle for each finger joint
            fingerMatrix(j, 0) = fingerAngles[j].GetX();
            fingerMatrix(j, 1) = fingerAngles[j].GetY();
            fingerMatrix(j, 2) = fingerAngles[j].GetZ();
        }
    }
    return true;
}

bool SenseGloveHelper::getHandJointsAngles(std::vector<double>& jointAngleList)
{
    getHandJointsAngles();

    jointAngleList.resize(m_humanJointNameList.size(), 0.0);

    for (size_t i = 0; i < m_humanJointNameList.size(); i++) {
        const auto& jointIndex = m_humanJointIndexMap[m_humanJointNameList[i]];
        jointAngleList[i] = m_handOrientationEulerAngles[jointIndex.fingerIndex](jointIndex.jointIndex, jointIndex.angleIndex);
    }

    return true;
}

bool SenseGloveHelper::getHandFingertipLinksPose(std::vector<std::vector<double>>& fingertipPoses)
{
    fingertipPoses.resize(m_humanFingerNameList.size());

    updateHandLinksPose();

    for (size_t i = 0; i < m_humanFingerNameList.size(); i++) {
        auto fingerIndex = m_humanToGloveMap[m_humanFingerNameList[i]].first;
        const auto& fingerMatrix = m_handPose[i];
        fingertipPoses[i].resize(PoseSize);
        Eigen::Map<Eigen::VectorXd> output(fingertipPoses[i].data(), PoseSize);
        output = fingerMatrix(Eigen::last, Eigen::all); //get the pose of the last row, i.e. the last finger link
    }

    return true;
}

bool SenseGloveHelper::getPalmLinkPose(std::vector<double>& palmLinkPose)
{
    std::shared_ptr<SGCore::HapticGlove> glove;
    if (!SGCore::HapticGlove::GetGlove(m_isRightHand, glove))
    {
        yWarning() << LogPrefix << "getPalmLinkPose: Cannot get glove object";
        return true; // to avoid crashing
    }
    SGCore::Kinematics::Quat imu;

    if (palmLinkPose.size() != 7) {
        palmLinkPose.resize(7, 0.0);
    }

    if (!glove->GetImuRotation(imu)) {
        yWarning() << LogPrefix << "Cannot get glove IMU value";
        return true; // to avoid crashing
    }
    // position
    palmLinkPose[0] = 0.0;
    palmLinkPose[1] = 0.0;
    palmLinkPose[2] = 0.0;
    // orientation: IMU
    palmLinkPose[3] = imu.GetW();
    palmLinkPose[4] = imu.GetX();
    palmLinkPose[5] = imu.GetY();
    palmLinkPose[6] = imu.GetZ();

    Eigen::Map<Eigen::Vector4d> output(palmLinkPose.data() + 3);
    output.normalize();

    return true;
}

bool SenseGloveHelper::getHumanJointNameList(std::vector<std::string>& jointList) const
{
    if (m_humanJointNameList.size() == 0) {
        yError() << LogPrefix << "No joints have been specified.";
        return false;
    }

    jointList = m_humanJointNameList;

    return true;
}

bool SenseGloveHelper::getHumanHandLinkName(std::string& handLinkName) const
{
    handLinkName = m_humanHandLinkName;
    return true;
}

bool SenseGloveHelper::getHumanFingerNameList(std::vector<std::string>& fingerList) const
{

    if (m_humanFingerNameList.size() == 0) {
        yError() << LogPrefix << "No fingers have been specified.";
        return false;
    }

    fingerList = m_humanFingerNameList;

    return true;
}

SenseGloveHelper::~SenseGloveHelper() {}

void SenseGloveHelper::close()
{
    SGCore::HandLayer::StopAllHaptics(m_isRightHand);
}
