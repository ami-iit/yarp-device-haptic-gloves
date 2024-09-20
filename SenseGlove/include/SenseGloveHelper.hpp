// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#ifndef SENSE_GLOVE_HELPER_HPP
#define SENSE_GLOVE_HELPER_HPP

#include <Eigen/Dense>

// std
#include <array>
#include <iostream>
#include <memory>
#include <stdint.h>
#include <unordered_map>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <SenseGlove/Core/HapticGlove.hpp>

/**
 * GloveControlHelper is an helper class for controlling the glove.
 */
namespace senseGlove {
    struct HandJointIndex;
    class SenseGloveHelper;
} // namespace senseGlove


struct senseGlove::HandJointIndex
{
    size_t fingerIndex{0};
    size_t jointIndex{0};
    size_t angleIndex{0};
};

class senseGlove::SenseGloveHelper
{
    std::string LogPrefix = "senseGlove::SenseGloveHelper::";

    const size_t PoseSize = 7; // [ position <x, y, z>, quaternion <w, x, y, z> ]

    /// true if the glove is ready to use, i.e., communication working
    bool m_isReady;

    /// true if the glove is the right hand
    bool m_isRightHand;

    /// sensory data of the glove in radians
    std::vector<float> m_sensorData;

    /// sensory data of the hand link poses;  from thumb to pinky, proximal to
    /// distal, pos [x y z] Quat [x y z w]
    std::vector<Eigen::MatrixXd> m_handPose;

    /// sensory data of the human hand joints angles; From thumb to pinky,
    /// proximal to distal [rad] [Pronation/Supination (x), Flexion/Extension (y),
    /// Abduction/Adduction (z)]
    std::vector<Eigen::Matrix3d> m_handOrientationEulerAngles;

    /// vector of the human joint names
    std::vector<std::string> m_humanJointNameList;

    /// vector of the human finger names
    std::vector<std::string> m_humanFingerNameList;

    /// map from the specified human hand joint names to the glove joint names
    std::unordered_map<std::string, std::pair<int32_t, SGCore::EHapticLocation>> m_humanToGloveMap;

    /// map from the specified human hand joint names to the glove joint index
    std::unordered_map<std::string, HandJointIndex> m_humanJointIndexMap;

    /// the name of the human hand link name
    std::string m_humanHandLinkName;

    /**
     * Setup the communication with the glove
     * @return true / false in case of success / failure
     */
    bool setupGlove();

    /**
     * Get the human hand joint angles from the sense glove data structure
     * @return true / false in case of success / failure
     */
    bool getHandJointsAngles();

public:
    /**
      Constructor
    **/
    SenseGloveHelper();

    /**
      Destructor
    **/
    ~SenseGloveHelper();

    /**
     * Configure the class
     * @param config configuration options
     * @param rightHand if true, the right hand glove will be configured,
     * otherwise left.
     * @return true / false in case of success / failure
     */
    bool configure(const yarp::os::Searchable& config);

    /**
     * Set the desired Force Feedback for all the fingers
     * @param desiredValue desired force feedback of all the fingers
     * @return true / false in case of success / failure
     */
    bool setFingersForceReference(const std::vector<double>& desiredValue);

    /**
     * Set the desired vibro-tactile feedback for all the fingers
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success / failure
     */
    bool setBuzzMotorsReference(const std::vector<double>& desiredValue);

    /**
     * Set the desired force feedback for the palm
     * @param desiredValue desired force value of the palm
     * @return true / false in case of success / failure
     */
    bool setPalmForceFeedback(double desiredValue);

    /**
     * Set the desired vibro-tactile feedback for the palm
     * @param desiredValue desired vibro-tactile value of the palm
     * @return true / false in case of success / failure
     */
    bool setPalmBuzzFeedback(double desiredValue);

    /**
     * Get the measured hand link poses values
     * @param measuredValue measured joint values
     * @return true / false in case of success / failure
     */
    void updateHandLinksPose();

    /**
     * Get the human hand joint angles
     * @param jointAngleList std vector of doubles of human hand joint angles
     * @return true / false in case of success / failure
     */
    bool getHandJointsAngles(std::vector<double>& jointAngleList);

    /**
     * Get the fingertip poses based on glove sensory data
     * @param fingertipPoses Eigen matrix of glove poses [(pos: x, y, z) ,
     * (rotation: w, x, y, z )]
     * @return true / false in case of success / failure
     */
    bool getHandFingertipLinksPose(std::vector<std::vector<double>>& fingertipPoses);

    /**
     * Get the glove IMU data
     * @param palmLinkPose human palm pose based on glove IMU data with the order
     * pos(x y z), quat(w x y z)
     * @return true / false in case of success / failure
     */
    bool getPalmLinkPose(std::vector<double>& palmLinkPose);

    /**
     * Get the human joint list
     * @param jointList the human joint list
     * @return true / false in case of success / failure
     */
    bool getHumanJointNameList(std::vector<std::string>& jointList) const;

    /**
     * Get the human hand link name
     * @param handLinkName the human hand link name
     * @return true / false in case of success / failure
     */
    bool getHumanHandLinkName(std::string& handLinkName) const;

    /**
     * Get the human finger list
     * @param fingerList the human joint list
     * @return true / false in case of success / failure
     */
    bool getHumanFingerNameList(std::vector<std::string>& fingerList) const;

    /**
     * close the device
     * @return true / false in case of connected / disconnected
     */
    void close();
};

#endif
