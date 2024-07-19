/**
 * @file WeArtGlove.cpp
 * @authors Ehsan Ranjbari <ehsan.ranjbari@iit.it>
 * @copyright 2022 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2022
 */

#include <WeArtGlove.h>
#include <WeArtGloveLib/WeArtGloveLib.h>

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

// iDynTree headers
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/EigenHelpers.h>

// blf headers
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/DistanceTask.h>
#include <BipedalLocomotion/IK/GravityTask.h>

const std::string DeviceName = "WeArtGlove";
const std::string LogPrefix = DeviceName + wearable::Separator;

using namespace wearable;
using namespace wearable::devices;

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::IK;
using namespace BipedalLocomotion::Conversions;
using namespace std::chrono_literals;

// SDK Pointers
static std::unique_ptr<WeArtClient> weArtClient{nullptr};
static std::mutex weArtClientMutex;

class WeArtGlove::WeArtGloveImpl
{
public:
    mutable std::mutex mutex;

    WearableName wearableName;

    TimeStamp timeStamp;

    HandSide handSide;
    std::string handSidePrefix;
    std::string PalmSidePrefix;

    double period;
    const std::chrono::nanoseconds dT = 10ms;

    // Vector of the human joint names
    std::vector<std::string> humanJointNameList;

    // Vector of the URDF considered joint names
    std::vector<std::string> consideredJointNameList;
    std::vector<std::string> fingerTipFrameNameList;

    // Vector of the human finger names
    std::vector<std::string> humanFingerNames;

    // The name of the human hand link name
    std::string humanHandLinkName;

    // The type of the tracking algorithm
    std::string trackingAlgorithmType;

    // Link Sensor
    std::string linkSensorPrefix;
    class WeArtGloveVirtualLinkKinSensor;
    std::vector<SensorPtr<WeArtGloveVirtualLinkKinSensor>> weartGloveVirtualLinkKinSensor;

    // Joint Sensor
    std::string jointSensorPrefix;
    class WeArtGloveVirtualJointKinSensor;
    std::vector<SensorPtr<WeArtGloveVirtualJointKinSensor>> weartGloveJointSensorVector;

    // WeArtGlove Force actuator TODO: We need to add IForce.h later. For now we use IHaptic.h
    std::string forceActuatorPrefix;
    class WeArtGloveForceActuator;
    std::vector<SensorPtr<WeArtGloveForceActuator>> weartGloveForceActuatorVector;
    bool useForceFeedback = true;

    // Texture actuator
    std::string textureActuatorPrefix;
    class WeArtGloveTextureActuator;
    std::vector<SensorPtr<WeArtGloveTextureActuator>> weartGloveTextureActuatorVector;
    bool useTextureFeedback = true;
    TextureType textureId = TextureType::DoubleSidedTape;

    // Temperature actuator
    std::string temperatureActuatorPrefix;
    class WeArtGloveTemperatureActuator;
    std::vector<SensorPtr<WeArtGloveTemperatureActuator>> weartGloveTemperatureActuatorVector;
    bool useTemperatureFeedBack = true;

    // Matrix closure to joint state
    Eigen::MatrixXd closureToHumanJoint;
    Eigen::VectorXd weartClosureSmooth = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd weartClosure = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd humanJointState = Eigen::VectorXd::Zero(20);
    Eigen::VectorXd MIN_JOINT_LIMIT = Eigen::VectorXd::Zero(20);
    Eigen::VectorXd MAX_JOINT_LIMIT = Eigen::VectorXd::Zero(20);
    Eigen::VectorXd upperLimits;
    Eigen::VectorXd lowerLimits; 

    Eigen::MatrixXd weartRawDataAcc = Eigen::MatrixXd::Zero(4,3);
    Eigen::MatrixXd weartRawDataGyro = Eigen::MatrixXd::Zero(4,3);
    Eigen::VectorXd weartRawDataTof = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d thimbleDistanceIK = Eigen::Vector3d::Zero(3);
    Eigen::MatrixXd weartRawDataAccRotated = Eigen::MatrixXd::Zero(3,4);
    Eigen::MatrixXd weartRawDataGyroRotated = Eigen::MatrixXd::Zero(3,4);
    Eigen::MatrixXd fingerSensorToBodyRotation = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd palmSensorToBodyRotation = Eigen::MatrixXd::Zero(3,3);

    iDynTree::Transform linkTransform;
    iDynTree::Vector3 gravityVector;

    double alpha = 1;
    const int N_THIMBLES = 3;
    double convergenceRate = 1;
    double regularizationRate = 0.5;
    double fingerLength = 150;

    // Load the URDF model
    std::string modelFile;
    iDynTree::ModelLoader mdlLoader;

    iDynTree::KinDynComputations kinDynComp;

    //Prepare the integrator
    std::shared_ptr<ForwardEuler<FloatingBaseSystemKinematics>> integrator;
    std::shared_ptr<FloatingBaseSystemKinematics> dynamics;
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    std::shared_ptr<BipedalLocomotion::IK::DistanceTask> thumbDistanceTaskptr;
    std::shared_ptr<BipedalLocomotion::IK::DistanceTask> indexDistanceTaskptr;
    std::shared_ptr<BipedalLocomotion::IK::DistanceTask> middleDistanceTaskptr;
    std::shared_ptr<BipedalLocomotion::IK::GravityTask> palmGravityTaskptr;
    std::shared_ptr<BipedalLocomotion::IK::GravityTask> thumbGravityTaskptr;
    std::shared_ptr<BipedalLocomotion::IK::GravityTask> indexGravityTaskptr;
    std::shared_ptr<BipedalLocomotion::IK::GravityTask> middleGravityTaskptr;
    std::shared_ptr<BipedalLocomotion::IK::IntegrationBasedIKProblem> ikProblem;

    WeArtTrackingRawData::Sample weartRawDataSample;
    std::shared_ptr<WeArtTrackingCalibration> weArtTrackingCalibration;
    std::shared_ptr<MiddlewareStatusListener> mwListener;
    MiddlewareStatusUpdate mwStatus;

    Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::VectorXd jointVelocity;

    Eigen::VectorXd jointVelocities;
    Eigen::Matrix4d basePose;
    Eigen::VectorXd jointPositions;
    Eigen::Vector3d gravity;

    // Thimble Effect Class Definition
    class ThimbleEffect : public TouchEffect
    {

    public:
        WeArtTemperature effTemperature;
        WeArtForce effForce;
        WeArtTexture effTexture;

        ThimbleEffect(WeArtTemperature temp, WeArtForce force, WeArtTexture texture) : TouchEffect(temp, force, texture)
        {
            effTemperature = temp;
            effForce = force;
            effTexture = texture;
        };
    };

    // Structure for Thimble implementation
    struct ThimbleInformation
    {
        std::unique_ptr<WeArtThimbleTrackingObject> tracking;
        std::unique_ptr<WeArtHapticObject> hapticObject;
        std::shared_ptr<ThimbleEffect> thimbleEffect;
        std::shared_ptr<WeArtTrackingRawData> rawSensorData;
    };

    // Map for Thimbles
    std::unordered_map<std::string, ThimbleInformation> thimbleInformationMap;

    // Constructor
    WeArtGloveImpl();

    bool update();

    bool open(yarp::os::Searchable& config);

    bool close();

    // Utilities Methods
    int thimbleNameToNumber(const std::string &name);

};

WeArtGlove::WeArtGloveImpl::WeArtGloveImpl() {}

bool WeArtGlove::WeArtGloveImpl::open(yarp::os::Searchable& config)
{
    // Parameters from xml file
    period = config.check("period", yarp::os::Value(0.01)).asFloat64();

    // Assigning the correct handside
    if (!config.check("is_right_hand"))
    {
        yError()<<"Cannot find the is_right_hand parameter";
        return false;
    }

    bool isRightHand = config.find("is_right_hand").asBool();
    handSide = isRightHand ? HandSide::Right : HandSide::Left;
    handSidePrefix = isRightHand ? "r_" : "l_";
    PalmSidePrefix = isRightHand ? "Right" : "Left";

/**
 * The matrices below represents the rotation from the finger/palm sensor's frame to the base frame.
 * The frames are visualised the model with iDyntree.
 */
    if (PalmSidePrefix == "Right")
    {
        //Right Hand
        fingerSensorToBodyRotation << 0, 1, 0,
                                    0, 0, 1,
                                    1, 0, 0;

        palmSensorToBodyRotation << -1, 0, 0,
                                    0, -1, 0,
                                    0, 0, 1;
    }
    else
    {
        //Left Hand
        fingerSensorToBodyRotation << 0, -1, 0,
                                    0, 0, -1,
                                    1, 0, 0;

        palmSensorToBodyRotation << -1, 0, 0,
                                    0, 1, 0,
                                    0, 0, -1;
    }

    // Select the tracking algorithm type
    if (!(config.check("tracking_algorithm") && config.find("tracking_algorithm").isString())) {
        yError() << LogPrefix << "Unable to find tracking_algorithm in the config file.";
        return false;
    }
    trackingAlgorithmType = config.find("tracking_algorithm").asString();
    yInfo() << LogPrefix << "Tracking Algorithm Type: " << trackingAlgorithmType;

    // Handling the list of thimbles of the WeArt device
    if (!config.check("weart_thimble_list"))
    {
        yError()<<"Cannot find the weart_thimble_list parameter";
        return false;
    }
    std::vector<std::string> thimbles = {"thumb", "index", "middle", "palm"};
    {
        std::lock_guard<std::mutex> lock(weArtClientMutex);
        if (!weArtClient)
        {
            weArtClient = std::make_unique<WeArtClient>("127.0.0.1", WeArtConstants::DEFAULT_TCP_PORT);
        }
    }

    // Add Middleware status listener
    mwListener = std::make_shared<MiddlewareStatusListener>();
    weArtClient->AddMessageListener(mwListener.get());

    // Create calibration tracker and add to client //TODO
    weArtTrackingCalibration = std::make_shared<WeArtTrackingCalibration>();
    weArtClient->AddMessageListener(weArtTrackingCalibration.get());
    if (weArtTrackingCalibration->getStatus() == CalibrationStatus::Running)
    {
        if (weArtTrackingCalibration->getResult())
        {
            yInfo() << "Calibration finished!";
        }
        else
        {
            yWarning() << "Calibration Failed!";
        }
    }

    for (std::string& thimble : thimbles)
    {
        thimbleInformationMap.emplace(thimble, ThimbleInformation());

        ActuationPoint actuationPoint;

        if (thimble ==  "index")
        {
            actuationPoint = ActuationPoint::Index;
        }
        else if(thimble ==  "middle")
        {
            actuationPoint = ActuationPoint::Middle;
        }
        else if (thimble ==  "thumb")
        {
            actuationPoint = ActuationPoint::Thumb;
        }
        else if (thimble ==  "palm")
        {
            actuationPoint = ActuationPoint::Palm;
        }
        else
        {
            yError()<<"Unsupported Thimble: "<<thimble;
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(weArtClientMutex);
            thimbleInformationMap[thimble].hapticObject = std::make_unique<WeArtHapticObject>(weArtClient.get());
            thimbleInformationMap[thimble].hapticObject->handSideFlag = (int)handSide;
            thimbleInformationMap[thimble].hapticObject->actuationPointFlag = (int)actuationPoint;
            thimbleInformationMap[thimble].tracking = std::make_unique<WeArtThimbleTrackingObject>(handSide, actuationPoint);
            thimbleInformationMap[thimble].thimbleEffect = std::make_shared<ThimbleEffect>(WeArtTemperature(), WeArtForce(), WeArtTexture());
            thimbleInformationMap[thimble].hapticObject->AddEffect(thimbleInformationMap[thimble].thimbleEffect.get());
            thimbleInformationMap[thimble].rawSensorData = std::make_shared<WeArtTrackingRawData>(handSide, actuationPoint);

            weArtClient->AddThimbleTracking(thimbleInformationMap[thimble].tracking.get());
            weArtClient->AddMessageListener(thimbleInformationMap[thimble].rawSensorData.get());
        }

    }

    // Actuators check (Force, Texture, Temperature)

    // Force
    if (!(config.check("use_force_feedback")))
    {
        yWarning()<<LogPrefix<<"couldn't find force feedback, setting use_force_feedback as default value";
    }
    else
    {
        useForceFeedback = config.find("use_force_feedback").asBool();
    }

    // Temperature
    if (!(config.check("use_temperature_feedback")))
    {
        yWarning()<<LogPrefix<<"couldn't find temperature feedback, setting use_temperature_feedback as default value";
    }
    else 
    {
        useTemperatureFeedBack = config.find("use_temperature_feedback").asBool();
    }

    // Texture
    if (!(config.check("use_texture_feedback")))
    {
        yWarning()<<LogPrefix<<"couldn't find texture feedback, setting use_texture_feedback as default value";
    }
    else
    {
        useTextureFeedback = config.find("use_texture_feedback").asBool();
    }

    // Get TextureID
    if (!(config.check("textureId")))
    {
        yWarning()<<LogPrefix<<"couldn't find the textureID, using the default value"<<(int) textureId;
    }
    else
    {
        int textureIdParam = config.find("textureId").asInt32();
        if (textureIdParam < 0 || textureIdParam > 22)
        {
            yError()<<LogPrefix<<"textureId should be between 0 and 21";
            return false;
        }
        
        textureId = (TextureType) textureIdParam;
        yInfo()<<LogPrefix<<"textureId is:"<<textureIdParam;
    }

    // Get smoothing factor
    if (!(config.check("smoothing_factor")))
    {
        yWarning()<<LogPrefix<<"couldn't find smoothing factor (alpha), setting smoothing_factor as default value"<< alpha;
    }
    else 
    {
        alpha = config.find("smoothing_factor").asFloat64();
        if (alpha <= 0 || alpha > 1)
        {
            yError()<<LogPrefix<<"smoothing factor is not acceptable"<<alpha;
            return false;
        }    
    }

    // Get finger length
    if (!(config.check("finger_length_mm")))
    {
        yWarning()<<LogPrefix<<"couldn't find finger length, setting finger_length as default value"<< fingerLength;
    }
    else 
    {
        fingerLength = config.find("finger_length_mm").asFloat64();
        if (fingerLength <= 0)
        {
            yError()<<LogPrefix<<"finger length is not acceptable"<<fingerLength;
            return false;
        }
    }

    // Get Robot Joint limit
    // MIN
    yarp::os::Bottle* robotJointLimitMinYarp;
    if (!(config.check("min_joint_limit")) || !config.find("min_joint_limit").isList())
    {
        yError()<<LogPrefix<<"couldn't find robot joint limits, check the xml file";
        return false;
    }
    robotJointLimitMinYarp = config.find("min_joint_limit").asList();

    for (int i = 0; i < robotJointLimitMinYarp->size(); i++)
    {
        MIN_JOINT_LIMIT(i) = robotJointLimitMinYarp->get(i).asFloat64();
    }

    // MAX
    yarp::os::Bottle* robotJointLimitMaxYarp;
    if (!(config.check("max_joint_limit")) || !config.find("max_joint_limit").isList())
    {
        yError()<<LogPrefix<<"couldn't find robot joint limits, check the xml file";
        return false;
    }
    robotJointLimitMaxYarp = config.find("max_joint_limit").asList();

    for (int i = 0; i < robotJointLimitMaxYarp->size(); i++)
    {
        MAX_JOINT_LIMIT(i) = robotJointLimitMaxYarp->get(i).asFloat64();
    }

    // Runs and Starts the Client and connect it to the Server (Middleware)
    {
        std::lock_guard<std::mutex> lock(weArtClientMutex);
        weArtClient->Run();
        while (!weArtClient->IsConnected())
        {
            yError() << "The weArtClient is not connected!";
            return false;
        }
        weArtClient->Start();
        // Start receiving raw sensor data from the middleware
        weArtClient->StartRawData();
        weArtClient->StartCalibration();
    }
    yarp::os::Time::delay(1);
    while (weArtTrackingCalibration->getStatus() == CalibrationStatus::Running)
    {
        yDebug() << "Calibrating...!";
    }
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

    // Get URDF hand Joint Names
    yarp::os::Bottle* jointListUrdfYarp;
    if (!(config.check("considered_joint_list") && config.find("considered_joint_list").isList())) {
        yError() << LogPrefix << "Unable to find considered joint list in the config file.";
        return false;
    }
    jointListUrdfYarp = config.find("considered_joint_list").asList();

    for (size_t i = 0; i < jointListUrdfYarp->size(); i++) {
        consideredJointNameList.push_back(jointListUrdfYarp->get(i).asString());
    }
    yInfo() << LogPrefix << "considered urdf joint names: " << consideredJointNameList;

    // Get Finger-Tip Frame Names
    yarp::os::Bottle* fingerTipFrameListUrdfYarp;
    if (!(config.check("finger_tip_frame_list") && config.find("finger_tip_frame_list").isList())) {
        yError() << LogPrefix << "Unable to find finger tip frame list in the config file.";
        return false;
    }
    fingerTipFrameListUrdfYarp = config.find("finger_tip_frame_list").asList();

    for (size_t i = 0; i < fingerTipFrameListUrdfYarp->size(); i++) {
        fingerTipFrameNameList.push_back(fingerTipFrameListUrdfYarp->get(i).asString());
    }
    yInfo() << LogPrefix << "finger-tip frame names: " << fingerTipFrameNameList;

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

    // Fill the coupling Matrix
    yarp::os::Bottle* closureToHumanJointYarp;
    if (trackingAlgorithmType == "SCTA")
    {
        if (!(config.check("closure_to_human_joint_simple")) || !config.find("closure_to_human_joint_simple").isList())
        {
            yError() << LogPrefix << "couldn't find the proper coupling matrix";
            return false;
        }
        closureToHumanJointYarp = config.find("closure_to_human_joint_simple").asList();

        if (closureToHumanJointYarp->size() < 60)
        {
            yError() << LogPrefix << "size of the coupling matrix for simple tracking is:" << closureToHumanJointYarp->size() << "Required 60";
            return false;
        }
        closureToHumanJoint.resize(20, N_THIMBLES);
        for (int j = 0; j < closureToHumanJointYarp->size(); j++)
        {
            int col = j % N_THIMBLES;
            int row = j / N_THIMBLES;
            closureToHumanJoint(row, col) = closureToHumanJointYarp->get(j).asFloat64();
        }
    }
    else if (trackingAlgorithmType == "ASTA")
    {
        if (!(config.check("closure_to_human_joint_rawSensor")) || !config.find("closure_to_human_joint_rawSensor").isList())
        {
            yError() << LogPrefix << "couldn't find the proper coupling matrix";
            return false;
        }
        closureToHumanJointYarp = config.find("closure_to_human_joint_rawSensor").asList();

        if (closureToHumanJointYarp->size() < humanJointState.size() * consideredJointNameList.size())
        {
            yError() << LogPrefix << "size of the coupling matrix for rawSensor tracking is:" << closureToHumanJointYarp->size() << "Required 260";
            return false;
        }
        closureToHumanJoint.resize(humanJointState.size(), consideredJointNameList.size());
        for (size_t j = 0; j < closureToHumanJointYarp->size(); j++)
        {
            size_t col = j % consideredJointNameList.size();
            size_t row = j / consideredJointNameList.size();

            closureToHumanJoint(row, col) = closureToHumanJointYarp->get(j).asFloat64();
        }
        if (closureToHumanJoint.cols() != consideredJointNameList.size())
        {
            yError() << LogPrefix << "Sizes of the closureToHumanJoint and consideredJointNameList do not match." << closureToHumanJoint.cols();
            return false;
        }

        // Get URDF model path. NOTE: The directory containing the model should be added to the `YARP_DATA_DIRS`.
        std::string modelFile = config.check("model", yarp::os::Value("HumanHandModel.urdf")).asString();
        std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(modelFile);

        if (pathToModel == "")
        {
            yError() << "Failed to find" << modelFile;
            return false;
        }
        if (!mdlLoader.loadReducedModelFromFile(pathToModel, consideredJointNameList, "model"))
        {
            std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
            return EXIT_FAILURE;
        }

        // Get convergance rate
        if (!(config.check("convergence_rate")))
        {
            yWarning() << LogPrefix << "couldn't find convergence rate (k), setting convergence_rate as default value" << convergenceRate;
        }
        convergenceRate = config.find("convergence_rate").asFloat64();

        // Get Regularization Rate
        if (!(config.check("regularization_rate")))
        {
            yWarning() << LogPrefix << "couldn't find Regularization Rate (r), setting it as default value" << regularizationRate;
        }
        regularizationRate = config.find("regularization_rate").asFloat64();

        // Create a KinDynComputations class from the model
        if (!kinDynComp.loadRobotModel(mdlLoader.model()))
        {
            std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                      << mdlLoader.model().toString() << std::endl;
            return EXIT_FAILURE;
        }

        const iDynTree::Model &model = kinDynComp.model();

        // Prepare pointer to KinDynComputations object
        kinDyn = std::make_shared<iDynTree::KinDynComputations>();
        kinDyn->setFrameVelocityRepresentation(iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION);
        kinDyn->loadRobotModel(mdlLoader.model());

        jointVelocities.setZero(kinDyn->model().getNrOfDOFs());
        jointPositions.setZero(kinDyn->model().getNrOfDOFs());

        kinDyn->getRobotState(basePose, jointPositions, baseVelocity, jointVelocities, gravity);

        dynamics = std::make_shared<FloatingBaseSystemKinematics>();

        // initial state
        dynamics->setState({basePose.topRightCorner<3, 1>(),            // base postion
                            toManifRot(basePose.topLeftCorner<3, 3>()), // base rotation
                            jointPositions});                           // joint position

        integrator = std::make_shared<ForwardEuler<FloatingBaseSystemKinematics>>();
        integrator->setIntegrationStep(dT); // dt->period
        integrator->setDynamicalSystem(dynamics);

        // Prepare the parameter handler. The task setup is done through this. This can be substituted with a configuration file
        auto parameterHandler = std::make_shared<StdImplementation>();

        auto ikParameterHandler = std::make_shared<StdImplementation>();
        ikParameterHandler->setParameter("robot_velocity_variable_name", "robotVelocity");
        ikParameterHandler->setParameter("verbosity", false);
        parameterHandler->setGroup("IK", ikParameterHandler);

        parameterHandler->setParameter("tasks",
                                       std::vector<std::string>{"THUMB_DISTANCE_TASK",
                                                                "INDEX_DISTANCE_TASK",
                                                                "MIDDLE_DISTANCE_TASK",
                                                                "THUMB_GRAVITY_TASK",
                                                                "INDEX_GRAVITY_TASK",
                                                                "MIDDLE_GRAVITY_TASK",
                                                                "PALM_GRAVITY_TASK",
                                                                "SE3_TASK",
                                                                "REGULARIZATION_TASK",
                                                                "JOINT_LIMITS_TASK"});

        // Setup for the distance task

        // thumb
        auto thumbDistanceParameterHandler = std::make_shared<StdImplementation>();
        thumbDistanceParameterHandler->setParameter("type", "DistanceTask");
        thumbDistanceParameterHandler->setParameter("kp", convergenceRate);
        const Eigen::VectorXd thumbWeightRegularization = 1000 * Eigen::VectorXd::Ones(1);
        thumbDistanceParameterHandler->setParameter("weight", thumbWeightRegularization);
        thumbDistanceParameterHandler->setParameter("reference_frame_name", PalmSidePrefix + "Palm");
        thumbDistanceParameterHandler->setParameter("target_frame_name", fingerTipFrameNameList[0]);
        thumbDistanceParameterHandler->setParameter("priority", 1);
        parameterHandler->setGroup("THUMB_DISTANCE_TASK", thumbDistanceParameterHandler);

        // index
        auto indexDistanceParameterHandler = std::make_shared<StdImplementation>();
        indexDistanceParameterHandler->setParameter("type", "DistanceTask");
        indexDistanceParameterHandler->setParameter("kp", convergenceRate);
        const Eigen::VectorXd indexWeightRegularization = 1000 * Eigen::VectorXd::Ones(1);
        indexDistanceParameterHandler->setParameter("weight", indexWeightRegularization);
        indexDistanceParameterHandler->setParameter("reference_frame_name", PalmSidePrefix + "Palm");
        indexDistanceParameterHandler->setParameter("target_frame_name", fingerTipFrameNameList[1]);
        indexDistanceParameterHandler->setParameter("priority", 1);
        parameterHandler->setGroup("INDEX_DISTANCE_TASK", indexDistanceParameterHandler);

        // middle
        auto middleDistanceParameterHandler = std::make_shared<StdImplementation>();
        middleDistanceParameterHandler->setParameter("type", "DistanceTask");
        middleDistanceParameterHandler->setParameter("kp", convergenceRate);
        const Eigen::VectorXd middleWeightRegularization = 1000 * Eigen::VectorXd::Ones(1);
        middleDistanceParameterHandler->setParameter("weight", middleWeightRegularization);
        middleDistanceParameterHandler->setParameter("reference_frame_name", PalmSidePrefix + "Palm");
        middleDistanceParameterHandler->setParameter("target_frame_name", fingerTipFrameNameList[2]);
        middleDistanceParameterHandler->setParameter("priority", 1);
        parameterHandler->setGroup("MIDDLE_DISTANCE_TASK", middleDistanceParameterHandler);

        // SE3 Task
        auto SE3ParameterHandler = std::make_shared<StdImplementation>();
        SE3ParameterHandler->setParameter("kp_linear", 0.0);
        SE3ParameterHandler->setParameter("kp_angular", 0.0);
        SE3ParameterHandler->setParameter("type", "SE3Task");
        SE3ParameterHandler->setParameter("frame_name", "Pelvis");
        SE3ParameterHandler->setParameter("priority", 0);
        parameterHandler->setGroup("SE3_TASK", SE3ParameterHandler);

        // Joint limit task
        auto jointLimitsHandler = std::make_shared<StdImplementation>();
        jointLimitsHandler->setParameter("type", "JointLimitsTask");
        jointLimitsHandler->setParameter("sampling_time", dT);
        jointLimitsHandler->setParameter("use_model_limits", false);
        jointLimitsHandler->setParameter("priority", 0);
        const Eigen::VectorXd kLimRegularization = 0.5 * Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());
        jointLimitsHandler->setParameter("klim", kLimRegularization);

        upperLimits.resize(kinDyn->model().getNrOfDOFs());
        lowerLimits.resize(kinDyn->model().getNrOfDOFs());

        if (PalmSidePrefix == "Right")
        {
            upperLimits << 1.5, 1.0, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 3.0, 3.0, 3.0; // TODO: Read from XML
            lowerLimits << 0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, -3.0, -3.0;
        }
        else if (PalmSidePrefix == "Left")
        {
            upperLimits << -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            lowerLimits << -1.0, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5;
        }

        jointLimitsHandler->setParameter("upper_limits", upperLimits);
        jointLimitsHandler->setParameter("lower_limits", lowerLimits);
        parameterHandler->setGroup("JOINT_LIMITS_TASK", jointLimitsHandler);

        // Joint regularization task
        auto jointRegularizationHandler = std::make_shared<StdImplementation>();
        jointRegularizationHandler->setParameter("type", "JointTrackingTask");
        const Eigen::VectorXd kpRegularization = Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());
        const Eigen::VectorXd weightRegularization = 10 * kpRegularization;
        jointRegularizationHandler->setParameter("kp", kpRegularization);
        jointRegularizationHandler->setParameter("weight", weightRegularization);
        jointRegularizationHandler->setParameter("priority", 1);
        parameterHandler->setGroup("REGULARIZATION_TASK", jointRegularizationHandler);

        // Setup for the grvity task

        // palm
        auto palmGravityParameterHandler = std::make_shared<StdImplementation>();
        palmGravityParameterHandler->setParameter("type", "GravityTask");
        palmGravityParameterHandler->setParameter("kp", convergenceRate);
        const Eigen::Vector2d palmWeightRegularizationGr = 10 * Eigen::Vector2d::Ones(2);
        palmGravityParameterHandler->setParameter("weight", palmWeightRegularizationGr);
        palmGravityParameterHandler->setParameter("target_frame_name", PalmSidePrefix + "Palm");
        palmGravityParameterHandler->setParameter("priority", 1);
        parameterHandler->setGroup("PALM_GRAVITY_TASK", palmGravityParameterHandler);

        // thumb
        auto thumbGravityParameterHandler = std::make_shared<StdImplementation>();
        thumbGravityParameterHandler->setParameter("type", "GravityTask");
        thumbGravityParameterHandler->setParameter("kp", convergenceRate);
        const Eigen::Vector2d thumbWeightRegularizationGr = 10 * Eigen::Vector2d::Ones(2);
        thumbGravityParameterHandler->setParameter("weight", thumbWeightRegularizationGr);
        thumbGravityParameterHandler->setParameter("target_frame_name", fingerTipFrameNameList[0]);
        thumbGravityParameterHandler->setParameter("priority", 1);
        parameterHandler->setGroup("THUMB_GRAVITY_TASK", thumbGravityParameterHandler);

        // index
        auto indexGravityParameterHandler = std::make_shared<StdImplementation>();
        indexGravityParameterHandler->setParameter("type", "GravityTask");
        indexGravityParameterHandler->setParameter("kp", convergenceRate);
        const Eigen::Vector2d indexWeightRegularizationGr = 10 * Eigen::Vector2d::Ones(2);
        indexGravityParameterHandler->setParameter("weight", indexWeightRegularizationGr);
        indexGravityParameterHandler->setParameter("target_frame_name", fingerTipFrameNameList[1]);
        indexGravityParameterHandler->setParameter("priority", 1);
        parameterHandler->setGroup("INDEX_GRAVITY_TASK", indexGravityParameterHandler);

        // middle
        auto middleGravityParameterHandler = std::make_shared<StdImplementation>();
        middleGravityParameterHandler->setParameter("type", "GravityTask");
        middleGravityParameterHandler->setParameter("kp", convergenceRate);
        const Eigen::Vector2d middleWeightRegularizationGr = 10 * Eigen::Vector2d::Ones(1);
        middleGravityParameterHandler->setParameter("weight", middleWeightRegularizationGr);
        middleGravityParameterHandler->setParameter("target_frame_name", fingerTipFrameNameList[2]);
        middleGravityParameterHandler->setParameter("priority", 1);
        parameterHandler->setGroup("MIDDLE_GRAVITY_TASK", middleGravityParameterHandler);

        // Construct the IK
        ikProblem = std::make_shared<IntegrationBasedIKProblem>(QPInverseKinematics::build(parameterHandler, kinDyn));

        auto baseSE3Task = std::dynamic_pointer_cast<SE3Task>(ikProblem->ik->getTask("SE3_TASK").lock());
        baseSE3Task->setTaskControllerMode(SE3Task::Mode::Disable);
        baseSE3Task->setSetPoint(manif::SE3d::Identity(),
                                 manif::SE3d::Tangent::Zero());

        auto regularizationTask = std::dynamic_pointer_cast<JointTrackingTask>(ikProblem->ik->getTask("REGULARIZATION_TASK").lock());
        regularizationTask->setSetPoint(regularizationRate * Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs()));

        // Distance Task
        thumbDistanceTaskptr = std::dynamic_pointer_cast<DistanceTask>(ikProblem->ik->getTask("THUMB_DISTANCE_TASK").lock());

        indexDistanceTaskptr = std::dynamic_pointer_cast<DistanceTask>(ikProblem->ik->getTask("INDEX_DISTANCE_TASK").lock());

        middleDistanceTaskptr = std::dynamic_pointer_cast<DistanceTask>(ikProblem->ik->getTask("MIDDLE_DISTANCE_TASK").lock());

        // Gravity Task
        palmGravityTaskptr = std::dynamic_pointer_cast<GravityTask>(ikProblem->ik->getTask("PALM_GRAVITY_TASK").lock());

        thumbGravityTaskptr = std::dynamic_pointer_cast<GravityTask>(ikProblem->ik->getTask("THUMB_GRAVITY_TASK").lock());

        indexGravityTaskptr = std::dynamic_pointer_cast<GravityTask>(ikProblem->ik->getTask("INDEX_GRAVITY_TASK").lock());

        middleGravityTaskptr = std::dynamic_pointer_cast<GravityTask>(ikProblem->ik->getTask("MIDDLE_GRAVITY_TASK").lock());

        jointVelocity = Eigen::VectorXd::Zero(model.getNrOfDOFs());

    }
    else
    {
        yError() << "The specified tracking algorithm is not supported!";
        return false;
    }

    return true;
}

bool WeArtGlove::WeArtGloveImpl::update()
{
    std::lock_guard<std::mutex> lock(mutex);

    // To get the sensor raw data (IMUs and ToF)
    for (auto &[key, element] : thimbleInformationMap)
    {
        int thimbleNumber = thimbleNameToNumber(key);

        if (trackingAlgorithmType == "SCTA")
        {
            //Raw Closure data
            weartClosure(thimbleNumber) = element.tracking->GetClosure();

            // Apply exponential smoothing
            weartClosureSmooth(thimbleNumber) = alpha * weartClosure(thimbleNumber) + (1 - alpha) * weartClosureSmooth(thimbleNumber);
        }
        else if (trackingAlgorithmType == "ASTA")
        {
            weartRawDataSample = element.rawSensorData->GetLastSample();

            // Acceleration
            weartRawDataAcc(thimbleNumber, 0) = weartRawDataSample.data.accelerometer.x;
            weartRawDataAcc(thimbleNumber, 1) = weartRawDataSample.data.accelerometer.y;
            weartRawDataAcc(thimbleNumber, 2) = weartRawDataSample.data.accelerometer.z;

            // Gyroscope
            weartRawDataGyro(thimbleNumber, 0) = weartRawDataSample.data.gyroscope.x;
            weartRawDataGyro(thimbleNumber, 1) = weartRawDataSample.data.gyroscope.y;
            weartRawDataGyro(thimbleNumber, 2) = weartRawDataSample.data.gyroscope.z;

            // Raw ToF
            weartRawDataTof(thimbleNumber) = weartRawDataSample.data.timeOfFlight.distance;
        }

        element.thimbleEffect->Set(element.thimbleEffect->effTemperature, element.thimbleEffect->effForce, element.thimbleEffect->effTexture);

        if (element.hapticObject->activeEffects.size() <= 0)
            element.hapticObject->AddEffect(element.thimbleEffect.get());
        else
            element.hapticObject->UpdateEffects();
    }

    if (trackingAlgorithmType == "SCTA")
    {
        humanJointState = closureToHumanJoint * weartClosureSmooth;

        // saturate the computed values in the interval [0,1]
        for (int i = 0; i < humanJointState.rows(); i++)
        {
            for (int j = 0; j < humanJointState.cols(); j++)
            {
                humanJointState(i, j) = std::max(0.0, std::min(1.0, humanJointState(i, j)));
            }
        }
        humanJointState = humanJointState.cwiseProduct((MAX_JOINT_LIMIT - MIN_JOINT_LIMIT)) + MIN_JOINT_LIMIT;
    }
    else if (trackingAlgorithmType == "ASTA")
    {
        // Set the desired distance for the distance task. The values are devided by 1000.0 since they ToFs are in mm.
        thumbDistanceTaskptr->setDesiredDistance(weartRawDataTof(0) / 1000.0);  // thumb
        indexDistanceTaskptr->setDesiredDistance(weartRawDataTof(1) / 1000.0);  // index
        middleDistanceTaskptr->setDesiredDistance(weartRawDataTof(2) / 1000.0); // middle

        weartRawDataAccRotated = fingerSensorToBodyRotation * weartRawDataAcc.transpose();
        weartRawDataGyroRotated = fingerSensorToBodyRotation * weartRawDataGyro.transpose();

        // Convert Degree to Radians
        weartRawDataGyroRotated = weartRawDataGyroRotated * EIGEN_PI / 180.0;

        // Set the desired gravity direction for the gravity task
        thumbGravityTaskptr->setDesiredGravityDirectionInTargetFrame(weartRawDataAccRotated.col(0));
        indexGravityTaskptr->setDesiredGravityDirectionInTargetFrame(weartRawDataAccRotated.col(1));
        middleGravityTaskptr->setDesiredGravityDirectionInTargetFrame(weartRawDataAccRotated.col(2));
        palmGravityTaskptr->setDesiredGravityDirectionInTargetFrame(palmSensorToBodyRotation * weartRawDataAcc.row(3).transpose());

        const auto &[basePosition, baseRotation, jointPosition] = integrator->getSolution();

        // update the KinDynComputations object
        baseTransform.topLeftCorner<3, 3>() = baseRotation.rotation();
        baseTransform.topRightCorner<3, 1>() = basePosition;

        // solve the IK
        ikProblem->ik->advance();

        // get the output of the IK
        baseVelocity = ikProblem->ik->getOutput().baseVelocity.coeffs();
        jointVelocity = ikProblem->ik->getOutput().jointVelocity;

        // propagate the dynamical system
        dynamics->setControlInput({baseVelocity, jointVelocity});
        integrator->integrate(0ms, dT);

        kinDyn->setRobotState(baseTransform,
                              jointPosition,
                              baseVelocity,
                              jointVelocity,
                              gravity);

        if (PalmSidePrefix == "Right")
        {
            humanJointState = closureToHumanJoint * jointPosition;
        }
        else if (PalmSidePrefix == "Left")
        {
            humanJointState = closureToHumanJoint * -jointPosition;
        }
    }

    return true;
}

bool WeArtGlove::WeArtGloveImpl::close()
{
    for (auto &[key, element] : thimbleInformationMap)
    {
        element.hapticObject->RemoveEffect(element.thimbleEffect.get());
    }

    yarp::os::Time::delay(1);
    {
        std::lock_guard<std::mutex> lock(weArtClientMutex);
        weArtClient->StopRawData();
        weArtClient->Stop();
        weArtClient->Close();
    }
    return true;
}

// Constructor
WeArtGlove::WeArtGlove()
    : PeriodicThread(0.01)
    , pImpl{std::make_unique<WeArtGloveImpl>()}
{}

// Destructor
WeArtGlove::~WeArtGlove() = default;

bool WeArtGlove::open(yarp::os::Searchable& config)
{

    // Check the device name
    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yInfo() << LogPrefix << "Using default wearable name WeArtGlove";
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
        pImpl->weartGloveJointSensorVector.push_back(std::make_shared<WeArtGloveImpl::WeArtGloveVirtualJointKinSensor>(pImpl.get(), i, pImpl->jointSensorPrefix + pImpl->humanJointNameList[i]));
    }

    // Create Virtual Link Sensors
    pImpl->linkSensorPrefix = getWearableName() + sensor::IVirtualLinkKinSensor::getPrefix();
    pImpl->weartGloveVirtualLinkKinSensor.push_back(
        std::make_shared<WeArtGloveImpl::WeArtGloveVirtualLinkKinSensor>(
            pImpl.get(), pImpl->linkSensorPrefix + pImpl->humanHandLinkName));

    for (size_t i = 0; i < pImpl->humanFingerNames.size(); i++) {
        pImpl->weartGloveVirtualLinkKinSensor.push_back(
            std::make_shared<WeArtGloveImpl::WeArtGloveVirtualLinkKinSensor>(
                pImpl.get(),
                pImpl->linkSensorPrefix + pImpl->humanFingerNames[i]
                    + "::fingertip"));
    }

    // Create Actuators (Force, Texture, Temperature)
    pImpl->forceActuatorPrefix = getWearableName() + actuator::IHaptic::getPrefix();
    pImpl->textureActuatorPrefix = getWearableName() + actuator::IHaptic::getPrefix();
    pImpl->temperatureActuatorPrefix = getWearableName() + actuator::IHaptic::getPrefix();

    for (auto& [key, entry] : pImpl->thimbleInformationMap)
    {
        // Force
        if (pImpl->useForceFeedback)
        {
            pImpl->weartGloveForceActuatorVector.push_back(std::make_shared<WeArtGloveImpl::WeArtGloveForceActuator>(pImpl.get(), entry.thimbleEffect, pImpl->forceActuatorPrefix));
        }

        // Texture
        if (pImpl->useTextureFeedback)
        {
            pImpl->weartGloveTextureActuatorVector.push_back(std::make_shared<WeArtGloveImpl::WeArtGloveTextureActuator>(pImpl.get(), entry.thimbleEffect, pImpl->textureActuatorPrefix));
        }

        // Temperature
        if (pImpl->useTemperatureFeedBack)
        {
            pImpl->weartGloveTemperatureActuatorVector.push_back(std::make_shared<WeArtGloveImpl::WeArtGloveTemperatureActuator>(pImpl.get(), entry.thimbleEffect, pImpl->temperatureActuatorPrefix));
        }
    }

    setPeriod(pImpl->period);

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the period thread.";
        return false;
    }
    yInfo() << "The Middleware version is " <<pImpl->mwListener->lastStatus().version;
    return true;
}

// =================================================
// WeArtGlove implementation of VirtualLinkKinSensor
// =================================================

class WeArtGlove::WeArtGloveImpl::WeArtGloveVirtualLinkKinSensor
    : public sensor::IVirtualLinkKinSensor
{
public:
    WeArtGloveVirtualLinkKinSensor(WeArtGlove::WeArtGloveImpl* gloveImplPtr,
                                   const sensor::SensorName& name = {},
                                   const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
        : IVirtualLinkKinSensor(name, status)
        , gloveImpl(gloveImplPtr)
        , sensorName(name)
    {
        assert(gloveImpl != nullptr);
    }

    ~WeArtGloveVirtualLinkKinSensor() override = default;

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
    WeArtGlove::WeArtGloveImpl* gloveImpl{nullptr};
    std::string sensorName;
};

// ==================================================
// WeArtGlove implementation of VirtualJointKinSensor
// ==================================================

class WeArtGlove::WeArtGloveImpl::WeArtGloveVirtualJointKinSensor
    : public sensor::IVirtualJointKinSensor
{
public:
    WeArtGloveVirtualJointKinSensor(WeArtGlove::WeArtGloveImpl* gloveImplPtr,
                                    const int humanJointIndex,
                                    const sensor::SensorName& name = {},
                                    const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
        : IVirtualJointKinSensor(name, status)
        , gloveImpl(gloveImplPtr)
        , humanJointIndex(humanJointIndex)
    {
        assert(gloveImpl != nullptr);
    }

    ~WeArtGloveVirtualJointKinSensor() override = default;

    bool getJointPosition(double& position) const override
    {

        std::lock_guard<std::mutex> lock(gloveImpl->mutex);

        position = gloveImpl->humanJointState(humanJointIndex);

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
    WeArtGlove::WeArtGloveImpl* gloveImpl{nullptr};
    const int humanJointIndex;
};

// ==================================================
// WeArtGlove implementation of IHaptic actuator TODO: IForce
// ==================================================

class WeArtGlove::WeArtGloveImpl::WeArtGloveForceActuator : public wearable::actuator::IHaptic
{
public:
    inline static const std::string suffix = "HapticFeedback"; //ForceFeedback

    WeArtGloveForceActuator(WeArtGlove::WeArtGloveImpl* gloveImplPtr,
                             std::shared_ptr<ThimbleEffect> thimbleEffect,
                             const actuator::ActuatorName& name = {},
                             const actuator::ActuatorStatus& status = actuator::ActuatorStatus::Ok)
        : IHaptic(name + suffix, status)
        , gloveImpl(gloveImplPtr)
        , thimbleEffect(thimbleEffect)
    {
        assert(gloveImpl != nullptr);

        yInfo()<<"Force Actuator name:"<<name + suffix;
    }
    ~WeArtGloveForceActuator() override = default;

    bool setHapticCommand(double& value) const override
    {
        yError() << LogPrefix << "Wrong method has been called! To set the haptic command please use the setHapticsCommand method.";
        return false;
    }

    bool setHapticCommands(const std::vector<double>& forceValue, const std::vector<double>& vibrotactileValue) const override
    {
        std::lock_guard<std::mutex> lock(gloveImpl->mutex);
        for (auto &[key, element] : gloveImpl->thimbleInformationMap)
        {
            int thimbleNumber = gloveImpl->thimbleNameToNumber(key);
            if (forceValue[thimbleNumber] < 0 || forceValue[thimbleNumber] > 100)
            {
                yError() << getActuatorName() << ":Received the command" << forceValue[thimbleNumber] << "The value should be between 0 and 100";
                return false;
            }
            WeArtForce force = WeArtForce(true, (float)(forceValue[thimbleNumber] / 100.0));

            gloveImpl->thimbleInformationMap[key].thimbleEffect->effForce = force;
        }

        return true;
    }

    inline void setStatus(const actuator::ActuatorStatus& status) { m_status = status; }

private:
    WeArtGlove::WeArtGloveImpl* gloveImpl{nullptr};
    std::shared_ptr<ThimbleEffect> thimbleEffect;
};

// ==================================================
// WeArtGlove implementation of IHaptic actuator for Texture
// ==================================================

class WeArtGlove::WeArtGloveImpl::WeArtGloveTextureActuator : public wearable::actuator::IHaptic
{
public:
    inline static const std::string suffix = "HapticFeedback"; //VibroTactileFeedback

    WeArtGloveTextureActuator(WeArtGlove::WeArtGloveImpl* gloveImplPtr,
                             std::shared_ptr<ThimbleEffect> thimbleEffect,
                             const actuator::ActuatorName& name = {},
                             const actuator::ActuatorStatus& status = actuator::ActuatorStatus::Ok)
        : IHaptic(name + suffix, status)
        , gloveImpl(gloveImplPtr)
        , thimbleEffect(thimbleEffect)
    {
        assert(gloveImpl != nullptr);

        yInfo()<<"Texture Actuator name:"<<name + suffix;
    }
    ~WeArtGloveTextureActuator() override = default;

    bool setHapticCommand(double& value) const override
    {
        yError() << LogPrefix << "Wrong method has been called! To set the haptic command please use the setHapticsCommand method.";
        return false;
    }

    bool setHapticCommands(const std::vector<double>& forceValue, const std::vector<double>& vibrotactileValue) const override
    {
        std::lock_guard<std::mutex> lock(gloveImpl->mutex);
        for (auto &[key, element] : gloveImpl->thimbleInformationMap)
        {
            int thimbleNumber = gloveImpl->thimbleNameToNumber(key);
            if (vibrotactileValue[thimbleNumber] > 100.0 || vibrotactileValue[thimbleNumber] < 0.0)
            {
                yError()<<getActuatorName()<<":Received the command"<<vibrotactileValue[thimbleNumber]<<"The value should be between 0 and 100";
                return false;
            }

            WeArtTexture texture;

            texture.active = true;
            texture.volume((float)vibrotactileValue[thimbleNumber]);
            texture.textureVelocity(float(0.1)); // The vale is chosen as a default intensity. It can be between (0-1).
            texture.textureType(gloveImpl->textureId);

            gloveImpl->thimbleInformationMap[key].thimbleEffect->effTexture = texture;
        }
        return true;
    }

    inline void setStatus(const actuator::ActuatorStatus& status) { m_status = status; }

private:
    WeArtGlove::WeArtGloveImpl* gloveImpl{nullptr};
    std::shared_ptr<ThimbleEffect> thimbleEffect;
};

// ==================================================
// WeArtGlove implementation of IHaptic actuator for Temperature    // TODO Iheater.h
// ==================================================

class WeArtGlove::WeArtGloveImpl::WeArtGloveTemperatureActuator : public wearable::actuator::IHaptic //TODO IHeater
{
public:
    inline static const std::string suffix = "::HapticFeedback"; //TemperatureFeedback

    WeArtGloveTemperatureActuator(WeArtGlove::WeArtGloveImpl* gloveImplPtr,
                             std::shared_ptr<ThimbleEffect> thimbleEffect,
                             const actuator::ActuatorName& name = {},
                             const actuator::ActuatorStatus& status = actuator::ActuatorStatus::Ok)
        : IHaptic(name + suffix, status)
        , gloveImpl(gloveImplPtr)
        , thimbleEffect(thimbleEffect)
    {
        assert(gloveImpl != nullptr);

        yInfo()<<"Temperature Actuator name:"<<name + suffix;
    }
    ~WeArtGloveTemperatureActuator() override = default;

    private:
        inline const static double MIN_TEMP = 18;
        inline const static double MAX_TEMP = 43;

    bool setHapticCommand(double& value) const override
    {
        yError() << LogPrefix << "Wrong method has been called! To set the haptic command please use the setHapticsCommand method.";
        return false;
    }

    bool setHapticCommands(const std::vector<double>& forceValue, const std::vector<double>& vibrotactileValue) const override
    {
        std::lock_guard<std::mutex> lock(gloveImpl->mutex);
        for (auto &[key, element] : gloveImpl->thimbleInformationMap)
        {
            double command = 0.0;// Temperature sensor is not on the robot yet!
            std::lock_guard<std::mutex> lock(gloveImpl->mutex);

            WeArtTemperature temperature;

            if (command >= MIN_TEMP && command <= MAX_TEMP)
            {
                temperature.active = true;
                command = (command - MIN_TEMP)/(MAX_TEMP - MIN_TEMP);
            }
            else if (command <= 0.0)
            {
                temperature.active = false;
            }
            else if (command < MIN_TEMP || command > MAX_TEMP)
            {
                yError()<<getActuatorName()<<":Received the command"<<command<<"The value should be between"<< MIN_TEMP <<"and" <<MAX_TEMP;
                return false;
            }

            temperature.value((float) command);

            gloveImpl->thimbleInformationMap[key].thimbleEffect->effTemperature = temperature;
        }
        return true;
    }

    inline void setStatus(const actuator::ActuatorStatus& status) { m_status = status; }

private:
    WeArtGlove::WeArtGloveImpl* gloveImpl{nullptr};
    std::shared_ptr<ThimbleEffect> thimbleEffect; 
};

// ==================================================

void WeArtGlove::run()
{
    // Get timestamp
    pImpl->timeStamp.time = yarp::os::Time::now();

    // to implement
    pImpl->update();
}

bool WeArtGlove::close()
{
    this->askToStop();
    while(this->isRunning())
    {
        yDebug()<<"Waiting for the thread to stop ";
        yarp::os::Time::delay(1);

    }    

    yDebug()<<"Thread stopped ";

    if (!pImpl->close()) {
        yError() << LogPrefix << "Cannot close correctly the WeArt glove implementation.";
        return false;
    }

    yDebug()<<"end of closing ";

    return true;
}

// =========================
// IPreciselyTimed interface
// =========================

yarp::os::Stamp WeArtGlove::getLastInputStamp()
{
    // Stamp count should be always zero
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return yarp::os::Stamp(0, pImpl->timeStamp.time);
}

// ---------------------------------------
// Implement Sensors and Actuators Methods
// ---------------------------------------

wearable::WearableName WeArtGlove::getWearableName() const
{
    return pImpl->wearableName + wearable::Separator;
}

wearable::WearStatus WeArtGlove::getStatus() const
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

wearable::TimeStamp WeArtGlove::getTimeStamp() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return {pImpl->timeStamp.time, 0};
}

wearable::SensorPtr<const wearable::sensor::ISensor>
WeArtGlove::getSensor(const wearable::sensor::SensorName name) const
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
WeArtGlove::getSensors(const wearable::sensor::SensorType aType) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> outVec;
    switch (aType)
    {
        case sensor::SensorType::VirtualLinkKinSensor: {
            outVec.reserve(pImpl->weartGloveVirtualLinkKinSensor.size());
            for (const auto& weartgloveLinkSensor : pImpl->weartGloveVirtualLinkKinSensor)
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(weartgloveLinkSensor));
            break;
        }
        case sensor::SensorType::VirtualJointKinSensor: {
            outVec.reserve(pImpl->weartGloveJointSensorVector.size());
            for (const auto& element : pImpl->weartGloveJointSensorVector)
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
WeArtGlove::getActuator(const wearable::actuator::ActuatorName name) const
{
    wearable::VectorOfElementPtr<const wearable::actuator::IActuator> actuators = getAllActuators();

    for (const auto& a : actuators) {
        if (a->getActuatorName() == name) {
            yDebug()<<"Found actuator"<<name<<"!";
            return a;
        }
    }
    yWarning() << LogPrefix << "User specified actuator name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfElementPtr<const wearable::actuator::IActuator>
WeArtGlove::getActuators(const wearable::actuator::ActuatorType aType) const
{
    wearable::VectorOfElementPtr<const wearable::actuator::IActuator> outVec;
    switch (aType) {
        case wearable::actuator::ActuatorType::Haptic: {
            outVec.reserve(pImpl->weartGloveForceActuatorVector.size());
            for (const auto& forceActuator : pImpl->weartGloveForceActuatorVector) {
                outVec.push_back(static_cast<ElementPtr<actuator::IActuator>>(forceActuator));
            }
            for (const auto& textureActuator : pImpl->weartGloveTextureActuatorVector) {
                outVec.push_back(static_cast<ElementPtr<actuator::IActuator>>(textureActuator));
            }
            for (const auto& temperatureActuator : pImpl->weartGloveTemperatureActuatorVector) {
                outVec.push_back(static_cast<ElementPtr<actuator::IActuator>>(temperatureActuator));
            }
            break;
        }
        default: {
            return {};
        }
    }
    return outVec;
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
WeArtGlove::getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
WeArtGlove::getVirtualJointKinSensor(const wearable::sensor::SensorName name) const
{
    for (SensorPtr<WeArtGloveImpl::WeArtGloveVirtualJointKinSensor> &element : pImpl->weartGloveJointSensorVector)
    {
        if (name == element->getSensorName())
        {
            return element;
        }
    }

    return nullptr;
}

wearable::ElementPtr<const wearable::actuator::IHaptic>
WeArtGlove::getHapticActuator(const actuator::ActuatorName name) const
{

    wearable::VectorOfElementPtr<const wearable::actuator::IActuator> actuators = getAllActuators();

    for (auto &entry : pImpl->weartGloveForceActuatorVector)
    {
        if (name == entry->getActuatorName())
        {
            return  static_cast<wearable::ElementPtr<const wearable::actuator::IHaptic>>(entry);
        }
    }

    for (auto &entry : pImpl->weartGloveTextureActuatorVector)
    {
        if (name == entry->getActuatorName())
        {
            return  static_cast<wearable::ElementPtr<const wearable::actuator::IHaptic>>(entry);
        }
    }

    for (auto &entry : pImpl->weartGloveTemperatureActuatorVector)
    {
        if (name == entry->getActuatorName())
        {
            return  static_cast<wearable::ElementPtr<const wearable::actuator::IHaptic>>(entry);
        }
    }

    return nullptr;
}

// =========================
// Defintion of Utilities
// =========================

int WeArtGlove::WeArtGloveImpl::thimbleNameToNumber(const std::string &name)
{
    if (name == "thumb")
    {
        return 0;
    }

    if (name == "index")
    {
        return 1;
    }

    if (name == "middle")
    {
        return 2;
    }
    if (name == "palm")
    {
        return 3;
    }

    return  -1;
}
