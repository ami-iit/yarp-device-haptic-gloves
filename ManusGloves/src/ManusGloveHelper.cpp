/**
 * @file ManusGloveHelper.cpp
 * @authors Ehsan Ranjbari <ehsan.ranjbari@iit.it>
 * @copyright 2024 AMI Lab - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2024
 */

#include <ManusGloveHelper.h>
#include <ManusGlove.h>

using namespace manusGlove;
using namespace std;

ManusGloveHelper* ManusGloveHelper::s_Instance = nullptr;

ManusGloveHelper::ManusGloveHelper()
{
    s_Instance = this;
}

ManusGloveHelper::~ManusGloveHelper()
{
    s_Instance = nullptr;
}

bool ManusGloveHelper::Initialize(bool p_hostType)
{
    m_ShouldConnectLocally = p_hostType;
    // before we can use the SDK, some internal SDK bits need to be initialized.
    // however after initializing, the SDK is not yet connected to a host or doing anything network related just yet.
    if (CoreSdk_Initialize(SessionType::SessionType_CoreSDK) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "SDK::Failed to initilize Core SDK.";
        return false;
    }

    if (!RegisterAllCallbacks())
    {
        yError() << ManusGlove_LogPrefix << "SDK::Failed to register all the callbacks.";
        return false;
    }

    if (!InitializeCoordinateSystem())
    {
        yError() << ManusGlove_LogPrefix << "SDK::Failed to initialize coordinate system.";
        return false;
    }

    // To look and get the number of available hosts.
    if(!LookingForHosts(m_ShouldConnectLocally))
    {
        yError() << ManusGlove_LogPrefix << "Failed to initilize. Check the hosts";
        return false;
    }

    if (!ConnectingToCore())
    {
        yError() << ManusGlove_LogPrefix << "Failed to connect to the core.";
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    PrintDongleData();

    return true;
}

bool ManusGloveHelper::ShutDown()
{
    if (CoreSdk_ShutDown() != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to shut down the SDK wrapper.";
        return false;
    }
    yInfo() << ManusGlove_LogPrefix << "Shutting down the core SDK.";

    return true;
}

bool ManusGloveHelper::RegisterAllCallbacks()
{
    // Regtister the callback for when manus core is connected to he SDK
    // it is optional, but helps trigger your client nicely if needed.
    // see the function OnConnectedCallback for more details
    if (CoreSdk_RegisterCallbackForOnConnect(*OnConnectedCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback function for after connecting to Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for when manus core is disconnected to the SDK
    // it is optional, but helps trigger your client nicely if needed.
    // see OnDisconnectedCallback for more details.
    if (CoreSdk_RegisterCallbackForOnDisconnect(*OnDisconnectedCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback function for after disconnecting from Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for when manus core is sending landscape data
    // it is optional, but this allows for a reactive adjustment of device information.
    // see OnLandscapeCallback for more details.
    if (CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback for landscape from Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for when manus core is sending System messages
    // This is usually not used by client applications unless they want to show errors/events from core.
    // see OnSystemCallback for more details.
    if (CoreSdk_RegisterCallbackForSystemStream(*OnSystemCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback function for system feedback from Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for when manus core is sending Ergonomics data
    // it is optional, but helps trigger your client nicely if needed.
    // see OnErgonomicsCallback for more details.
    if (CoreSdk_RegisterCallbackForErgonomicsStream(*OnErgonomicsCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() <<ManusGlove_LogPrefix << "Failed to register callback function for ergonomics data from Manus Core. SDK is not available.";
        return false;
    }

    return true;
}

bool ManusGloveHelper::InitializeCoordinateSystem()
{
    // after everything is registered and initialized.
    // we must also set the coordinate system being used for the data in this client.
    // (each client can have their own settings. unreal and unity for instance use different coordinate systems)
    // if this is not set, the SDK will NOT connect to any Manus core host.
    CoordinateSystemDirection t_Direction;
    bool p_UseWorldCoordinates = true;
    t_Direction.x = AxisDirection::AxisDirection_Forward;
    t_Direction.y = AxisDirection::AxisDirection_Right;
    t_Direction.z = AxisDirection::AxisDirection_Up;

    if (CoreSdk_InitializeCoordinateSystemWithDirection(t_Direction, p_UseWorldCoordinates) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to Initialize the coordinate system.";
        return false;
    }
    return true;
}

void ManusGloveHelper::LoadSkeleton()
{
    uint32_t t_SklIndex = 0;

    SkeletonSetupInfo t_SKL;
    SkeletonSetupInfo_Init(&t_SKL);
    t_SKL.type = SkeletonType::SkeletonType_Hand;
    t_SKL.settings.scaleToTarget = true;
    t_SKL.settings.useEndPointApproximations = true;
    t_SKL.settings.targetType = SkeletonTargetType::SkeletonTargetType_UserIndexData;
    // If the user does not exist then the added skeleton will not be animated.
    // Same goes for any other skeleton made for invalid users/gloves.
    t_SKL.settings.skeletonTargetUserIndexData.userIndex = 0;

    CopyString(t_SKL.name, sizeof(t_SKL.name), std::string("LeftHand"));//TODO: RightHand? (create a handside string)

    SDKReturnCode t_Res = CoreSdk_CreateSkeletonSetup(t_SKL, &t_SklIndex);
    if (t_Res != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to Create Skeleton Setup.";
        return;
    }

    // setup nodes and chains for the skeleton hand
    if (!SetupHandNodes(t_SklIndex))
        return;
    if (!SetupHandChains(t_SklIndex))
        return;

    // load skeleton
    uint32_t t_ID = 0;
    t_Res = CoreSdk_LoadSkeleton(t_SklIndex, &t_ID);
    if (t_Res != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to load skeleton.";
        return;
    }

    if (t_ID == 0)
    {
        yError() << ManusGlove_LogPrefix << "Failed to give skeleton an ID.";
    }
    m_LoadedSkeletons.push_back(t_ID);
}

NodeSetup ManusGloveHelper::CreateNodeSetup(uint32_t p_Id, uint32_t p_ParentId, float p_PosX, float p_PosY, float p_PosZ, std::string p_Name)
{
    NodeSetup t_Node;
    NodeSetup_Init(&t_Node);
    t_Node.id = p_Id; // Every ID needs to be unique per node in a skeleton.
    CopyString(t_Node.name, sizeof(t_Node.name), p_Name);
    t_Node.type = NodeType::NodeType_Joint;
    // Every node should have a parent unless it is the Root node.
    t_Node.parentID = p_ParentId; // Setting the node ID to its own ID ensures it has no parent.
    t_Node.settings.usedSettings = NodeSettingsFlag::NodeSettingsFlag_None;

    t_Node.transform.position.x = p_PosX;
    t_Node.transform.position.y = p_PosY;
    t_Node.transform.position.z = p_PosZ;
    return t_Node;
}

ManusVec3 ManusGloveHelper::CreateManusVec3(float p_X, float p_Y, float p_Z)
{
    ManusVec3 t_Vec;
    t_Vec.x = p_X;
    t_Vec.y = p_Y;
    t_Vec.z = p_Z;
    return t_Vec;
}

bool ManusGloveHelper::SetupHandNodes(uint32_t p_SklIndex)
{
    // Define number of fingers per hand and number of joints per finger
    const uint32_t t_NumFingers = 5;
    const uint32_t t_NumJoints = 4;

    // Create an array with the initial position of each hand node.
    // Note, these values are just an example of node positions and refer to the hand laying on a flat surface.
    ManusVec3 t_Fingers[t_NumFingers * t_NumJoints] = {
        CreateManusVec3(0.024950f, 0.000000f, 0.025320f), // Thumb CMC joint
        CreateManusVec3(0.000000f, 0.000000f, 0.032742f), // Thumb MCP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.028739f), // Thumb IP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.028739f), // Thumb Tip joint

        // CreateManusVec3(0.011181f, 0.031696f, 0.000000f), //Index CMC joint // Note: we are not adding the matacarpal bones in this example, if you want to animate the metacarpals add each of them to the corresponding finger chain.
        CreateManusVec3(0.011181f, 0.000000f, 0.052904f), // Index MCP joint, if metacarpal is present: CreateManusVec3(0.000000f, 0.000000f, 0.052904f)
        CreateManusVec3(0.000000f, 0.000000f, 0.038257f), // Index PIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.020884f), // Index DIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.018759f), // Index Tip joint

        // CreateManusVec3(0.000000f, 0.033452f, 0.000000f), //Middle CMC joint
        CreateManusVec3(0.000000f, 0.000000f, 0.051287f), // Middle MCP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.041861f), // Middle PIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.024766f), // Middle DIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.019683f), // Middle Tip joint

        // CreateManusVec3(-0.011274f, 0.031696f, 0.000000f), //Ring CMC joint
        CreateManusVec3(-0.011274f, 0.000000f, 0.049802f), // Ring MCP joint, if metacarpal is present: CreateManusVec3(0.000000f, 0.000000f, 0.049802f),
        CreateManusVec3(0.000000f, 0.000000f, 0.039736f),  // Ring PIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.023564f),  // Ring DIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.019868f),  // Ring Tip joint

        // CreateManusVec3(-0.020145f, 0.027538f, 0.000000f), //Pinky CMC joint
        CreateManusVec3(-0.020145f, 0.000000f, 0.047309f), // Pinky MCP joint, if metacarpal is present: CreateManusVec3(0.000000f, 0.000000f, 0.047309f),
        CreateManusVec3(0.000000f, 0.000000f, 0.033175f),  // Pinky PIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.018020f),  // Pinky DIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.019129f),  // Pinky Tip joint
    };

    // skeleton entry is already done. just the nodes now.
    // setup a very simple node hierarchy for fingers
    // first setup the root node
    //
    // root, This node has ID 0 and parent ID 0, to indicate it has no parent.
    SDKReturnCode t_Res = CoreSdk_AddNodeToSkeletonSetup(p_SklIndex, CreateNodeSetup(0, 0, 0, 0, 0, "Hand"));
    if (t_Res != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << "Failed to Add Node To Skeleton Setup. The error given was: " << t_Res;
        return false;
    }

    // then loop for 5 fingers
    int t_FingerId = 0;
    for (uint32_t i = 0; i < t_NumFingers; i++)
    {
        uint32_t t_ParentID = 0;
        // then the digits of the finger that are linked to the root of the finger.
        for (uint32_t j = 0; j < t_NumJoints; j++)
        {
            t_Res = CoreSdk_AddNodeToSkeletonSetup(p_SklIndex, CreateNodeSetup(1 + t_FingerId + j, t_ParentID, t_Fingers[i * 4 + j].x, t_Fingers[i * 4 + j].y, t_Fingers[i * 4 + j].z, "fingerdigit"));
            if (t_Res != SDKReturnCode::SDKReturnCode_Success)
            {
                yError() << ManusGlove_LogPrefix << "Failed to Add Node To Skeleton Setup.";
                return false;
            }
            t_ParentID = 1 + t_FingerId + j;
        }
        t_FingerId += t_NumJoints;
    }
    return true;
}

bool ManusGloveHelper::SetupHandChains(uint32_t p_SklIndex)
{
    // Add the Hand chain, this identifies the wrist of the hand
    {
        ChainSettings t_ChainSettings;
        ChainSettings_Init(&t_ChainSettings);
        t_ChainSettings.usedSettings = ChainType::ChainType_Hand;
        t_ChainSettings.hand.handMotion = HandMotion::HandMotion_IMU;
        t_ChainSettings.hand.fingerChainIdsUsed = 5; // we will have 5 fingers
        t_ChainSettings.hand.fingerChainIds[0] = 1;  // links to the other chains we will define further down
        t_ChainSettings.hand.fingerChainIds[1] = 2;
        t_ChainSettings.hand.fingerChainIds[2] = 3;
        t_ChainSettings.hand.fingerChainIds[3] = 4;
        t_ChainSettings.hand.fingerChainIds[4] = 5;

        ChainSetup t_Chain;
        ChainSetup_Init(&t_Chain);
        t_Chain.id = 0; // Every ID needs to be unique per chain in a skeleton.
        t_Chain.type = ChainType::ChainType_Hand;
        t_Chain.dataType = ChainType::ChainType_Hand;
        t_Chain.side = Side::Side_Left;
        t_Chain.dataIndex = 0;
        t_Chain.nodeIdCount = 1;
        t_Chain.nodeIds[0] = 0; // this links to the hand node created in the SetupHandNodes
        t_Chain.settings = t_ChainSettings;

        SDKReturnCode t_Res = CoreSdk_AddChainToSkeletonSetup(p_SklIndex, t_Chain);
        if (t_Res != SDKReturnCode::SDKReturnCode_Success)
        {
            yError() << ManusGlove_LogPrefix << "Failed to Add Chain To Skeleton Setup.";
            return false;
        }
    }

    // Add the 5 finger chains
    const ChainType t_FingerTypes[5] = {ChainType::ChainType_FingerThumb,
                                        ChainType::ChainType_FingerIndex,
                                        ChainType::ChainType_FingerMiddle,
                                        ChainType::ChainType_FingerRing,
                                        ChainType::ChainType_FingerPinky};
    for (int i = 0; i < 5; i++)
    {
        ChainSettings t_ChainSettings;
        ChainSettings_Init(&t_ChainSettings);
        t_ChainSettings.usedSettings = t_FingerTypes[i];
        t_ChainSettings.finger.handChainId = 0; // This links to the wrist chain above.
        // This identifies the metacarpal bone, if none exists, or the chain is a thumb it should be set to -1.
        // The metacarpal bone should not be part of the finger chain, unless you are defining a thumb which does need it.
        t_ChainSettings.finger.metacarpalBoneId = -1;
        t_ChainSettings.finger.useLeafAtEnd = false; // this is set to true if there is a leaf bone to the tip of the finger.
        ChainSetup t_Chain;
        ChainSetup_Init(&t_Chain);
        t_Chain.id = i + 1; // Every ID needs to be unique per chain in a skeleton.
        t_Chain.type = t_FingerTypes[i];
        t_Chain.dataType = t_FingerTypes[i];
        t_Chain.side = Side::Side_Left;
        t_Chain.dataIndex = 0;
        if (i == 0) // Thumb
        {
            t_Chain.nodeIdCount = 4; // The amount of node id's used in the array
            t_Chain.nodeIds[0] = 1;  // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[1] = 2;  // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[2] = 3;  // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[3] = 4;  // this links to the hand node created in the SetupHandNodes
        }
        else // All other fingers
        {
            t_Chain.nodeIdCount = 4;          // The amount of node id's used in the array
            t_Chain.nodeIds[0] = (i * 4) + 1; // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[1] = (i * 4) + 2; // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[2] = (i * 4) + 3; // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[3] = (i * 4) + 4; // this links to the hand node created in the SetupHandNodes
        }
        t_Chain.settings = t_ChainSettings;

        SDKReturnCode t_Res = CoreSdk_AddChainToSkeletonSetup(p_SklIndex, t_Chain);
        if (t_Res != SDKReturnCode::SDKReturnCode_Success)
        {
            return false;
        }
    }
    return true;
}

bool ManusGloveHelper::LookingForHosts(bool p_hostType)
{
    if(!p_hostType)
    {
    yInfo() << ManusGlove_LogPrefix << "Finding a host running Core anywhere on the network...";
    }
    else
    {
    yInfo() << ManusGlove_LogPrefix << "Finding a host running Core locally on the network...";
    }

    // Underlying function will sleep for m_SecondsToFindHosts to allow servers to reply.
    if (CoreSdk_LookForHosts(m_SecondsToFindHosts, p_hostType) == SDKReturnCode::SDKReturnCode_SdkNotAvailable)
    {
        yError() << ManusGlove_LogPrefix << "Failed to look for hosts. Sdk Not Available";
        return false;
    }

    m_NumberOfHostsFound = 0;
    if (CoreSdk_GetNumberOfAvailableHostsFound(&m_NumberOfHostsFound) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to get the number of available hosts.";
        return false;
    }

    if (m_NumberOfHostsFound == 0)
    {
        yWarning() << ManusGlove_LogPrefix << "No hosts found.";
    }
    else
    {
        yInfo() << ManusGlove_LogPrefix << "Found " << m_NumberOfHostsFound << " available hosts.";
    }

    m_AvailableHosts.reset(new ManusHost[m_NumberOfHostsFound]);
    if (CoreSdk_GetAvailableHostsFound(m_AvailableHosts.get(), m_NumberOfHostsFound) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to get the available hosts.";
        return false;
    }

    return true;
}

bool ManusGloveHelper::ConnectingToCore()
{
    yInfo() << ManusGlove_LogPrefix << "Found the following hosts: ";

    // Note: only 10 hosts are shown, to match the number of number keys, for easy selection.
    for (unsigned int t_HostNumber = 0; t_HostNumber < 10 && t_HostNumber < m_NumberOfHostsFound; t_HostNumber++)
    {
        yInfo() << ManusGlove_LogPrefix << "[ " << t_HostNumber << "] "<<
                " hostname: " << m_AvailableHosts[t_HostNumber].hostName <<
                " IP address: " << m_AvailableHosts[t_HostNumber].ipAddress <<
                " Version " << m_AvailableHosts[t_HostNumber].manusCoreVersion.major << "." <<
                m_AvailableHosts[t_HostNumber].manusCoreVersion.minor << "." <<
                m_AvailableHosts[t_HostNumber].manusCoreVersion.patch;

        m_HostToConnectTo = t_HostNumber;
    }

    SDKReturnCode t_ConnectResult = SDKReturnCode::SDKReturnCode_Error;

    t_ConnectResult = CoreSdk_ConnectToHost(m_AvailableHosts[m_HostToConnectTo]);

    if (t_ConnectResult == SDKReturnCode::SDKReturnCode_NotConnected)
    {
        yWarning() << ManusGlove_LogPrefix << "The host and SDK are not connected.";
        return true; // Differentiating between error and no connect
    }
    if (t_ConnectResult != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to connect the host and SDK.";
        return false;
    }

    // Note: a log message from somewhere in the SDK during the connection process can cause text
    // to permanently turn green after this step. Adding a sleep here of 2+ seconds "fixes" the
    // issue. It seems to be caused by a threading issue somewhere, resulting in a log call being
    // interrupted while it is printing the green [info] text. The log output then gets stuck in
    // green mode.

    return true;
}

bool ManusGloveHelper::UpdateBeforeDisplayingData() //TODO: FailedUpdate??
{
    m_SkeletonMutex.lock();
    if (m_NextSkeleton != nullptr)
    {
        if (m_Skeleton != nullptr)//TODO: Can be removed!
            delete m_Skeleton;
        m_Skeleton = m_NextSkeleton;
        m_NextSkeleton = nullptr;
    }
    m_SkeletonMutex.unlock();

    m_LandscapeMutex.lock();
    if (m_NewLandscape != nullptr)
    {
        if (m_Landscape != nullptr)
        {
            delete m_Landscape;
        }
        m_Landscape = m_NewLandscape;
        m_NewLandscape = nullptr;
        m_GestureLandscapeData.swap(m_NewGestureLandscapeData);
    }
    m_LandscapeMutex.unlock();

    m_FirstLeftGloveID = 0;
    m_FirstRightGloveID = 0;
    if (m_Landscape == nullptr)
        return true;
    for (size_t i = 0; i < m_Landscape->gloveDevices.gloveCount; i++)
    {
        if (m_FirstLeftGloveID == 0 && m_Landscape->gloveDevices.gloves[i].side == Side::Side_Left)
        {
            m_FirstLeftGloveID = m_Landscape->gloveDevices.gloves[i].id;
            continue;
        }
        if (m_FirstRightGloveID == 0 && m_Landscape->gloveDevices.gloves[i].side == Side::Side_Right)
        {
            m_FirstRightGloveID = m_Landscape->gloveDevices.gloves[i].id;
            continue;
        }
    }

    return true;
}

bool ManusGloveHelper::DisplayingData()
{
    DisplayingDataGlove();
    PrintSystemMessage();

    return true;
}

bool ManusGloveHelper::DisplayingDataGlove()
{
    yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << "Glove Data: ";
    PrintErgonomicsData();

    return true;
}

void ManusGloveHelper::PrintSystemMessage()
{
    m_SystemMessageMutex.lock();
    yInfo() << ManusGlove_LogPrefix << "Received System data: " << m_SystemMessage << " code: " << m_SystemMessageCode;
    m_SystemMessageMutex.unlock();
}

void ManusGloveHelper::PrintHandErgoData(ErgonomicsData &p_ErgoData, bool p_Left)
{
    const std::string t_FingerNames[NUM_FINGERS_ON_HAND] = {"[thumb] ", "[index] ", "[middle]", "[ring]  ", "[pinky] "};
    const std::string t_FingerJointNames[NUM_FINGERS_ON_HAND] = {"mcp", "pip", "dip"};
    const std::string t_ThumbJointNames[NUM_FINGERS_ON_HAND] = {"cmc", "mcp", "ip "};

    t_DataOffset = 0;
    if (!p_Left)
        t_DataOffset = 20;

    const std::string *t_JointNames = t_ThumbJointNames;
    for (unsigned int t_FingerNumber = 0; t_FingerNumber < NUM_FINGERS_ON_HAND; t_FingerNumber++)
    {
        yInfo() << ManusGlove_LogPrefix << t_FingerNames[t_FingerNumber] << " " << t_JointNames[0] << " spread: " << 
        RoundFloatValue(p_ErgoData.data[t_DataOffset], 2) << " " << t_JointNames[0] << " stretch: " << 
        RoundFloatValue(p_ErgoData.data[t_DataOffset + 1], 2) << " " << t_JointNames[1] << " stretch: " << 
        RoundFloatValue(p_ErgoData.data[t_DataOffset + 2], 2) << " " << t_JointNames[2] << " stretch: " << 
        RoundFloatValue(p_ErgoData.data[t_DataOffset + 3], 2);

        t_JointNames = t_FingerJointNames;
        t_DataOffset += 4;
    }
}

void ManusGloveHelper::PrintErgonomicsData()
{
    yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " -- Left Glove -- " << m_FirstLeftGloveID;
    if (m_LeftGloveErgoData.id == m_FirstLeftGloveID)
    {
        PrintHandErgoData(m_LeftGloveErgoData, true);
    }

    else
    {
        yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " ...No Data...";
    }

    yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " -- Right Glove -- " << m_FirstRightGloveID;
    if (m_RightGloveErgoData.id == m_FirstRightGloveID)
    {
        PrintHandErgoData(m_RightGloveErgoData, false);
    }
    else
    {
        yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " ...No Data...";
    }

}

void ManusGloveHelper::PrintDongleData()
{
    // get a dongle id
    uint32_t t_DongleCount = 0;
    if (CoreSdk_GetNumberOfDongles(&t_DongleCount) != SDKReturnCode::SDKReturnCode_Success)
        return;
    if (t_DongleCount == 0)
        return; // we got no gloves to work on anyway!

    uint32_t *t_DongleIds = new uint32_t[t_DongleCount]();
    if (CoreSdk_GetDongleIds(t_DongleIds, t_DongleCount) != SDKReturnCode::SDKReturnCode_Success)
        return;

    DongleLandscapeData t_DongleData;

    for (uint32_t i = 0; i < t_DongleCount; i++)
    {
        SDKReturnCode t_Result = CoreSdk_GetDataForDongle(t_DongleIds[i], &t_DongleData);
        yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " -- Dongle -- " << t_DongleData.id;

        if (t_Result == SDKReturnCode::SDKReturnCode_Success)
        {
            yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " Type: "<< ConvertDeviceClassTypeToString(t_DongleData.classType) << "-" << ConvertDeviceFamilyTypeToString(t_DongleData.familyType);

            yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " License: {}" << t_DongleData.licenseType;
        }
        else
        {
            yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " ...No Data...";
        }
    }
}

// ==============================================
//                  Callbacks
// ==============================================
void ManusGloveHelper::OnConnectedCallback(const ManusHost *const p_Host)
{
    yInfo() << s_Instance->ManusGlove_LogPrefix << "Connected to Manus Core.";

    // No need to initialize these as they get filled in the CoreSdk_GetVersionsAndCheckCompatibility
    ManusVersion t_SdkVersion;
    ManusVersion t_CoreVersion;
    bool t_IsCompatible;

    const SDKReturnCode t_Result = CoreSdk_GetVersionsAndCheckCompatibility(&t_SdkVersion, &t_CoreVersion, &t_IsCompatible);

    if (t_Result == SDKReturnCode::SDKReturnCode_Success)
    {
        const std::string t_Versions = "Sdk version : " + std::string(t_SdkVersion.versionInfo) + ", Core version : " + std::string(t_CoreVersion.versionInfo) + ".";

        if (t_IsCompatible)
        {
            yInfo() << s_Instance->ManusGlove_LogPrefix << "Versions are compatible.{}" << t_Versions;
        }
        else
        {
            yWarning() << s_Instance->ManusGlove_LogPrefix << "Versions are not compatible with each other.";
        }
    }
    else
    {
        yError() << s_Instance->ManusGlove_LogPrefix << "Failed to get the versions from the SDK.";
    }
    uint32_t t_SessionId;
    const SDKReturnCode t_SessionIdResult = CoreSdk_GetSessionId(&t_SessionId);
    if (t_SessionIdResult == SDKReturnCode::SDKReturnCode_Success && t_SessionId != 0)
    {
        yInfo() << s_Instance->ManusGlove_LogPrefix << "Session Id: {}" << t_SessionId;
        s_Instance->m_SessionId = t_SessionId;
    }
    else
    {
        yInfo() << s_Instance->ManusGlove_LogPrefix << "Failed to get the Session ID from Core. The value returned was{}." << t_SessionIdResult;
    }

    ManusHost t_Host(*p_Host);
    s_Instance->m_Host = std::make_unique<ManusHost>(t_Host);
}

void ManusGloveHelper::OnDisconnectedCallback(const ManusHost* const p_Host)
{
    yInfo() << s_Instance->ManusGlove_LogPrefix << "Disconnected from Manus Core.";

    s_Instance->m_TimeSinceLastDisconnect = std::chrono::high_resolution_clock::now();

    ManusHost t_Host(*p_Host);
    s_Instance->m_Host = std::make_unique<ManusHost>(t_Host);
}

void ManusGloveHelper::OnSkeletonStreamCallback(const SkeletonStreamInfo* const p_SkeletonStreamInfo)
{
    if (s_Instance)
    {
        ClientSkeletonCollection *t_NxtClientSkeleton = new ClientSkeletonCollection();
        t_NxtClientSkeleton->skeletons.resize(p_SkeletonStreamInfo->skeletonsCount);

        for (uint32_t i = 0; i < p_SkeletonStreamInfo->skeletonsCount; i++)
        {
            CoreSdk_GetSkeletonInfo(i, &t_NxtClientSkeleton->skeletons[i].info);
            t_NxtClientSkeleton->skeletons[i].nodes.resize(t_NxtClientSkeleton->skeletons[i].info.nodesCount);
            t_NxtClientSkeleton->skeletons[i].info.publishTime = p_SkeletonStreamInfo->publishTime;
            CoreSdk_GetSkeletonData(i, t_NxtClientSkeleton->skeletons[i].nodes.data(), t_NxtClientSkeleton->skeletons[i].info.nodesCount);
        }
        s_Instance->m_SkeletonMutex.lock();
        if (s_Instance->m_NextSkeleton != nullptr)
            delete s_Instance->m_NextSkeleton;
        s_Instance->m_NextSkeleton = t_NxtClientSkeleton;
        s_Instance->m_SkeletonMutex.unlock();
    }
}

void ManusGloveHelper::OnLandscapeCallback(const Landscape* const p_Landscape)
{
    if (s_Instance == nullptr)
        return;

    Landscape *t_Landscape = new Landscape(*p_Landscape);
    s_Instance->m_LandscapeMutex.lock();
    if (s_Instance->m_NewLandscape != nullptr)
        delete s_Instance->m_NewLandscape;
    s_Instance->m_NewLandscape = t_Landscape;
    s_Instance->m_NewGestureLandscapeData.resize(t_Landscape->gestureCount);
    CoreSdk_GetGestureLandscapeData(s_Instance->m_NewGestureLandscapeData.data(), (uint32_t)s_Instance->m_NewGestureLandscapeData.size());
    s_Instance->m_LandscapeMutex.unlock();
}

void ManusGloveHelper::OnSystemCallback(const SystemMessage* const p_SystemMessage)
{
    if (s_Instance)
    {
        s_Instance->m_SystemMessageMutex.lock();

        switch (p_SystemMessage->type)
        {
        case SystemMessageType::SystemMessageType_TemporarySkeletonModified:
            // if the message was triggered by a temporary skeleton being modified then save the skeleton index,
            // this information will be used to get and load the skeleton into core
            s_Instance->m_ModifiedSkeletonIndex = p_SystemMessage->infoUInt;
            break;
        default:
            s_Instance->m_SystemMessageCode = p_SystemMessage->type;
            s_Instance->m_SystemMessage = p_SystemMessage->infoString;
            break;
        }
        s_Instance->m_SystemMessageMutex.unlock();
    }
}

void ManusGloveHelper::OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo)
{
    if (s_Instance)
    {
        for (uint32_t i = 0; i < p_Ergo->dataCount; i++)
        {
            if (p_Ergo->data[i].isUserID)
                continue;

            ErgonomicsData *t_Ergo = nullptr;
            if (p_Ergo->data[i].id == s_Instance->m_FirstLeftGloveID)
            {
                t_Ergo = &s_Instance->m_LeftGloveErgoData;
            }
            if (p_Ergo->data[i].id == s_Instance->m_FirstRightGloveID)
            {
                t_Ergo = &s_Instance->m_RightGloveErgoData;
            }
            if (t_Ergo == nullptr)
                continue;
            CoreSdk_GetTimestampInfo(p_Ergo->publishTime, &s_Instance->m_ErgoTimestampInfo);
            t_Ergo->id = p_Ergo->data[i].id;
            t_Ergo->isUserID = p_Ergo->data[i].isUserID;
            for (int j = 0; j < ErgonomicsDataType::ErgonomicsDataType_MAX_SIZE; j++)
            {
                t_Ergo->data[j] = p_Ergo->data[i].data[j];// TODO: HumanJointState
            }
        }
    }
}

// ==============================================
//                  Utility
// ==============================================

float ManusGloveHelper::RoundFloatValue(float p_Value, int p_NumDecimalsToKeep)
{
    // Since C++11, powf is supposed to be declared in <cmath>.
    // Unfortunately, gcc decided to be non-compliant on this for no apparent
    // reason, so now we have to do this.
    // https://stackoverflow.com/questions/5483930/powf-is-not-a-member-of-std
    float t_Power = static_cast<float>(std::pow(
        10.0,
        static_cast<double>(p_NumDecimalsToKeep)));
    return std::round(p_Value * t_Power) / t_Power;
}

std::string ManusGloveHelper::ConvertDeviceClassTypeToString(DeviceClassType p_Type)
{
    switch (p_Type)
    {
    case DeviceClassType_Dongle:
        return "Dongle";
    case DeviceClassType_Glove:
        return "Glove";
    case DeviceClassType_Glongle:
        return "Glongle (Glove Dongle)";
    default:
        return "Unknown";
    }
}

std::string ManusGloveHelper::ConvertDeviceFamilyTypeToString(DeviceFamilyType p_Type)
{
    switch (p_Type)
    {
    case DeviceFamilyType_Prime1:
        return "Prime 1";
    case DeviceFamilyType_Prime2:
        return "Prime 2";
    case DeviceFamilyType_PrimeX:
        return "Prime X";
    case DeviceFamilyType_Quantum:
        return "Quantum";
    case DeviceFamilyType_Prime3:
        return "Prime 3";
    case DeviceFamilyType_Virtual:
        return "Virtual";
    default:
        return "Unknown";
    }
}

bool ManusGloveHelper::CopyString(char *const p_Target, const size_t p_MaxLengthThatWillFitInTarget, const std::string &p_Source)
{
    const errno_t t_CopyResult = strcpy_s(
        p_Target,
        p_MaxLengthThatWillFitInTarget,
        p_Source.c_str());
    if (t_CopyResult != 0)
    {
        yError() << ManusGlove_LogPrefix <<"Copying the string "<<p_Source.c_str()<< "resulted in the error " << t_CopyResult;
        return false;
    }
    return true;
}

bool ManusGloveHelper::getHandJointPosition(std::vector<double>& jointAngleList, bool p_handSide)
{

    if (!UpdateBeforeDisplayingData())
    {
        yError() << ManusGlove_LogPrefix << "Failed to update the joint position.";
        return false;
    }

    switch (p_handSide)
    {
    case false:
        j_DataOffset = 0;
        t_DataOffset = 0;
        for (unsigned int t_FingerNumber = 0; t_FingerNumber < NUM_FINGERS_ON_HAND; t_FingerNumber++)
        {
            jointAngleList[t_FingerNumber + j_DataOffset] = RoundFloatValue(s_Instance->m_LeftGloveErgoData.data[t_DataOffset], 2);
            jointAngleList[t_FingerNumber + j_DataOffset + 1] = RoundFloatValue(s_Instance->m_LeftGloveErgoData.data[t_DataOffset + 1], 2);
            jointAngleList[t_FingerNumber + j_DataOffset + 2] = RoundFloatValue(s_Instance->m_LeftGloveErgoData.data[t_DataOffset + 2], 2);
            jointAngleList[t_FingerNumber + j_DataOffset + 3] = RoundFloatValue(s_Instance->m_LeftGloveErgoData.data[t_DataOffset + 3], 2);

            t_DataOffset += 4;
            j_DataOffset += 3;
        }
        if (*t_counter < 3)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::fill(jointAngleList.begin(), jointAngleList.end(), 0);
        }
        (*t_counter)++;
        break;
    case true:
        j_DataOffset = 0;
        t_DataOffset = 20;
        for (unsigned int t_FingerNumber = 0; t_FingerNumber < NUM_FINGERS_ON_HAND; t_FingerNumber++)
        {
            jointAngleList[t_FingerNumber + j_DataOffset] = RoundFloatValue(s_Instance->m_RightGloveErgoData.data[t_DataOffset], 2);
            jointAngleList[t_FingerNumber + j_DataOffset + 1] = RoundFloatValue(s_Instance->m_RightGloveErgoData.data[t_DataOffset + 1], 2);
            jointAngleList[t_FingerNumber + j_DataOffset + 2] = RoundFloatValue(s_Instance->m_RightGloveErgoData.data[t_DataOffset + 2], 2);
            jointAngleList[t_FingerNumber + j_DataOffset + 3] = RoundFloatValue(s_Instance->m_RightGloveErgoData.data[t_DataOffset + 3], 2);

            t_DataOffset += 4;
            j_DataOffset += 3;
        }
        if (*t_counter < 3)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::fill(jointAngleList.begin(), jointAngleList.end(), 0);
        }
        (*t_counter)++;
        break;
    }

    return true;
}
