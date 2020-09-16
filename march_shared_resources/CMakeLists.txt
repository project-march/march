cmake_minimum_required(VERSION 3.0.2)
project(march_shared_resources)

find_package(catkin REQUIRED COMPONENTS
    control_msgs
    geometry_msgs
    message_generation
    std_msgs
    trajectory_msgs
)

add_message_files(
    FILES
    AfterLimitJointCommand.msg
    Alive.msg
    CurrentGait.msg
    CurrentState.msg
    Error.msg
    GaitInstruction.msg
    GaitInstructionResponse.msg
    HighVoltageNet.msg
    ImcState.msg
    JointValues.msg
    LowVoltageNet.msg
    PowerDistributionBoardState.msg
    PressureSole.msg
)

add_service_files(
    FILES
    CapturePointPose.srv
    ContainsGait.srv
    PossibleGaits.srv
    SetGaitVersion.srv
)

generate_messages(
    DEPENDENCIES
    control_msgs
    std_msgs
    trajectory_msgs
)

catkin_package(
    CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    message_runtime
    std_msgs
    trajectory_msgs
)
