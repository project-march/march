cmake_minimum_required(VERSION 3.0.2)
project(march_shared_resources)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    control_msgs
    message_generation
    std_msgs
    trajectory_msgs
)

add_message_files(
    FILES
    AfterLimitJointCommand.msg
    Alive.msg
    Error.msg
    GaitInstruction.msg
    GaitInstructionResponse.msg
    HighVoltageNet.msg
    ImcState.msg
    JointInertia.msg
    JointValues.msg
    LowVoltageNet.msg
    PowerDistributionBoardState.msg
    PressureSole.msg
)

add_service_files(
    FILES
    ContainsGait.srv
    CurrentState.srv
    PossibleGaits.srv
    SetGaitVersion.srv
)

add_action_files(
    DIRECTORY action
    FILES
    Gait.action
    GaitName.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    control_msgs
    std_msgs
    trajectory_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    control_msgs
    message_runtime
    std_msgs
    trajectory_msgs
)
