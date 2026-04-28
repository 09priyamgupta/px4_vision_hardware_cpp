#include "px4_vision_hardware_cpp/mission_executor.hpp"

// --------------------- Constructor ---------------------
MissionExecutor::MissionExecutor(px4_ros2::ModeBase & owned_mode)
: ModeExecutorBase(owned_mode.node(), px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
  node_(owned_mode.node())
{
    // The executor shares the same ROS 2 node as the mode it owns.
    if (!node_.has_parameter("battery_failsafe_pct")) 
    {
        node_.declare_parameter("battery_failsafe_pct", 15.0);
    }
    double battery_failsafe = node_.get_parameter("battery_failsafe_pct").as_double();
    
    RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Initialized. Battery Failsafe set to: %.1f%%", battery_failsafe);
}


// --------------------- Mode Lifecycle Callbacks ---------------------
void MissionExecutor::onActivate()
{
    RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Mission Activated! Starting mission sequence...");

    // Start the state machine starting with the first state: TakingOff
    runState(State::VisionLanding, px4_ros2::Result::Success);
}


void MissionExecutor::onDeactivate(DeactivateReason reason)
{
    // This triggers automatically if the pilot moves the RC sticks to switch mode,
    // or if the user clicks "Switch Mode" in QGC, or if a failsafe triggers a mode switch
    RCLCPP_WARN(node_.get_logger(), "[EXECUTOR] Mission Deactivated! Reason: %i. Handing over control back to the pilot...", (int)reason);
}


// -----------------------------------------------------------------------------
// --------------------- State Machine Implementation ---------------------
// -----------------------------------------------------------------------------
void MissionExecutor::runState(State state, px4_ros2::Result previous_result)
{
    // Safety Check: If the previous state failed, abort sequence
    if (previous_result != px4_ros2::Result::Success)
    {
        RCLCPP_ERROR(node_.get_logger(), "[EXECUTOR] State %i failed: %s. Aborting mission sequence!", 
                    (int)state, px4_ros2::resultToString(previous_result));
        return;
    }

    switch (state)
    {
        // case State::TakingOff:
        //     RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] State: Taking Off");
            
        //     takeoff([this](px4_ros2::Result result)
        //     {
        //         runState(State::SearchTarget, result);
        //     }); 
        //     break;

        // case State::SearchTarget:
        //     RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] State: Searching for Target Phase");
            
        //     // We will add the search target logic based on GPS in the future
        //     runState(State::VisionLanding, px4_ros2::Result::Success);
        //     break;

        case State::VisionLanding:
            RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Activating Custom Vision Landing Mode");

            // Stays in this mode/state until the custom mode finishes and returns Success/failure
            scheduleMode(ownedMode().id(), [this](px4_ros2::Result result)
            {
                runState(State::Land, result);
            });
            break;

        case State::Land:
            RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] State: Landing...");

            land([this](px4_ros2::Result result)
            {
                runState(State::WaitUntilDisarmed, result);
            });
            break;

        case State::WaitUntilDisarmed:
            RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] -> Waiting for Disarm...");

            // Halts the executor until the flight controller confirms props are stopped
            waitUntilDisarmed([this](px4_ros2::Result result) 
            {
                RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Mission Complete! (%s)", px4_ros2::resultToString(result));
            });
            break;
    }
}
