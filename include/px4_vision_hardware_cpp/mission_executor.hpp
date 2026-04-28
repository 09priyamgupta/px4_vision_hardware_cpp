#ifndef MISSION_EXECUTOR_HPP
#define MISSION_EXECUTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/mode.hpp>


class MissionExecutor : public px4_ros2::ModeExecutorBase
{
    public:
        // The MissionExecutor constructor requires the specific Mode "owned mode"
        // The "owned mode" is the mode that this executor will execute when the mode is active
        // It stays incharge until the user switches mode (by RC or QGC), or a failsafe triggers a mode switch
        MissionExecutor(px4_ros2::ModeBase & owned_mode);

        // Our high-level mission states
        enum class State
        {
            // TakingOff,
            // SearchTarget,
            VisionLanding,
            Land,
            WaitUntilDisarmed
        };

        // Overrides from ModeExecutorBase, this is where the main logic of the mission will be implemented
        void onActivate() override;
        void onDeactivate(DeactivateReason reason) override;

    private:
        // The main state machine runner, this will be called periodically by the ModeExecutorBase
        void runState(State state, px4_ros2::Result previous_result);

        rclcpp::Node & node_;
};

#endif // MISSION_EXECUTOR_HPP