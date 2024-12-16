#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "ethercat.h"

namespace synapticon_ros2_control {

class SynapticonSystemInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SynapticonSystemInterface)

  ~SynapticonSystemInterface();

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  /**
   * @brief Error checking. Typically runs in a separate thread.
   */
  OSAL_THREAD_FUNC ecatCheck(void *ptr);

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store the commands for the simulated robot
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_accelerations_;
  std::vector<double> hw_states_efforts_;

  // Enum defining current control level
  // TODO: enable position commands
  enum control_level_t : std::uint8_t {
    UNDEFINED = 0,
    VELOCITY = 1,
    EFFORT = 2, // aka torque
  };

  // Active control mode for each actuator
  std::vector<control_level_t> control_level_;

  // For SOEM
  OSAL_THREAD_HANDLE ecat_error_thread_;
  char io_map_[4096];

  // Somanet structs
  typedef struct {
    uint16_t Statusword;
    int8_t OpModeDisplay;
    int32_t PositionValue;
    int32_t VelocityValue;
    int16_t TorqueValue;
    uint16_t AnalogInput1;
    uint16_t AnalogInput2;
    uint16_t AnalogInput3;
    uint16_t AnalogInput4;
    uint32_t TuningStatus;
    uint32_t DigitalInputs;
    uint32_t UserMISO;
    uint32_t Timestamp;
    int32_t PositionDemandInternalValue;
    int32_t VelocityDemandValue;
    int16_t TorqueDemand;
  } InSomanet50t;

  typedef struct {
    uint16_t Controlword;
    int8_t OpMode;
    int16_t TargetTorque;
    int32_t TargetPosition;
    int32_t TargetVelocity;
    int16_t TorqueOffset;
    int32_t TuningCommand;
    int32_t PhysicalOutputs;
    int32_t BitMask;
    int32_t UserMOSI;
    int32_t VelocityOffset;
  } OutSomanet50t;

  std::vector<InSomanet50t *> in_somanet_1_;
  std::vector<OutSomanet50t *> out_somanet_1_;

  // For coordination between threads
  std::atomic<bool> in_operation_;
  volatile std::atomic<int> wkc_;
  std::atomic<int> expected_wkc_;
  std::atomic<bool> needlf_;
};

} // namespace synapticon_ros2_control
