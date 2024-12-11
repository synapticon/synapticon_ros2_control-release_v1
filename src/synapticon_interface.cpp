#include "synapticon_ros2_control/synapticon_interface.hpp"
#include "synapticon_ros2_control/soem_utilities.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace synapticon_ros2_control {
namespace {
constexpr char LOG_NAME[] = "synapticon_ros2_control";
constexpr char ETHERCAT_INTERFACE[] = "eno0";
} // namespace

hardware_interface::CallbackReturn SynapticonSystemInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
      rclcpp::get_logger("synapticon_interface"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  hw_start_sec_ = 1.0;
  hw_stop_sec_ = 1.0;
  hw_slowdown_ = 1.0;
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code
  hw_states_positions_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(),
                               std::numeric_limits<double>::quiet_NaN());
  hw_states_accelerations_.resize(info_.joints.size(),
                                  std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(),
                                 std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(),
                                    std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(info_.joints.size(), integration_level_t::POSITION);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu command interfaces. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_ACCELERATION)) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' has %s command interface. Expected %s, %s, or %s.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_ACCELERATION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s'has %zu state interfaces. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_ACCELERATION)) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %s state interface. Expected %s, %s, or %s.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY,
                   hardware_interface::HW_IF_ACCELERATION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // A thread to handle ethercat errors
  osal_thread_create(&ecat_error_thread_, 128000, (void *)&ecatcheck,
                     (void *)&ctime);

  // Ethercat initialization
  if (!ec_init(ETHERCAT_INTERFACE)) {
    RCLCPP_FATAL(get_logger(),
                 "Error during initialization of ethercat interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (ec_config_init(FALSE) <= 0) {
    RCLCPP_FATAL(get_logger(), "No ethercat slaves found!");
    ec_close();
    return hardware_interface::CallbackReturn::ERROR;
  }
  ec_config_map(&io_map_);
  ec_configdc();
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  // Request operational state for all slaves
  soem_globals::kExpectedWkc =
      (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  // send one valid process data to make outputs in slaves happy
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  // request OP state for all slaves
  ec_writestate(0);
  size_t chk = 200;
  // wait for all slaves to reach OP state
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state != EC_STATE_OPERATIONAL)
  {
    RCLCPP_FATAL(get_logger(), "An ethercat slave failed to reach OPERATIONAL state");
    return hardware_interface::CallbackReturn::ERROR;
  }
  soem_globals::kInOp = true;

  // Connect struct pointers to I/O
  in_somanet_1_ = (InSomanet50t*)ec_slave[0].inputs;
  out_somanet_1_ = (OutSomanet50t*)ec_slave[0].outputs;

  RCLCPP_FATAL(get_logger(), "Ethercat initialization successful!");
  RCLCPP_FATAL(get_logger(), "Ethercat initialization successful!");
  RCLCPP_FATAL(get_logger(), "Ethercat initialization successful!");
  RCLCPP_FATAL(get_logger(), "Ethercat initialization successful!");
  RCLCPP_FATAL(get_logger(), "Ethercat initialization successful!");
  RCLCPP_FATAL(get_logger(), "Ethercat initialization successful!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SynapticonSystemInterface::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  // Prepare for new command modes
  std::vector<integration_level_t> new_modes = {};
  for (std::string key : start_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key ==
          info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        new_modes.push_back(integration_level_t::POSITION);
      }
      if (key ==
          info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_modes.push_back(integration_level_t::VELOCITY);
      }
      if (key ==
          info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION) {
        new_modes.push_back(integration_level_t::ACCELERATION);
      }
    }
  }
  // // Example criteria: All joints must be given new command mode at the same
  // time if (new_modes.size() != info_.joints.size())
  // {
  //   return hardware_interface::return_type::ERROR;
  // }
  // // Example criteria: All joints must have the same command mode
  // if (!std::all_of(
  //       new_modes.begin() + 1, new_modes.end(),
  //       [&](integration_level_t mode) { return mode == new_modes[0]; }))
  // {
  //   return hardware_interface::return_type::ERROR;
  // }

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        hw_commands_velocities_[i] = 0;
        hw_commands_accelerations_[i] = 0;
        control_level_[i] =
            integration_level_t::UNDEFINED; // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    if (control_level_[i] != integration_level_t::UNDEFINED) {
      // Something else is using the joint! Abort!
      return hardware_interface::return_type::ERROR;
    }
    control_level_[i] = new_modes[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn SynapticonSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(get_logger(), "Activating... please wait...");

  for (int i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  // Set some default values
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++) {
    if (std::isnan(hw_states_positions_[i])) {
      hw_states_positions_[i] = 0;
    }
    if (std::isnan(hw_states_velocities_[i])) {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_accelerations_[i])) {
      hw_states_accelerations_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i])) {
      hw_commands_positions_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i])) {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_accelerations_[i])) {
      hw_commands_accelerations_[i] = 0;
    }
    control_level_[i] = integration_level_t::UNDEFINED;
  }

  RCLCPP_INFO(get_logger(), "System successfully activated! Control level: %u",
              control_level_[0]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SynapticonSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(get_logger(), "Deactivating... please wait...");

  for (int i = 0; i < hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SynapticonSystemInterface::read(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration &period) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  // std::stringstream ss;
  // ss << "Reading states:";
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++) {
    switch (control_level_[i]) {
    case integration_level_t::UNDEFINED:
      // RCLCPP_INFO(get_logger(), "Nothing is using the hardware interface!");
      return hardware_interface::return_type::OK;
      break;
    case integration_level_t::POSITION:
      hw_states_accelerations_[i] = 0;
      hw_states_velocities_[i] = 0;
      hw_states_positions_[i] +=
          (hw_commands_positions_[i] - hw_states_positions_[i]) / hw_slowdown_;
      break;
    case integration_level_t::VELOCITY:
      hw_states_accelerations_[i] = 0;
      hw_states_velocities_[i] = hw_commands_velocities_[i];
      hw_states_positions_[i] +=
          (hw_states_velocities_[i] * period.seconds()) / hw_slowdown_;
      break;
    case integration_level_t::ACCELERATION:
      hw_states_accelerations_[i] = hw_commands_accelerations_[i];
      hw_states_velocities_[i] +=
          (hw_states_accelerations_[i] * period.seconds()) / hw_slowdown_;
      hw_states_positions_[i] +=
          (hw_states_velocities_[i] * period.seconds()) / hw_slowdown_;
      break;
    }
    // ss << std::fixed << std::setprecision(2) << std::endl
    //    << "\t"
    //    << "pos: " << hw_states_positions_[i]
    //    << ", vel: " << hw_states_velocities_[i]
    //    << ", acc: " << hw_states_accelerations_[i] << " for joint " << i;
  }
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s",
  // ss.str().c_str()); END: This part here is for exemplary purposes - Please
  // do not copy to your production code
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SynapticonSystemInterface::write(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  // // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code std::stringstream ss; ss << "Writing commands:"; for
  // (std::size_t i = 0; i < hw_commands_positions_.size(); i++)
  // {
  //   // Simulate sending commands to the hardware
  //   ss << std::fixed << std::setprecision(2) << std::endl
  //      << "\t"
  //      << "command pos: " << hw_commands_positions_[i] << ", vel: " <<
  //      hw_commands_velocities_[i]
  //      << ", acc: " << hw_commands_accelerations_[i] << " for joint " << i
  //      << ", control lvl: " << static_cast<int>(control_level_[i]);
  // }
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s",
  // ss.str().c_str());
  // // END: This part here is for exemplary purposes - Please do not copy to
  // your production code

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
SynapticonSystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,
        &hw_states_accelerations_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SynapticonSystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,
        &hw_commands_accelerations_[i]));
  }
  return command_interfaces;
}

SynapticonSystemInterface::~SynapticonSystemInterface() {
  // Close the ethercat connection
  ec_close();
}

} // namespace synapticon_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(synapticon_ros2_control::SynapticonSystemInterface,
                       hardware_interface::SystemInterface)
