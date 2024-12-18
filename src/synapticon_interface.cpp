#include "synapticon_ros2_control/synapticon_interface.hpp"

#include <chrono>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <inttypes.h>
#include <rclcpp/rclcpp.hpp>

#define EC_TIMEOUTMON 500

using namespace std::chrono_literals;

namespace synapticon_ros2_control {
namespace {
constexpr char LOG_NAME[] = "synapticon_ros2_control";
constexpr char ETHERCAT_INTERFACE[] = "eno0";
constexpr double DEG_TO_RAD = 0.0174533;
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

  hw_states_positions_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(),
                               std::numeric_limits<double>::quiet_NaN());
  hw_states_accelerations_.resize(info_.joints.size(),
                                  std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(),
                            std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(),
                                 std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(info_.joints.size(), control_level_t::UNDEFINED);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (!(joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %s command interface. Expected %s or %s.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY,
                   hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_ACCELERATION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' has %s state interface. Expected %s, %s, %s, or %s.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_ACCELERATION,
          hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // A thread to handle ethercat errors
  osal_thread_create(&ecat_error_thread_, 128000,
                     (void *)&SynapticonSystemInterface::ecatCheck,
                     (void *)&ctime);

  // Ethercat initialization
  int ec_init_status = ec_init(ETHERCAT_INTERFACE);
  if (ec_init_status <= 0) {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Error during initialization of ethercat interface: "
                            << ec_init_status);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (ec_config_init(false) <= 0) {
    RCLCPP_FATAL(get_logger(), "No ethercat slaves found!");
    ec_close();
    return hardware_interface::CallbackReturn::ERROR;
  }
  ec_config_map(&io_map_);
  ec_configdc();
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  // Request operational state for all slaves
  expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  // TODO: does this need to be vectorized?
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

  if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
    RCLCPP_FATAL(get_logger(),
                 "An ethercat slave failed to reach OPERATIONAL state");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Connect struct pointers to I/O
  for (size_t joint_idx = 0; joint_idx < info_.joints.size(); ++joint_idx) {
    in_somanet_1_.push_back((InSomanet50t *)ec_slave[joint_idx].inputs);
    out_somanet_1_.push_back((OutSomanet50t *)ec_slave[joint_idx].outputs);
  }

  // // Read encoder resolution
  // uint8_t encoder_source;
  // int size = sizeof(encoder_source);
  // ec_SDOread(1, 0x2012, 0x09, false, &size, &encoder_source, EC_TIMEOUTRXM);
  // size = sizeof(encoder_resolution_);
  // if (encoder_source == 1) {
  //   ec_SDOread(1, 0x2110, 0x03, false, &size, &encoder_resolution_,
  //              EC_TIMEOUTRXM);
  // } else if (encoder_source == 2) {
  //   ec_SDOread(1, 0x2112, 0x03, false, &size, &encoder_resolution_,
  //              EC_TIMEOUTRXM);
  // } else {
  //   std::cerr
  //       << "No encoder configured for position control. Terminating the program"
  //       << std::endl;
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SynapticonSystemInterface::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  // Prepare for new command modes
  std::vector<control_level_t> new_modes = {};
  for (std::string key : start_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      // TODO: add additional control modes
      if (key ==
          info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_modes.push_back(control_level_t::EFFORT);
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
  //       [&](control_level_t mode) { return mode == new_modes[0]; }))
  // {
  //   return hardware_interface::return_type::ERROR;
  // }

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        control_level_[i] = control_level_t::UNDEFINED; // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    if (control_level_[i] != control_level_t::UNDEFINED) {
      // Something else is using the joint! Abort!
      return hardware_interface::return_type::ERROR;
    }
    control_level_[i] = new_modes[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn SynapticonSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

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
    if (std::isnan(hw_states_efforts_[i])) {
      hw_states_efforts_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i])) {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_efforts_[i])) {
      hw_commands_efforts_[i] = 0;
    }
  }

  // Start the control loop, wait for it to reach normal operation mode
  somanet_control_thread_ = std::thread(&SynapticonSystemInterface::somanetCyclicLoop, this, std::ref(in_normal_op_mode_));
  while (!in_normal_op_mode_)
  {
    std::this_thread::sleep_for(10ms);
  }

  RCLCPP_INFO(get_logger(), "System successfully activated! Control level: %u",
              control_level_[0]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SynapticonSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  if (somanet_control_thread_ && somanet_control_thread_->joinable())
  {
    somanet_control_thread_->join();
  }

  for (std::size_t i = 0; i < hw_states_positions_.size(); i++) {
    control_level_[i] = control_level_t::UNDEFINED;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SynapticonSystemInterface::read(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) {

  // for (std::size_t i = 0; i < hw_states_positions_.size(); i++) {
  //   switch (control_level_[i]) {
  //   case control_level_t::UNDEFINED:
  //     // RCLCPP_INFO(get_logger(), "Nothing is using the hardware interface!");
  //     return hardware_interface::return_type::OK;
  //   // TODO: add additional control modes
  //   case control_level_t::EFFORT:
  //     {
  //       std::lock_guard<std::mutex> lock(in_somanet_mtx_);
  //       // InSomanet50t doesn't include acceleration
  //       hw_states_accelerations_[i] = 0;
  //       hw_states_velocities_[i] = in_somanet_1_[i]->VelocityValue;
  //       hw_states_positions_[i] = in_somanet_1_[i]->PositionValue;
  //       hw_states_efforts_[i] = in_somanet_1_[i]->TorqueValue;
  //       break;
  //     }
  //   default:
  //     // This should never happen
  //     RCLCPP_FATAL(get_logger(), "Unexpected control_level_t within read()");
  //     return hardware_interface::return_type::ERROR;
  //   }
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SynapticonSystemInterface::write(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  // This function doesn't do much.
  // It's taken care of in separate thread, somanet_control_thread_

  // for (std::size_t i = 0; i < hw_commands_efforts_.size(); ++i) {
  //   if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
  //     std::cerr << "A slave is not in operational state" << std::endl;
  //     return hardware_interface::return_type::ERROR;
  //   }
  // }

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
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &hw_states_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SynapticonSystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &hw_commands_efforts_[i]));
  }
  return command_interfaces;
}

SynapticonSystemInterface::~SynapticonSystemInterface() {
  // A flag to ecat_error_check_ thread
  in_normal_op_mode_ = false;

  // Close the ethercat connection
  ec_close();
}

OSAL_THREAD_FUNC SynapticonSystemInterface::ecatCheck(void * /*ptr*/) {
  int slave;
  uint8 currentgroup = 0;

  while (1) {
    if (in_normal_op_mode_ &&
        ((wkc_ < expected_wkc_) || ec_group[currentgroup].docheckstate)) {
      if (needlf_) {
        needlf_ = false;
        printf("\n");
      }
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = false;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == currentgroup) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = true;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n",
                   slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n",
                   slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = false;
              printf("MESSAGE : slave %d reconfigured\n", slave);
            }
          } else if (!ec_slave[slave].islost) {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = true;
              printf("ERROR : slave %d lost\n", slave);
            }
          }
        }
        if (ec_slave[slave].islost) {
          if (ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = false;
              printf("MESSAGE : slave %d recovered\n", slave);
            }
          } else {
            ec_slave[slave].islost = false;
            printf("MESSAGE : slave %d found\n", slave);
          }
        }
      }
      if (!ec_group[currentgroup].docheckstate)
        printf("OK : all slaves resumed OPERATIONAL.\n");
    }
    osal_usleep(10000);
  }
}

void SynapticonSystemInterface::somanetCyclicLoop(std::atomic<bool>& in_normal_op_mode)
{
  bool is_first_iteration = true;
  while (1)
  {
    {
      // std::lock_guard<std::mutex> lock_in(in_somanet_mtx_);

      ec_send_processdata();
      wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

      if (wkc_ >= expected_wkc_)
      {
        for (size_t joint_idx = 0; joint_idx < info_.joints.size(); ++joint_idx)
        {
          // Profile torque mode
          // TODO: other modes
          if (is_first_iteration)
          {
            is_first_iteration = false;
            out_somanet_1_[joint_idx]->OpMode = 4;
          }

          // Fault reset: Fault -> Swith on disabled, if the drive is in fault state
          if ((in_somanet_1_[joint_idx]->Statusword & 0b0000000001001111) == 0b0000000000001000)
              out_somanet_1_[joint_idx]->Controlword = 0b10000000;

          // Shutdown: Switch on disabled -> Ready to switch on
          else if ((in_somanet_1_[joint_idx]->Statusword & 0b0000000001001111) == 0b0000000001000000)
              out_somanet_1_[joint_idx]->Controlword = 0b00000110;

          // Switch on: Ready to switch on -> Switched on
          else if ((in_somanet_1_[joint_idx]->Statusword & 0b0000000001101111) == 0b0000000000100001)
              out_somanet_1_[joint_idx]->Controlword = 0b00000111;

          // Enable operation: Switched on -> Operation enabled
          else if ((in_somanet_1_[joint_idx]->Statusword & 0b0000000001101111) == 0b0000000000100011)
              out_somanet_1_[joint_idx]->Controlword = 0b00001111;

          // Sending torque command
          else if ((in_somanet_1_[joint_idx]->Statusword & 0b0000000001101111) == 0b0000000000100111)
          {
            in_normal_op_mode = true;
            if (control_level_[joint_idx] == control_level_t::EFFORT)
            {
              // TODO: lock hw_commands_efforts_[joint_idx] with a mutex and use that
              out_somanet_1_[joint_idx]->TargetTorque = 0;
            }
          }

          // printf(" Statusword 0: %X ,", in_somanet_1_[0]->Statusword);
          // printf(" Op Mode Display 0: %d ,", in_somanet_1_[0]->OpModeDisplay);
          // printf(" ActualPos 0: %" PRId32 " ,", in_somanet_1_[0]->PositionValue);
          // printf(" ActualVel 0: %" PRId32 " ,", in_somanet_1_[0]->VelocityValue);
          // printf(" DemandVel 0: %" PRId32 " ,",
          // in_somanet_1_[0]->VelocityDemandValue); printf(" ActualTorque 0: %" PRId32
          // " ,", in_somanet_1_[0]->TorqueValue); printf(" DemandTorque 0: %" PRId32 "
          // ,", in_somanet_1_[0]->TorqueDemand);

          // std::cerr << (int32_t)(in_somanet_1_[0]->PositionValue *
          // (double)(DEG_TO_RAD * 360.0 / encoder_resolution_)) << std::endl;
          // std::cerr << in_somanet_1_[0]->PositionValue << std::endl;
          printf(" ActualPos 0: %" PRId32 "\n", in_somanet_1_[0]->PositionValue);
          // printf(" ActualTorque 0: %" PRId32 "\n", in_somanet_1_[0]->TorqueValue);
        }

        needlf_ = true;
      }
    }

    // TODO: parameterize
    osal_usleep(5000);
  }
}

} // namespace synapticon_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(synapticon_ros2_control::SynapticonSystemInterface,
                       hardware_interface::SystemInterface)
