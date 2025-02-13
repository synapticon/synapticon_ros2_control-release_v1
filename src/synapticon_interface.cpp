// Copyright (c) 2025 Elevate Robotics Inc
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "synapticon_ros2_control/synapticon_interface.hpp"

#include <algorithm>
#include <chrono>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <inttypes.h>
#include <rclcpp/rclcpp.hpp>

#define EC_TIMEOUTMON 500

using namespace std::chrono_literals;

namespace synapticon_ros2_control {
namespace {
constexpr char LOG_NAME[] = "synapticon_ros2_control";
constexpr double DEG_TO_RAD = 0.0174533;
constexpr size_t PROFILE_TORQUE_MODE = 4;
constexpr size_t CYCLIC_VELOCITY_MODE = 9;
constexpr size_t CYCLIC_POSITION_MODE = 8;
constexpr double RPM_TO_RAD_PER_S = 0.10472;
constexpr double RAD_PER_S_TO_RPM = 1 / RPM_TO_RAD_PER_S;
unsigned int NORMAL_OPERATION_BRAKES_OFF = 0b00001111;
// Bit 2 (0-indexed) goes to 0 to turn on Quick Stop
unsigned int NORMAL_OPERATION_BRAKES_ON = 0b00001011;
constexpr char EXPECTED_SLAVE_NAME[] = "SOMANET";
} // namespace

hardware_interface::CallbackReturn SynapticonSystemInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
      rclcpp::get_logger("synapticon_interface"));

  num_joints_ = info_.joints.size();

  hw_states_positions_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(num_joints_,
                               std::numeric_limits<double>::quiet_NaN());
  hw_states_accelerations_.resize(num_joints_,
                                  std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(num_joints_,
                            std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(num_joints_,
                                std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(num_joints_,
                                 std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  hw_commands_quick_stop_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(num_joints_, control_level_t::UNDEFINED);
  // Atomic deques are difficult to initialize
  threadsafe_commands_efforts_.resize(num_joints_);
  for (auto &effort : threadsafe_commands_efforts_) {
    effort.store(std::numeric_limits<double>::quiet_NaN());
  }
  threadsafe_commands_velocities_.resize(num_joints_);
  for (auto &velocity : threadsafe_commands_velocities_) {
    velocity.store(0.0);
  }
  threadsafe_commands_positions_.resize(num_joints_);
  for (auto &position : threadsafe_commands_positions_) {
    position.store(std::numeric_limits<double>::quiet_NaN());
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (!(joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name ==
              "quick_stop" ||
          joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' has %s command interface. Expected %s, %s, %s, or %s.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          "quick_stop",
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
  // Define the interface name (e.g. eth0 or eno0) in the ros2_control.xacro
  std::string interface_name = info_.hardware_parameters["interface_name"];
  int ec_init_status = ec_init(interface_name.c_str());
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
  for (int slave_id = 0; slave_id < ec_slavecount; ++slave_id) {
    ec_slave[slave_id].state = EC_STATE_OPERATIONAL;
  }
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
  for (size_t joint_idx = 1; joint_idx < (num_joints_ + 1); ++joint_idx) {
    in_somanet_1_.push_back((InSomanet50t *)ec_slave[joint_idx].inputs);
    out_somanet_1_.push_back((OutSomanet50t *)ec_slave[joint_idx].outputs);
  }

  // Read encoder resolution
  uint8_t encoder_source;
  int size = sizeof(encoder_source);
  ec_SDOread(1, 0x2012, 0x09, false, &size, &encoder_source, EC_TIMEOUTRXM);
  size = sizeof(encoder_resolution_);
  if (encoder_source == 1) {
    ec_SDOread(1, 0x2110, 0x03, false, &size, &encoder_resolution_,
               EC_TIMEOUTRXM);
  } else if (encoder_source == 2) {
    ec_SDOread(1, 0x2112, 0x03, false, &size, &encoder_resolution_,
               EC_TIMEOUTRXM);
  } else {
    RCLCPP_FATAL(
        get_logger(),
        "No encoder configured for position control. Terminating the program");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Start the control loop, wait for it to reach normal operation mode
  somanet_control_thread_ =
      std::thread(&SynapticonSystemInterface::somanetCyclicLoop, this,
                  std::ref(in_normal_op_mode_));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SynapticonSystemInterface::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  // Prepare for new command modes
  std::vector<control_level_t> new_modes = {};
  for (const std::string& key : start_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key ==
          info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_modes.push_back(control_level_t::EFFORT);
      } else if (key == info_.joints[i].name + "/" +
                            hardware_interface::HW_IF_VELOCITY) {
        new_modes.push_back(control_level_t::VELOCITY);
      } else if (key == info_.joints[i].name + "/" +
                            hardware_interface::HW_IF_POSITION) {
        new_modes.push_back(control_level_t::POSITION);
      } else if (key == info_.joints[i].name + "/quick_stop") {
        new_modes.push_back(control_level_t::QUICK_STOP);
      }
    }
  }
  // All joints must be given new command mode at the same time
  if (!start_interfaces.empty() && (new_modes.size() != num_joints_)) {
    RCLCPP_FATAL(get_logger(),
                 "All joints must be given a new mode at the same time.");
    return hardware_interface::return_type::ERROR;
  }
  // All joints must have the same command mode
  if (!std::all_of(
          new_modes.begin() + 1, new_modes.end(),
          [&](control_level_t mode) { return mode == new_modes[0]; })) {
    RCLCPP_FATAL(get_logger(), "All joints must have the same command mode.");
    return hardware_interface::return_type::ERROR;
  }

  // Stop motion on all relevant joints
  for (const std::string& key : stop_interfaces) {
    for (std::size_t i = 0; i < num_joints_; i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        threadsafe_commands_efforts_[i] =
            std::numeric_limits<double>::quiet_NaN();
        threadsafe_commands_velocities_[i] = 0;
        threadsafe_commands_positions_[i] =
            std::numeric_limits<double>::quiet_NaN();
        control_level_[i] = control_level_t::UNDEFINED;
      }
    }
  }

  for (const std::string& key : start_interfaces) {
    for (std::size_t i = 0; i < num_joints_; i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        if (control_level_[i] != control_level_t::UNDEFINED) {
          // Something else is using the joint! Abort!
          RCLCPP_FATAL(get_logger(),
                       "Something else is using the joint. Abort!");
          return hardware_interface::return_type::ERROR;
        }
        control_level_[i] = new_modes[i];
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn SynapticonSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  // Set some default values
  for (std::size_t i = 0; i < num_joints_; i++) {
    if (std::isnan(hw_states_velocities_[i])) {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_accelerations_[i])) {
      hw_states_accelerations_[i] = 0;
    }
    if (std::isnan(hw_states_efforts_[i])) {
      hw_states_efforts_[i] = 0;
    }

    hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_velocities_[i] = 0;
    hw_commands_efforts_[i] = 0;
    threadsafe_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0;
    threadsafe_commands_positions_[i] =
        std::numeric_limits<double>::quiet_NaN();
  }

  RCLCPP_INFO(get_logger(), "System successfully activated! Control level: %u",
              control_level_[0]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SynapticonSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  for (std::size_t i = 0; i < num_joints_; i++) {
    control_level_[i] = control_level_t::UNDEFINED;

    hw_commands_velocities_[i] = 0;
    hw_commands_efforts_[i] = 0;
    threadsafe_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0;
    threadsafe_commands_positions_[i] =
        std::numeric_limits<double>::quiet_NaN();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SynapticonSystemInterface::read(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) {

  for (std::size_t i = 0; i < num_joints_; i++) {
    std::lock_guard<std::mutex> lock(in_somanet_mtx_);
    // InSomanet50t doesn't include acceleration
    hw_states_accelerations_[i] = 0;
    hw_states_velocities_[i] = in_somanet_1_[i]->VelocityValue * RPM_TO_RAD_PER_S;
    hw_states_positions_[i] = in_somanet_1_[0]->PositionValue * 2 * 3.14159 / encoder_resolution_;
    hw_states_efforts_[i] = in_somanet_1_[i]->TorqueValue;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SynapticonSystemInterface::write(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  // This function doesn't do much.
  // It's taken care of in separate thread, somanet_control_thread_

  // Share the commands with somanet control loop in a threadsafe way
  for (std::size_t i = 0; i < num_joints_; i++) {
    // Torque commands are "per thousand of rated torque"
    if (!std::isnan(hw_commands_efforts_[i]))
    {
      hw_commands_efforts_[i] =
          std::clamp(hw_commands_efforts_[i], -1000.0, 1000.0);
      threadsafe_commands_efforts_[i] = hw_commands_efforts_[i];
    }
    if (!std::isnan(hw_commands_velocities_[i]))
    {
      threadsafe_commands_velocities_[i] = hw_commands_velocities_[i] * RAD_PER_S_TO_RPM;
    }
    if (!std::isnan(hw_commands_positions_[i]))
    {
      threadsafe_commands_positions_[i] = hw_commands_positions_[i] * encoder_resolution_ / (2 * 3.14159);
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
SynapticonSystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < num_joints_; i++) {
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
  for (std::size_t i = 0; i < num_joints_; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &hw_commands_efforts_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "quick_stop",
        &hw_commands_quick_stop_[i]));
  }
  return command_interfaces;
}

SynapticonSystemInterface::~SynapticonSystemInterface() {
  // A flag to ecat_error_check_ thread
  in_normal_op_mode_ = false;

  if (somanet_control_thread_ && somanet_control_thread_->joinable()) {
    somanet_control_thread_->join();
  }

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
      // one or more slaves are not responding
      ec_group[currentgroup].docheckstate = false;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++) {
        if (ec_slave[slave].name != EXPECTED_SLAVE_NAME)
        {
          continue;
        }
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
            // re-check state
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

void SynapticonSystemInterface::somanetCyclicLoop(
    std::atomic<bool> &in_normal_op_mode) {
  std::vector<bool> first_iteration(num_joints_ , true);

  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(in_somanet_mtx_);
      ec_send_processdata();
      wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

      if (wkc_ >= expected_wkc_) {
        for (size_t joint_idx = 0; joint_idx < num_joints_; ++joint_idx) {
          if (first_iteration.at(joint_idx)) {
            // Default to PROFILE_TORQUE_MODE
            out_somanet_1_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
            out_somanet_1_[joint_idx]->TorqueOffset = 0;
            out_somanet_1_[joint_idx]->TargetTorque = 0;
            first_iteration.at(joint_idx) = false;
          }

          // Fault reset: Fault -> Switch on disabled, if the drive is in fault
          // state
          if ((in_somanet_1_[joint_idx]->Statusword & 0b0000000001001111) ==
              0b0000000000001000)
            out_somanet_1_[joint_idx]->Controlword = 0b10000000;

          // Shutdown: Switch on disabled -> Ready to switch on
          else if ((in_somanet_1_[joint_idx]->Statusword &
                    0b0000000001001111) == 0b0000000001000000)
          {
            // If the QUICK_STOP controller is on, don't leave this state
            if ((control_level_[joint_idx] != control_level_t::QUICK_STOP) &&
               (control_level_[joint_idx] != control_level_t::UNDEFINED))
            {
              out_somanet_1_[joint_idx]->Controlword = 0b00000110;
            }
          }
          // Switch on: Ready to switch on -> Switched on
          else if ((in_somanet_1_[joint_idx]->Statusword &
                    0b0000000001101111) == 0b0000000000100001)
            out_somanet_1_[joint_idx]->Controlword = 0b00000111;

          // Enable operation: Switched on -> Operation enabled
          else if ((in_somanet_1_[joint_idx]->Statusword &
                    0b0000000001101111) == 0b0000000000100011)
            out_somanet_1_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;

          // Normal operation
          else if ((in_somanet_1_[joint_idx]->Statusword &
                    0b0000000001101111) == 0b0000000000100111) {
            in_normal_op_mode = true;
            if (control_level_[joint_idx] == control_level_t::EFFORT) {
              if (!std::isnan(threadsafe_commands_efforts_[joint_idx])) {
                out_somanet_1_[joint_idx]->TargetTorque =
                    threadsafe_commands_efforts_[joint_idx];
                out_somanet_1_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
                out_somanet_1_[joint_idx]->TorqueOffset = 0;
                out_somanet_1_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              }
            } else if (control_level_[joint_idx] == control_level_t::VELOCITY) {
              if (!std::isnan(threadsafe_commands_velocities_[joint_idx])) {
                out_somanet_1_[joint_idx]->TargetVelocity =
                    threadsafe_commands_velocities_[joint_idx];
                out_somanet_1_[joint_idx]->OpMode = CYCLIC_VELOCITY_MODE;
                out_somanet_1_[joint_idx]->VelocityOffset = 0;
                out_somanet_1_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              }
            } else if (control_level_[joint_idx] == control_level_t::POSITION) {
              if (!std::isnan(threadsafe_commands_positions_[joint_idx])) {
                out_somanet_1_[joint_idx]->TargetPosition = threadsafe_commands_positions_[joint_idx];
                out_somanet_1_[joint_idx]->OpMode = CYCLIC_POSITION_MODE;
                out_somanet_1_[joint_idx]->VelocityOffset = 0;
                out_somanet_1_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              }
            } else if (control_level_[joint_idx] == control_level_t::QUICK_STOP)
            {
              // Turn the brake on
              out_somanet_1_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
              out_somanet_1_[joint_idx]->TorqueOffset = 0;
              out_somanet_1_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_ON;
            } else if (control_level_[joint_idx] == control_level_t::UNDEFINED) {
              out_somanet_1_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
              out_somanet_1_[joint_idx]->TorqueOffset = 0;
              out_somanet_1_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
            }
          }
        }

        // printf("Processdata cycle %4d , WKC %d ,", i, wkc);
        // printf(" Statusword: %X ,", in_somanet_1->Statusword);
        // printf(" Op Mode Display: %d ,", in_somanet_1->OpModeDisplay);
        // printf(" ActualPos: %" PRId32 " ,\n", in_somanet_1_[0]->PositionValue);
        // printf(" DemandPos: %" PRId32 " ,",
        // in_somanet_1_[0]->PositionDemandInternalValue); printf(" ActualVel:
        // %" PRId32 " ,", in_somanet_1_[0]->VelocityValue);
        // printf(" DemandVel: %" PRId32 " ,", in_somanet_1_[0]->VelocityDemandValue);
        //printf("ActualTorque: %" PRId32 " ,", in_somanet_1_[0]->TorqueValue);
        // printf(" DemandTorque: %" PRId32 " ,", in_somanet_1_[0]->TorqueDemand);
        // printf("\n");

        // printf(" T:%" PRId64 "\r", ec_DCtime);
        needlf_ = true;
      }
    } // scope of in_somanet_ mutex lock
    osal_usleep(5000);
  }
}

} // namespace synapticon_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(synapticon_ros2_control::SynapticonSystemInterface,
                       hardware_interface::SystemInterface)
