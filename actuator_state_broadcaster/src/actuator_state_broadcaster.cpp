// Copyright 2021 ros2_control development team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "actuator_state_broadcaster/actuator_state_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace actuator_state_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

ActuatorStateBroadcaster::ActuatorStateBroadcaster() {}

controller_interface::CallbackReturn ActuatorStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ActuatorStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ActuatorStateBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (use_all_available_interfaces())
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  }
  else
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & actuator : params_.actuators)
    {
      for (const auto & interface : params_.interfaces)
      {
        state_interfaces_config.names.push_back(actuator + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn ActuatorStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (use_all_available_interfaces())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'actuators' or 'interfaces' parameter is empty. "
      "All available state interfaces will be published");
    params_.actuators.clear();
    params_.interfaces.clear();
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Publishing state interfaces defined in 'actuators' and 'interfaces' parameters.");
  }

  auto get_map_interface_parameter =
    [&](std::string const & interface, std::string const & interface_to_map)
  {
    if (
      std::find(params_.interfaces.begin(), params_.interfaces.end(), interface) !=
      params_.interfaces.end())
    {
      map_interface_to_actuator_state_[interface] = interface;
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Mapping from '%s' to interface '%s' will not be done, because '%s' is defined "
        "in 'interface' parameter.",
        interface_to_map.c_str(), interface.c_str(), interface.c_str());
    }
    else
    {
      map_interface_to_actuator_state_[interface_to_map] = interface;
    }
  };

  map_interface_to_actuator_state_ = {};
  get_map_interface_parameter(HW_IF_POSITION, params_.map_interface_to_actuator_state.position);
  get_map_interface_parameter(HW_IF_VELOCITY, params_.map_interface_to_actuator_state.velocity);
  get_map_interface_parameter(HW_IF_EFFORT, params_.map_interface_to_actuator_state.effort);

  try
  {
    const std::string topic_name_prefix = params_.use_local_topics ? "~/" : "";

    actuator_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      topic_name_prefix + "actuator_states", rclcpp::SystemDefaultsQoS());

    realtime_actuator_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        actuator_state_publisher_);
  }
  catch (const std::exception & e)
  {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ActuatorStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_actuator_data())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  init_actuator_state_msg();

  if (
    !use_all_available_interfaces() &&
    state_interfaces_.size() != (params_.actuators.size() * params_.interfaces.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exists. "
      "Check ControllerManager output for more detailed information.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ActuatorStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  actuator_names_.clear();

  return CallbackReturn::SUCCESS;
}

template <typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  bool found_key = false;
  for (const auto & key_item : map)
  {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend())
    {
      found_key = true;
      break;
    }
  }
  return found_key;
}

bool ActuatorStateBroadcaster::init_actuator_data()
{
  actuator_names_.clear();
  if (state_interfaces_.empty())
  {
    return false;
  }

  // loop in reverse order, this maintains the order of values at retrieval time
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // initialize map if name is new
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0)
    {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    // add interface name
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_actuator_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_actuator_state_[interface_name];
    }
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = kUninitializedValue;
  }

  // filter state interfaces that have at least one of the actuator_states fields,
  // the rest will be ignored for this message
  for (const auto & name_ifv : name_if_value_mapping_)
  {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT}))
    {
      actuator_names_.push_back(name_ifv.first);
    }
  }

  // Add extra actuators from parameters, each actuator will be added to actuator_names_ and
  // name_if_value_mapping_ if it is not already there
  rclcpp::Parameter extra_actuators;
  if (get_node()->get_parameter("extra_actuators", extra_actuators))
  {
    const std::vector<std::string> & extra_actuators_names = extra_actuators.as_string_array();
    for (const auto & extra_actuator_name : extra_actuators_names)
    {
      if (name_if_value_mapping_.count(extra_actuator_name) == 0)
      {
        name_if_value_mapping_[extra_actuator_name] = {
          {HW_IF_POSITION, 0.0}, {HW_IF_VELOCITY, 0.0}, {HW_IF_EFFORT, 0.0}};
        actuator_names_.push_back(extra_actuator_name);
      }
    }
  }

  return true;
}

void ActuatorStateBroadcaster::init_actuator_state_msg()
{
  const size_t num_actuators = actuator_names_.size();

  /// @note actuator_state_msg publishes position, velocity and effort for all actuators,
  /// with at least one of these interfaces, the rest are omitted from this message

  // default initialization for actuator state message
  auto & actuator_state_msg = realtime_actuator_state_publisher_->msg_;
  actuator_state_msg.name = actuator_names_;
  actuator_state_msg.position.resize(num_actuators, kUninitializedValue);
  actuator_state_msg.velocity.resize(num_actuators, kUninitializedValue);
  actuator_state_msg.effort.resize(num_actuators, kUninitializedValue);
}

bool ActuatorStateBroadcaster::use_all_available_interfaces() const
{
  return params_.actuators.empty() || params_.interfaces.empty();
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend())
  {
    return interface_and_value->second;
  }
  else
  {
    return kUninitializedValue;
  }
}

controller_interface::return_type ActuatorStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (const auto & state_interface : state_interfaces_)
  {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_actuator_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_actuator_state_[interface_name];
    }
    name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
      state_interface.get_value());
  }

  if (realtime_actuator_state_publisher_ && realtime_actuator_state_publisher_->trylock())
  {
    auto & actuator_state_msg = realtime_actuator_state_publisher_->msg_;

    actuator_state_msg.header.stamp = time;

    // update actuator state message
    for (size_t i = 0; i < actuator_names_.size(); ++i)
    {
      actuator_state_msg.position[i] =
        get_value(name_if_value_mapping_, actuator_names_[i], HW_IF_POSITION);
      actuator_state_msg.velocity[i] =
        get_value(name_if_value_mapping_, actuator_names_[i], HW_IF_VELOCITY);
      actuator_state_msg.effort[i] = get_value(name_if_value_mapping_, actuator_names_[i], HW_IF_EFFORT);
    }
    realtime_actuator_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace actuator_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  actuator_state_broadcaster::ActuatorStateBroadcaster, controller_interface::ControllerInterface)
