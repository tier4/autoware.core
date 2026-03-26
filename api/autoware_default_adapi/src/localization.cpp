// Copyright 2022 TIER IV, Inc.
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

#include "localization.hpp"

#include "utils/localization_conversion.hpp"

#include <autoware/component_interface_specs/utils.hpp>

namespace autoware::default_adapi
{

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("localization", options)
{
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // AD API
  pub_state_ =
    this->create_publisher<autoware::adapi_specs::localization::InitializationState::Message>(
      autoware::adapi_specs::localization::InitializationState::name,
      autoware::component_interface_specs::get_qos<
        autoware::adapi_specs::localization::InitializationState>());
  srv_initialize_ =
    this->create_service<autoware::adapi_specs::localization::Initialize::Service>(
      autoware::adapi_specs::localization::Initialize::name,
      std::bind(
        &LocalizationNode::on_initialize, this, std::placeholders::_1, std::placeholders::_2));

  // Component Interface
  sub_state_ = this->create_subscription<
    autoware::component_interface_specs::localization::InitializationState::Message>(
    autoware::component_interface_specs::localization::InitializationState::name,
    autoware::component_interface_specs::get_qos<
      autoware::component_interface_specs::localization::InitializationState>(),
    std::bind(&LocalizationNode::on_state, this, std::placeholders::_1));
  cli_initialize_ =
    this->create_client<autoware::component_interface_specs::localization::Initialize::Service>(
      autoware::component_interface_specs::localization::Initialize::name,
      rclcpp::ServicesQoS(), group_cli_);

  state_.state = ImplState::Message::UNKNOWN;
}

void LocalizationNode::on_state(const agnocast::ipc_shared_ptr<const ImplState::Message> & msg)
{
  state_ = *msg;
  auto loaned = pub_state_->borrow_loaned_message();
  *loaned = *msg;
  pub_state_->publish(std::move(loaned));
}

void LocalizationNode::on_initialize(
  const agnocast::ipc_shared_ptr<
    agnocast::Service<autoware::adapi_specs::localization::Initialize::Service>::RequestT> & req,
  agnocast::ipc_shared_ptr<
    agnocast::Service<autoware::adapi_specs::localization::Initialize::Service>::ResponseT> & res)
{
  if (!cli_initialize_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Initialize service is not ready");
    return;
  }
  res->status = localization_conversion::convert_call(cli_initialize_, req);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::LocalizationNode)
