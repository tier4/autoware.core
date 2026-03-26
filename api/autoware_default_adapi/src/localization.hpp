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

#ifndef LOCALIZATION_HPP_
#define LOCALIZATION_HPP_

#include <agnocast/agnocast.hpp>
#include <autoware/adapi_specs/localization.hpp>
#include <autoware/component_interface_specs/localization.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::default_adapi
{

class LocalizationNode : public agnocast::Node
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options);

private:
  using ImplState = autoware::component_interface_specs::localization::InitializationState;

  rclcpp::CallbackGroup::SharedPtr group_cli_;
  agnocast::Service<autoware::adapi_specs::localization::Initialize::Service>::SharedPtr
    srv_initialize_;
  agnocast::Publisher<autoware::adapi_specs::localization::InitializationState::Message>::SharedPtr
    pub_state_;
  agnocast::Client<autoware::component_interface_specs::localization::Initialize::Service>::SharedPtr
    cli_initialize_;
  agnocast::Subscription<
    autoware::component_interface_specs::localization::InitializationState::Message>::SharedPtr
    sub_state_;

  void on_state(const agnocast::ipc_shared_ptr<const ImplState::Message> & msg);
  void on_initialize(
    const agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::localization::Initialize::Service>::RequestT> & req,
    agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::localization::Initialize::Service>::ResponseT> &
      res);

  ImplState::Message state_;
};

}  // namespace autoware::default_adapi

#endif  // LOCALIZATION_HPP_
