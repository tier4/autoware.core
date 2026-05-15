// Copyright 2025 TIER IV, Inc.
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

#pragma once

#include "autoware/agnocast_wrapper/node.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/diagnostic_updater/diagnostic_updater.hpp>

#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>

namespace autoware::agnocast_wrapper
{

/// @brief Wrapper Updater that dispatches between diagnostic_updater::Updater (rclcpp mode)
///        and agnocast::Updater (agnocast mode) at runtime, depending on whether the given
///        autoware::agnocast_wrapper::Node is running in agnocast mode.
///
/// Constructor signature mirrors `diagnostic_updater::Updater updater_{this};` so nodes
/// inheriting from autoware::agnocast_wrapper::Node can use the same idiom in both modes.
class Updater
{
public:
  explicit Updater(autoware::agnocast_wrapper::Node * node, double period = 1.0)
  {
    if (node->is_using_agnocast()) {
      agnocast_impl_ = std::make_unique<agnocast::Updater>(*node->get_agnocast_node(), period);
    } else {
      ros2_impl_ =
        std::make_unique<diagnostic_updater::Updater>(node->get_rclcpp_node(), period);
    }
  }

  void add(const std::string & name, diagnostic_updater::TaskFunction f)
  {
    if (agnocast_impl_) {
      agnocast_impl_->add(name, f);
    } else {
      ros2_impl_->add(name, f);
    }
  }

  void add(diagnostic_updater::DiagnosticTask & task)
  {
    if (agnocast_impl_) {
      agnocast_impl_->add(task);
    } else {
      ros2_impl_->add(task);
    }
  }

  template <class T>
  void add(
    const std::string name, T * c,
    void (T::*f)(diagnostic_updater::DiagnosticStatusWrapper &))
  {
    if (agnocast_impl_) {
      agnocast_impl_->add(name, c, f);
    } else {
      ros2_impl_->add(name, c, f);
    }
  }

  bool removeByName(const std::string name)
  {
    return agnocast_impl_ ? agnocast_impl_->removeByName(name)
                          : ros2_impl_->removeByName(name);
  }

  auto getPeriod() const
  {
    return agnocast_impl_ ? agnocast_impl_->getPeriod() : ros2_impl_->getPeriod();
  }

  void setPeriod(rclcpp::Duration period)
  {
    if (agnocast_impl_) {
      agnocast_impl_->setPeriod(period);
    } else {
      ros2_impl_->setPeriod(period);
    }
  }

  void setPeriod(double period)
  {
    if (agnocast_impl_) {
      agnocast_impl_->setPeriod(period);
    } else {
      ros2_impl_->setPeriod(period);
    }
  }

  void force_update()
  {
    if (agnocast_impl_) {
      agnocast_impl_->force_update();
    } else {
      ros2_impl_->force_update();
    }
  }

  void broadcast(unsigned char lvl, const std::string msg)
  {
    if (agnocast_impl_) {
      agnocast_impl_->broadcast(lvl, msg);
    } else {
      ros2_impl_->broadcast(lvl, msg);
    }
  }

  void setHardwareID(const std::string & hwid)
  {
    if (agnocast_impl_) {
      agnocast_impl_->setHardwareID(hwid);
    } else {
      ros2_impl_->setHardwareID(hwid);
    }
  }

  void setHardwareIDf(const char * format, ...)
  {
    va_list va;
    constexpr int kBufferSize = 1000;
    char buff[kBufferSize];
    va_start(va, format);
    vsnprintf(buff, kBufferSize, format, va);
    va_end(va);
    setHardwareID(std::string(buff));
  }

  Updater(const Updater &) = delete;
  Updater & operator=(const Updater &) = delete;

private:
  std::unique_ptr<diagnostic_updater::Updater> ros2_impl_;
  std::unique_ptr<agnocast::Updater> agnocast_impl_;
};

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{
using Updater = diagnostic_updater::Updater;
}  // namespace autoware::agnocast_wrapper

#endif
