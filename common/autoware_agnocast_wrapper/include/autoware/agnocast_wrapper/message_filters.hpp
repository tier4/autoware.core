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

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"
#include "autoware/agnocast_wrapper/node.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/message_filters/subscriber.hpp>
#include <agnocast/message_filters/sync_policies/approximate_time.hpp>
#include <agnocast/message_filters/sync_policies/exact_time.hpp>
#include <agnocast/message_filters/synchronizer.hpp>

namespace autoware::agnocast_wrapper
{
namespace message_filters
{

/// @brief Wrapper message_filters Subscriber that switches between
///        rclcpp and agnocast message_filters at runtime via use_agnocast().
///
/// Accepts either an `autoware::agnocast_wrapper::Node *` or a plain `rclcpp::Node *`. The
/// rclcpp::Node* overload exists so that nodes whose base is `rclcpp::Node` — e.g. filters
/// templated on `rclcpp::Node` to opt out of the wrapper migration — can still use this
/// Subscriber type without a compile-time mismatch. At runtime that overload requires
/// use_agnocast()=false; otherwise it logs an error and skips subscribing because a
/// rclcpp::Node has no agnocast::Node backing.
template <class M>
class Subscriber
{
public:
  Subscriber() = default;

  Subscriber(
    autoware::agnocast_wrapper::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node, topic, qos);
  }

  Subscriber(
    rclcpp::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node, topic, qos);
  }

  void subscribe(
    autoware::agnocast_wrapper::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    if (use_agnocast()) {
      agnocast_sub_.subscribe(node->get_agnocast_node().get(), topic, qos);
    } else {
      rclcpp_sub_.subscribe(node->get_rclcpp_node().get(), topic, qos);
    }
  }

  // Subscribe via a plain rclcpp::Node*. Requires use_agnocast()=false because a rclcpp::Node
  // has no agnocast::Node backing. If use_agnocast()=true at runtime this is a logic error
  // (the wrapper-on side of the same process should not be paired with a rclcpp::Node-only
  // call site), so we log and skip rather than silently routing through an empty agnocast_sub_.
  void subscribe(
    rclcpp::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    if (use_agnocast()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "agnocast_wrapper::message_filters::Subscriber::subscribe(rclcpp::Node*) called while "
        "use_agnocast()=true. A rclcpp::Node has no agnocast backing — this subscriber will be "
        "left unsubscribed. Switch the node to autoware::agnocast_wrapper::Node, or run with "
        "ENABLE_AGNOCAST=0.");
      return;
    }
    rclcpp_sub_.subscribe(node, topic, qos);
  }

  void unsubscribe()
  {
    if (use_agnocast()) {
      agnocast_sub_.unsubscribe();
    } else {
      rclcpp_sub_.unsubscribe();
    }
  }

  // Internal API: used by ApproximateTimeSynchronizer / ExactTimeSynchronizer.
  // Not intended for downstream use.
  ::message_filters::Subscriber<M> & rclcpp_subscriber() { return rclcpp_sub_; }
  agnocast::message_filters::Subscriber<M, agnocast::Node> & agnocast_subscriber()
  {
    return agnocast_sub_;
  }

private:
  ::message_filters::Subscriber<M> rclcpp_sub_;
  agnocast::message_filters::Subscriber<M, agnocast::Node> agnocast_sub_;
};

/// @brief Wrapper ApproximateTime Synchronizer that switches between
///        rclcpp and agnocast message_filters at runtime.
///
/// The callback uses the rclcpp message_filters legacy signature
///   `(const M0::ConstSharedPtr&, const M1::ConstSharedPtr&)`.
/// In agnocast mode the wrapper promotes the agnocast event into an aliasing
/// std::shared_ptr<const M> that keeps the underlying ipc_shared_ptr alive — zero-copy is
/// preserved for the duration of the callback. This lets caller code register callbacks the
/// same way whether built with USE_AGNOCAST_ENABLED=on or off.
///
/// @note Current limitations:
///   - Maximum 2 message types per Synchronizer.
///
/// @code
/// using namespace autoware::agnocast_wrapper::message_filters;
///
/// Subscriber<sensor_msgs::msg::Image> image_sub;
/// Subscriber<sensor_msgs::msg::CameraInfo> info_sub;
/// image_sub.subscribe(node, "/camera/image", rmw_qos_profile_sensor_data);
/// info_sub.subscribe(node, "/camera/info", rmw_qos_profile_sensor_data);
///
/// using Policy = sync_policies::ApproximateTime<
///     sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
/// auto sync = std::make_shared<Synchronizer<Policy>>(Policy(10), image_sub, info_sub);
/// // or: build the synchronizer first and connect later (mirrors rclcpp message_filters pattern)
/// // auto sync = std::make_shared<Synchronizer<Policy>>(Policy(10));
/// // sync->connectInput(image_sub, info_sub);
///
/// sync->registerCallback(
///   std::bind(&MyNode::onSynchronized, this, std::placeholders::_1, std::placeholders::_2));
///
/// // Where the callback method signature is:
/// // void onSynchronized(
/// //   const sensor_msgs::msg::Image::ConstSharedPtr & img,
/// //   const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);
/// @endcode
template <typename M0, typename M1>
class ApproximateTimeSynchronizer
{
public:
  // Legacy callback signature, matching rclcpp message_filters Synchronizer. Under
  // USE_AGNOCAST_ENABLED=on the wrapper internally converts the agnocast MessageEvent into an
  // aliasing shared_ptr<const M> that keeps the ipc_shared_ptr alive — zero-copy is preserved
  // for the duration of the callback. Using the legacy signature lets caller code register
  // callbacks the same way regardless of build mode, with `std::bind(&Cls::cb, this, _1, _2)`.
  using Callback = std::function<void(
    const typename M0::ConstSharedPtr &, const typename M1::ConstSharedPtr &)>;

  ApproximateTimeSynchronizer(uint32_t queue_size, Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  {
    if (use_agnocast()) {
      agnocast_sync_ = std::make_unique<AgnocastSync>(
        AgnocastPolicy(queue_size), sub0.agnocast_subscriber(), sub1.agnocast_subscriber());
    } else {
      rclcpp_sync_ = std::make_unique<RclcppSync>(
        RclcppPolicy(queue_size), sub0.rclcpp_subscriber(), sub1.rclcpp_subscriber());
    }
  }

  // Queue-only constructor. Pair with connectInput() to attach subscribers after construction,
  // matching the rclcpp message_filters Synchronizer pattern.
  explicit ApproximateTimeSynchronizer(uint32_t queue_size)
  {
    if (use_agnocast()) {
      agnocast_sync_ = std::make_unique<AgnocastSync>(AgnocastPolicy(queue_size));
    } else {
      rclcpp_sync_ = std::make_unique<RclcppSync>(RclcppPolicy(queue_size));
    }
  }

  void connectInput(Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  {
    if (use_agnocast()) {
      agnocast_sync_->connectInput(sub0.agnocast_subscriber(), sub1.agnocast_subscriber());
    } else {
      rclcpp_sync_->connectInput(sub0.rclcpp_subscriber(), sub1.rclcpp_subscriber());
    }
  }

  // Templated to accept any callable (lambdas, std::bind results, std::function). Internally
  // we wrap into the Callback std::function so both backends see a uniform target.
  template <typename CallbackT>
  void registerCallback(CallbackT && callback)
  {
    stored_callback_ = Callback(std::forward<CallbackT>(callback));
    if (use_agnocast()) {
      agnocast_sync_->registerCallback(&ApproximateTimeSynchronizer::agnocastCallbackAdapter, this);
    } else {
      // rclcpp Synchronizer wraps the callable in std::bind with 9 placeholders internally
      // (signal9). A raw 2-arg lambda fails to compile there; std::bind absorbs the extra
      // placeholders, so wrap the stored callback in a 2-arg std::bind before handing it over.
      rclcpp_sync_->registerCallback(
        std::bind(stored_callback_, std::placeholders::_1, std::placeholders::_2));
    }
  }

private:
  Callback stored_callback_;

  using M0Event = agnocast::message_filters::MessageEvent<const M0>;
  using M1Event = agnocast::message_filters::MessageEvent<const M1>;

  void agnocastCallbackAdapter(const M0Event & e0, const M1Event & e1)
  {
    // Promote the agnocast ipc_shared_ptr into a std::shared_ptr<const M> using shared_ptr's
    // aliasing constructor so the ipc_shared_ptr's refcount (and the underlying shared-memory
    // page) stays alive for as long as the callback holds the pointer — zero-copy semantics
    // are preserved for the callback's lifetime.
    auto m0_holder = std::make_shared<agnocast::ipc_shared_ptr<const M0>>(e0.getMessage());
    auto m1_holder = std::make_shared<agnocast::ipc_shared_ptr<const M1>>(e1.getMessage());
    typename M0::ConstSharedPtr m0_ptr(m0_holder, m0_holder->get());
    typename M1::ConstSharedPtr m1_ptr(m1_holder, m1_holder->get());
    stored_callback_(m0_ptr, m1_ptr);
  }

  using RclcppPolicy = ::message_filters::sync_policies::ApproximateTime<M0, M1>;
  using RclcppSync = ::message_filters::Synchronizer<RclcppPolicy>;
  std::unique_ptr<RclcppSync> rclcpp_sync_;

  using AgnocastPolicy = agnocast::message_filters::sync_policies::ApproximateTime<M0, M1>;
  using AgnocastSync = agnocast::message_filters::Synchronizer<AgnocastPolicy>;
  std::unique_ptr<AgnocastSync> agnocast_sync_;
};

/// @brief Wrapper ExactTime Synchronizer mirroring ApproximateTimeSynchronizer.
///
/// Same callback signature and zero-copy semantics as ApproximateTimeSynchronizer;
/// only the sync policy differs.
template <typename M0, typename M1>
class ExactTimeSynchronizer
{
public:
  using Callback = std::function<void(
    const typename M0::ConstSharedPtr &, const typename M1::ConstSharedPtr &)>;

  ExactTimeSynchronizer(uint32_t queue_size, Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  {
    if (use_agnocast()) {
      agnocast_sync_ = std::make_unique<AgnocastSync>(
        AgnocastPolicy(queue_size), sub0.agnocast_subscriber(), sub1.agnocast_subscriber());
    } else {
      rclcpp_sync_ = std::make_unique<RclcppSync>(
        RclcppPolicy(queue_size), sub0.rclcpp_subscriber(), sub1.rclcpp_subscriber());
    }
  }

  // Queue-only constructor. Pair with connectInput() to attach subscribers after construction.
  explicit ExactTimeSynchronizer(uint32_t queue_size)
  {
    if (use_agnocast()) {
      agnocast_sync_ = std::make_unique<AgnocastSync>(AgnocastPolicy(queue_size));
    } else {
      rclcpp_sync_ = std::make_unique<RclcppSync>(RclcppPolicy(queue_size));
    }
  }

  void connectInput(Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  {
    if (use_agnocast()) {
      agnocast_sync_->connectInput(sub0.agnocast_subscriber(), sub1.agnocast_subscriber());
    } else {
      rclcpp_sync_->connectInput(sub0.rclcpp_subscriber(), sub1.rclcpp_subscriber());
    }
  }

  template <typename CallbackT>
  void registerCallback(CallbackT && callback)
  {
    stored_callback_ = Callback(std::forward<CallbackT>(callback));
    if (use_agnocast()) {
      agnocast_sync_->registerCallback(&ExactTimeSynchronizer::agnocastCallbackAdapter, this);
    } else {
      rclcpp_sync_->registerCallback(
        std::bind(stored_callback_, std::placeholders::_1, std::placeholders::_2));
    }
  }

private:
  Callback stored_callback_;

  using M0Event = agnocast::message_filters::MessageEvent<const M0>;
  using M1Event = agnocast::message_filters::MessageEvent<const M1>;

  void agnocastCallbackAdapter(const M0Event & e0, const M1Event & e1)
  {
    auto m0_holder = std::make_shared<agnocast::ipc_shared_ptr<const M0>>(e0.getMessage());
    auto m1_holder = std::make_shared<agnocast::ipc_shared_ptr<const M1>>(e1.getMessage());
    typename M0::ConstSharedPtr m0_ptr(m0_holder, m0_holder->get());
    typename M1::ConstSharedPtr m1_ptr(m1_holder, m1_holder->get());
    stored_callback_(m0_ptr, m1_ptr);
  }

  using RclcppPolicy = ::message_filters::sync_policies::ExactTime<M0, M1>;
  using RclcppSync = ::message_filters::Synchronizer<RclcppPolicy>;
  std::unique_ptr<RclcppSync> rclcpp_sync_;

  using AgnocastPolicy = agnocast::message_filters::sync_policies::ExactTime<M0, M1>;
  using AgnocastSync = agnocast::message_filters::Synchronizer<AgnocastPolicy>;
  std::unique_ptr<AgnocastSync> agnocast_sync_;
};

/// @brief Policy and Synchronizer types that mirror the rclcpp message_filters API.
///        Allows node code to use the same pattern as rclcpp:
///          using SyncPolicy = sync_policies::ApproximateTime<M0, M1>;
///          using Sync = Synchronizer<SyncPolicy>;
///          sync = std::make_shared<Sync>(SyncPolicy(10), sub0, sub1);
namespace sync_policies
{
template <typename M0, typename M1>
struct ApproximateTime
{
  uint32_t queue_size;
  explicit ApproximateTime(uint32_t qs) : queue_size(qs) {}
};

template <typename M0, typename M1>
struct ExactTime
{
  uint32_t queue_size;
  explicit ExactTime(uint32_t qs) : queue_size(qs) {}
};
}  // namespace sync_policies

/// @brief Primary Synchronizer template — supports ApproximateTime and ExactTime specializations.
template <typename Policy>
class Synchronizer
{
  static_assert(
    sizeof(Policy) == 0,
    "Only sync_policies::ApproximateTime<M0, M1> and sync_policies::ExactTime<M0, M1> "
    "are supported. Policies with more than 2 message types are not implemented.");
};

template <typename M0, typename M1>
class Synchronizer<sync_policies::ApproximateTime<M0, M1>>
: public ApproximateTimeSynchronizer<M0, M1>
{
public:
  Synchronizer(
    sync_policies::ApproximateTime<M0, M1> policy, Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  : ApproximateTimeSynchronizer<M0, M1>(policy.queue_size, sub0, sub1)
  {
  }

  explicit Synchronizer(sync_policies::ApproximateTime<M0, M1> policy)
  : ApproximateTimeSynchronizer<M0, M1>(policy.queue_size)
  {
  }

  // Mirrors the rclcpp message_filters Synchronizer constructor that takes only a queue size.
  explicit Synchronizer(uint32_t queue_size) : ApproximateTimeSynchronizer<M0, M1>(queue_size) {}
};

template <typename M0, typename M1>
class Synchronizer<sync_policies::ExactTime<M0, M1>> : public ExactTimeSynchronizer<M0, M1>
{
public:
  Synchronizer(
    sync_policies::ExactTime<M0, M1> policy, Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  : ExactTimeSynchronizer<M0, M1>(policy.queue_size, sub0, sub1)
  {
  }

  explicit Synchronizer(sync_policies::ExactTime<M0, M1> policy)
  : ExactTimeSynchronizer<M0, M1>(policy.queue_size)
  {
  }

  explicit Synchronizer(uint32_t queue_size) : ExactTimeSynchronizer<M0, M1>(queue_size) {}
};

}  // namespace message_filters
}  // namespace autoware::agnocast_wrapper

#else

namespace autoware
{
namespace agnocast_wrapper
{
namespace message_filters
{

template <class M>
using Subscriber = ::message_filters::Subscriber<M>;

template <typename M0, typename M1>
using ApproximateTimeSynchronizer =
  ::message_filters::Synchronizer<::message_filters::sync_policies::ApproximateTime<M0, M1>>;

template <typename M0, typename M1>
using ExactTimeSynchronizer =
  ::message_filters::Synchronizer<::message_filters::sync_policies::ExactTime<M0, M1>>;

namespace sync_policies
{
template <typename M0, typename M1>
using ApproximateTime = ::message_filters::sync_policies::ApproximateTime<M0, M1>;
template <typename M0, typename M1>
using ExactTime = ::message_filters::sync_policies::ExactTime<M0, M1>;
}  // namespace sync_policies

template <typename Policy>
using Synchronizer = ::message_filters::Synchronizer<Policy>;

}  // namespace message_filters
}  // namespace agnocast_wrapper
}  // namespace autoware

#endif
