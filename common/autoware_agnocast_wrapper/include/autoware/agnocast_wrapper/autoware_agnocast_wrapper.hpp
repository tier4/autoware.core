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

#include <string>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED

#include "autoware_utils_rclcpp/polling_subscriber.hpp"

#include <agnocast/agnocast.hpp>
#include <rcl/timer.h>
#include <rclcpp/exceptions/exceptions.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <type_traits>

#define AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) \
  autoware::agnocast_wrapper::message_ptr<    \
    MessageT, autoware::agnocast_wrapper::OwnershipType::Unique>
// For publisher (mutable message)
#define AUTOWARE_MESSAGE_SHARED_PTR(MessageT) \
  autoware::agnocast_wrapper::message_ptr<    \
    MessageT, autoware::agnocast_wrapper::OwnershipType::Shared>
// For subscription (read-only message)
#define AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) \
  autoware::agnocast_wrapper::message_ptr<          \
    const MessageT, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) \
  typename autoware::agnocast_wrapper::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) \
  typename autoware::agnocast_wrapper::Publisher<MessageT>::SharedPtr
#define AUTOWARE_POLLING_SUBSCRIBER_PTR(MessageT) \
  typename autoware::agnocast_wrapper::PollingSubscriber<MessageT>::SharedPtr
#define AUTOWARE_TIMER_PTR autoware::agnocast_wrapper::Timer::SharedPtr
#if 0  // Client/Service wrappers disabled for the initial port.
#define AUTOWARE_CLIENT_PTR(ServiceT) \
  typename autoware::agnocast_wrapper::Client<ServiceT>::SharedPtr
#define AUTOWARE_SERVICE_PTR(ServiceT) \
  typename autoware::agnocast_wrapper::Service<ServiceT>::SharedPtr
#endif

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  autoware::agnocast_wrapper::create_subscription<message_type>(this, topic, qos, callback, options)
#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  autoware::agnocast_wrapper::create_publisher<message_type>(this, arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  autoware::agnocast_wrapper::create_publisher<message_type>(this, arg1, arg2, arg3)
#define AUTOWARE_CREATE_POLLING_SUBSCRIBER(message_type, topic, qos) \
  autoware::agnocast_wrapper::create_polling_subscriber<message_type>(this, topic, qos)
#define AUTOWARE_CREATE_TIMER(period, callback) \
  autoware::agnocast_wrapper::create_timer(this, period, callback)
#define AUTOWARE_CREATE_WALL_TIMER(period, callback) \
  autoware::agnocast_wrapper::create_wall_timer(this, period, callback)

#define AUTOWARE_SUBSCRIPTION_OPTIONS agnocast::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS agnocast::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) publisher->allocate_output_message_unique()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) publisher->allocate_output_message_shared()

namespace autoware::agnocast_wrapper
{

enum class OwnershipType { Unique, Shared };

template <typename MessageT, OwnershipType Ownership>
class message_interface;

template <typename MessageT>
class message_interface<MessageT, OwnershipType::Unique>
{
public:
  message_interface() = default;

  virtual ~message_interface() = default;

  message_interface(const message_interface & r) = delete;
  message_interface & operator=(const message_interface & r) = delete;

  message_interface(message_interface && r) = default;
  message_interface & operator=(message_interface && r) = default;

  virtual MessageT & as_ref() const noexcept = 0;
  virtual MessageT * as_ptr() const noexcept = 0;

  virtual agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept = 0;
  virtual std::unique_ptr<MessageT> move_ros2_ptr() && noexcept = 0;
};

template <typename MessageT>
class message_interface<MessageT, OwnershipType::Shared>
{
public:
  virtual ~message_interface() = default;

  virtual MessageT & as_ref() const noexcept = 0;
  virtual MessageT * as_ptr() const noexcept = 0;

  virtual agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept = 0;
  virtual std::shared_ptr<MessageT> move_ros2_ptr() && noexcept = 0;
};

template <typename MessageT, OwnershipType Ownership>
class agnocast_message : public message_interface<MessageT, Ownership>
{
  using ros2_ptr_t = std::conditional_t<
    Ownership == OwnershipType::Unique, std::unique_ptr<MessageT>, std::shared_ptr<MessageT>>;

  agnocast::ipc_shared_ptr<MessageT> ptr_;

public:
  explicit agnocast_message(agnocast::ipc_shared_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return std::move(ptr_);
  }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  ros2_ptr_t move_ros2_ptr() && noexcept override { return ros2_ptr_t{}; }
};

template <typename MessageT, OwnershipType Ownership>
class ros2_message : public message_interface<MessageT, Ownership>
{
  using ros2_ptr_t = std::conditional_t<
    Ownership == OwnershipType::Unique, std::unique_ptr<MessageT>, std::shared_ptr<MessageT>>;

  ros2_ptr_t ptr_;

public:
  explicit ros2_message(ros2_ptr_t && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  ros2_ptr_t move_ros2_ptr() && noexcept override { return std::move(ptr_); }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return agnocast::ipc_shared_ptr<MessageT>{};
  }
};

template <typename MessageT, OwnershipType Ownership>
class message_ptr
{
  using ros2_ptr_t = std::conditional_t<
    Ownership == OwnershipType::Unique, std::unique_ptr<MessageT>, std::shared_ptr<MessageT>>;

  std::shared_ptr<message_interface<MessageT, Ownership>> ptr_;

  template <typename U>
  friend class AgnocastPublisher;
  template <typename U>
  friend class ROS2Publisher;

private:
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept
  {
    return std::move(*(std::move(ptr_))).move_agnocast_ptr();
  }

  auto move_ros2_ptr() && noexcept { return std::move(*(std::move(ptr_))).move_ros2_ptr(); }

public:
  message_ptr() : ptr_(nullptr) {}

  explicit message_ptr(agnocast::ipc_shared_ptr<MessageT> && ptr)
  : ptr_(std::make_unique<agnocast_message<MessageT, Ownership>>(std::move(ptr)))
  {
  }

  explicit message_ptr(ros2_ptr_t && ptr)
  : ptr_(std::make_unique<ros2_message<MessageT, Ownership>>(std::move(ptr)))
  {
  }

  MessageT & operator*() const noexcept { return ptr_->as_ref(); }

  MessageT * operator->() const noexcept { return ptr_->as_ptr(); }

  explicit operator bool() const noexcept { return ptr_ && static_cast<bool>(ptr_->as_ptr()); }

  MessageT * get() const noexcept { return ptr_ ? ptr_->as_ptr() : nullptr; }
};

// Defaults to zero if the environment variable is missing or invalid.
inline int get_ENABLE_AGNOCAST()
{
  const char * env = std::getenv("ENABLE_AGNOCAST");
  if (env) {
    return std::atoi(env);
  }
  return 0;
}

inline bool use_agnocast()
{
  static const int sv = get_ENABLE_AGNOCAST();
  return sv == 1;
}

template <typename MessageT>
class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;

  virtual ~Subscription() = default;

  virtual const std::string & get_topic_name() const = 0;
};

template <typename MessageT>
class AgnocastSubscription : public Subscription<MessageT>
{
  typename agnocast::Subscription<MessageT>::SharedPtr subscription_;
  std::string topic_name_;

public:
  const std::string & get_topic_name() const override { return topic_name_; }

  template <typename NodeT, typename Func>
  explicit AgnocastSubscription(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options)
  : topic_name_(topic_name)
  {
    // TODO(Koichi98): AUTOWARE_MESSAGE_UNIQUE_PTR should be disallowed for Agnocast subscriptions.
    // Agnocast uses shared memory, so mutable exclusive ownership is semantically incorrect and
    // risks corrupting data read by other subscribers. Currently kept for compatibility with
    // CudaPointcloudPreprocessorNode which uses UNIQUE_PTR callbacks.
    static_assert(
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
        std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&>,
      "callback should be invocable with an rvalue reference to either "
      "AUTOWARE_MESSAGE_UNIQUE_PTR or AUTOWARE_MESSAGE_CONST_SHARED_PTR");

    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    subscription_ = agnocast::create_subscription<MessageT>(
      node, topic_name, qos,
      [callback = std::forward<Func>(callback)](agnocast::ipc_shared_ptr<MessageT> && msg) {
        if constexpr (ownership == OwnershipType::Unique) {
          callback(message_ptr<MessageT, ownership>(std::move(msg)));
        } else {
          callback(
            message_ptr<const MessageT, ownership>(
              agnocast::ipc_shared_ptr<const MessageT>(std::move(msg))));
        }
      },
      options);
  }
};

template <typename MessageT>
class ROS2Subscription : public Subscription<MessageT>
{
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  std::string topic_name_;

public:
  const std::string & get_topic_name() const override { return topic_name_; }

  template <typename Func>
  explicit ROS2Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options)
  : topic_name_(topic_name)
  {
    static_assert(
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
        std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&>,
      "callback should be invocable with an rvalue reference to either "
      "AUTOWARE_MESSAGE_UNIQUE_PTR or AUTOWARE_MESSAGE_CONST_SHARED_PTR");

    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    rclcpp::SubscriptionOptions ros2_options;
    ros2_options.callback_group = options.callback_group;
    subscription_ = node->create_subscription<MessageT>(
      topic_name, qos,
      [callback = std::forward<Func>(callback)](std::unique_ptr<MessageT> msg) {
        if constexpr (ownership == OwnershipType::Unique) {
          callback(message_ptr<MessageT, ownership>(std::move(msg)));
        } else {
          callback(
            message_ptr<const MessageT, ownership>(
              std::shared_ptr<const MessageT>(std::move(msg))));
        }
      },
      ros2_options);
  }
};

template <typename MessageT, typename Func>
typename Subscription<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
  const agnocast::SubscriptionOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastSubscription<MessageT>>(
      node, topic_name, qos, std::forward<Func>(callback), options);
  } else {
    return std::make_shared<ROS2Subscription<MessageT>>(
      node, topic_name, qos, std::forward<Func>(callback), options);
  }
}

template <typename MessageT, typename Func>
typename Subscription<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth,
  Func && callback, const agnocast::SubscriptionOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastSubscription<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)),
      std::forward<Func>(callback), options);
  } else {
    return std::make_shared<ROS2Subscription<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)),
      std::forward<Func>(callback), options);
  }
}

template <typename MessageT>
class PollingSubscriber
{
public:
  using SharedPtr = std::shared_ptr<PollingSubscriber<MessageT>>;

  virtual ~PollingSubscriber() = default;

  virtual AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() = 0;
  virtual AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() = 0;
};

template <typename MessageT>
class AgnocastPollingSubscriber : public PollingSubscriber<MessageT>
{
  typename agnocast::PollingSubscriber<MessageT>::SharedPtr subscriber_;

public:
  template <typename NodeT>
  explicit AgnocastPollingSubscriber(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(agnocast::create_subscription<MessageT>(node, topic_name, qos))
  {
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() override
  {
    auto data = subscriber_->take_data();
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(data));
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() override
  {
    auto data = subscriber_->take_data();
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(data));
  }
};

template <typename MessageT>
class ROS2PollingSubscriber : public PollingSubscriber<MessageT>
{
  typename autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT>::SharedPtr subscriber_;

public:
  explicit ROS2PollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(
      autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT>::create_subscription(
        node, topic_name, qos))
  {
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(subscriber_->take_data()));
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(subscriber_->take_data()));
  }
};

template <typename MessageT>
typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  } else {
    return std::make_shared<ROS2PollingSubscriber<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }
}

template <typename MessageT>
typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT>>(node, topic_name, qos);
  } else {
    return std::make_shared<ROS2PollingSubscriber<MessageT>>(node, topic_name, qos);
  }
}

template <typename MessageT>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<MessageT>>;

  virtual ~Publisher() = default;

  virtual AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() = 0;
  virtual AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() = 0;

  virtual void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message) = 0;
  virtual void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message) = 0;

  /// Publish by const reference (internally copies into allocated message).
  /// This method is discouraged because it performs an implicit copy.
  /// Prefer ALLOCATE_OUTPUT_MESSAGE_{UNIQUE,SHARED}(publisher) + the corresponding publish()
  /// overload. May be marked [[deprecated]] in the future once autoware_cmake supports
  /// suppressing deprecation warnings for test targets.
  virtual void publish(const MessageT & data) = 0;

  virtual uint32_t get_subscription_count() const = 0;
  virtual uint32_t get_intra_process_subscription_count() const = 0;
  virtual const rmw_gid_t & get_gid() const = 0;
  virtual const char * get_topic_name() const = 0;
};

template <typename MessageT>
class AgnocastPublisher : public Publisher<MessageT>
{
  typename agnocast::Publisher<MessageT>::SharedPtr publisher_;

public:
  template <typename NodeT>
  explicit AgnocastPublisher(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::PublisherOptions & options)
  : publisher_(agnocast::create_publisher<MessageT>(node, topic_name, qos, options))
  {
  }

  AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() override
  {
    return AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT){publisher_->borrow_loaned_message()};
  }

  AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(MessageT){publisher_->borrow_loaned_message()};
  }

  void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message) override
  {
    publisher_->publish(std::move(message).move_agnocast_ptr());
  }

  void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message) override
  {
    publisher_->publish(std::move(message).move_agnocast_ptr());
  }

  // See the comment on Publisher::publish(const MessageT &) for why this exists.
  void publish(const MessageT & data) override
  {
    auto msg = publisher_->borrow_loaned_message();
    *msg = data;
    publisher_->publish(std::move(msg));
  }

  uint32_t get_subscription_count() const override { return publisher_->get_subscription_count(); }
  uint32_t get_intra_process_subscription_count() const override
  {
    return publisher_->get_intra_subscription_count();
  }
  const rmw_gid_t & get_gid() const override { return publisher_->get_gid(); }
  const char * get_topic_name() const override { return publisher_->get_topic_name(); }
};

template <typename MessageT>
class ROS2Publisher : public Publisher<MessageT>
{
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_{nullptr};

public:
  explicit ROS2Publisher(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::PublisherOptions & options)
  {
    rclcpp::PublisherOptions ros2_options;
    ros2_options.qos_overriding_options = options.qos_overriding_options;
    publisher_ = node->create_publisher<MessageT>(topic_name, qos, ros2_options);
  }

  AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() override
  {
    return AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT){std::make_unique<MessageT>()};
  }

  AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(MessageT){std::make_shared<MessageT>()};
  }

  void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message) override
  {
    publisher_->publish(std::move(message).move_ros2_ptr());
  }

  void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message) override
  {
    publisher_->publish(*message);
  }

  // See the comment on Publisher::publish(const MessageT &) for why this exists.
  void publish(const MessageT & data) override { publisher_->publish(data); }

  uint32_t get_subscription_count() const override { return publisher_->get_subscription_count(); }
  uint32_t get_intra_process_subscription_count() const override
  {
    return publisher_->get_intra_process_subscription_count();
  }
  const rmw_gid_t & get_gid() const override { return publisher_->get_gid(); }
  const char * get_topic_name() const override { return publisher_->get_topic_name(); }
};

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  agnocast::PublisherOptions options;
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(node, topic_name, qos, options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(node, topic_name, qos, options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth)
{
  agnocast::PublisherOptions options;
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
  const agnocast::PublisherOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(node, topic_name, qos, options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(node, topic_name, qos, options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth,
  const agnocast::PublisherOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  }
}

class Timer
{
public:
  using SharedPtr = std::shared_ptr<Timer>;
  virtual ~Timer() = default;

  virtual void cancel() = 0;
  virtual void reset() = 0;
  virtual bool is_canceled() const = 0;
  virtual void set_period(std::chrono::nanoseconds period) = 0;
  virtual std::chrono::nanoseconds time_until_trigger() const = 0;
};

class AgnocastTimer : public Timer
{
  std::shared_ptr<agnocast::TimerBase> timer_;

public:
  explicit AgnocastTimer(std::shared_ptr<agnocast::TimerBase> timer) : timer_(std::move(timer)) {}

  void cancel() override { timer_->cancel(); }
  void reset() override { timer_->reset(); }
  bool is_canceled() const override { return timer_->is_canceled(); }
  void set_period(std::chrono::nanoseconds period) override { timer_->set_period(period); }
  std::chrono::nanoseconds time_until_trigger() const override
  {
    return timer_->time_until_trigger();
  }
};

class ROS2Timer : public Timer
{
  rclcpp::TimerBase::SharedPtr timer_;

public:
  explicit ROS2Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}

  void cancel() override { timer_->cancel(); }
  void reset() override { timer_->reset(); }
  bool is_canceled() const override { return timer_->is_canceled(); }
  // rclcpp::TimerBase does not expose a set_period API; fall back to the rcl C API and
  // convert the rcl_ret_t to an rclcpp::exceptions::RCLError (matching the throw style used
  // by the other rclcpp timer methods such as cancel/reset/time_until_trigger).
  void set_period(std::chrono::nanoseconds period) override
  {
    int64_t old_period = 0;
    const rcl_ret_t ret =
      rcl_timer_exchange_period(timer_->get_timer_handle().get(), period.count(), &old_period);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't exchange_period");
    }
  }
  std::chrono::nanoseconds time_until_trigger() const override
  {
    return timer_->time_until_trigger();
  }
};

template <typename DurationRepT, typename DurationT, typename CallbackT>
Timer::SharedPtr create_timer(
  rclcpp::Node * node, std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2Timer>(rclcpp::create_timer(
    node, node->get_clock(), period, std::forward<CallbackT>(callback), group));
}

template <typename DurationRepT, typename DurationT, typename CallbackT>
Timer::SharedPtr create_timer(
  agnocast::Node * node, std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<AgnocastTimer>(agnocast::create_timer(
    node, node->get_clock(), rclcpp::Duration(period),
    std::forward<CallbackT>(callback), group));
}

template <typename DurationRepT, typename DurationT, typename CallbackT>
Timer::SharedPtr create_wall_timer(
  rclcpp::Node * node, std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2Timer>(
    node->create_wall_timer(period, std::forward<CallbackT>(callback), group));
}

template <typename DurationRepT, typename DurationT, typename CallbackT>
Timer::SharedPtr create_wall_timer(
  agnocast::Node * node, std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<AgnocastTimer>(
    node->create_wall_timer(period, std::forward<CallbackT>(callback), group));
}

#if 0  // Client/Service wrappers disabled for the initial port.
// ===================== Client =====================
// Unified callback signature: receives `std::shared_ptr<const Response>`. In Agnocast mode the
// underlying ipc_shared_ptr is held alive through an aliased shared_ptr so zero-copy semantics
// are preserved for the duration the user retains the response.
template <typename ServiceT>
class Client
{
public:
  using SharedPtr = std::shared_ptr<Client<ServiceT>>;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using SharedRequest = std::shared_ptr<Request>;
  using SharedResponse = std::shared_ptr<const Response>;
  using ResponseCallback = std::function<void(SharedResponse)>;

  virtual ~Client() = default;
  virtual bool wait_for_service(std::chrono::nanoseconds timeout) = 0;
  virtual void async_send_request(
    const SharedRequest & request, ResponseCallback callback) = 0;
};

template <typename ServiceT>
class AgnocastClient : public Client<ServiceT>
{
  typename agnocast::Client<ServiceT>::SharedPtr client_;

public:
  AgnocastClient(
    agnocast::Node * node, const std::string & service_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : client_(node->template create_client<ServiceT>(service_name, qos, group))
  {
  }

  bool wait_for_service(std::chrono::nanoseconds timeout) override
  {
    return client_->wait_for_service(timeout);
  }

  void async_send_request(
    const typename Client<ServiceT>::SharedRequest & request,
    typename Client<ServiceT>::ResponseCallback callback) override
  {
    // agnocast::Client expects an ipc_shared_ptr<Request>; borrow a loaned request and copy
    // the caller's data into shared memory. The copy is acceptable because service calls
    // are low-frequency; bulk IPC payloads should use pub/sub instead.
    auto ipc_request = client_->borrow_loaned_request();
    // ipc_request points at agnocast::Client::RequestT, which derives from ServiceT::Request.
    // Cast to the base so the generated message assignment operator is callable.
    static_cast<typename ServiceT::Request &>(*ipc_request) = *request;
    client_->async_send_request(
      std::move(ipc_request),
      [cb = std::move(callback)](typename agnocast::Client<ServiceT>::SharedFuture future) {
        auto ipc_ptr = future.get();
        auto holder =
          std::make_shared<agnocast::ipc_shared_ptr<typename ServiceT::Response>>(
            std::move(ipc_ptr));
        cb(std::shared_ptr<const typename ServiceT::Response>(holder, holder->get()));
      });
  }
};

template <typename ServiceT>
class ROS2Client : public Client<ServiceT>
{
  typename rclcpp::Client<ServiceT>::SharedPtr client_;

public:
  ROS2Client(
    rclcpp::Node * node, const std::string & service_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : client_(node->template create_client<ServiceT>(
      service_name, qos.get_rmw_qos_profile(), group))
  {
  }

  bool wait_for_service(std::chrono::nanoseconds timeout) override
  {
    return client_->wait_for_service(timeout);
  }

  void async_send_request(
    const typename Client<ServiceT>::SharedRequest & request,
    typename Client<ServiceT>::ResponseCallback callback) override
  {
    client_->async_send_request(
      request,
      [cb = std::move(callback)](typename rclcpp::Client<ServiceT>::SharedFuture future) {
        cb(future.get());
      });
  }
};

template <typename ServiceT>
typename Client<ServiceT>::SharedPtr create_client(
  rclcpp::Node * node, const std::string & service_name,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2Client<ServiceT>>(node, service_name, qos, group);
}

template <typename ServiceT>
typename Client<ServiceT>::SharedPtr create_client(
  agnocast::Node * node, const std::string & service_name,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<AgnocastClient<ServiceT>>(node, service_name, qos, group);
}

// ===================== Service =====================
// Unified callback signature: `void(std::shared_ptr<Request const>, std::shared_ptr<Response>)`.
// In Agnocast mode the underlying ipc_shared_ptrs are wrapped in aliased std::shared_ptr so
// the user-supplied callback writes directly into the shared-memory response.
template <typename ServiceT>
class Service
{
public:
  using SharedPtr = std::shared_ptr<Service<ServiceT>>;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using SharedRequest = std::shared_ptr<const Request>;
  using SharedResponse = std::shared_ptr<Response>;
  using CallbackType = std::function<void(SharedRequest, SharedResponse)>;

  virtual ~Service() = default;
};

template <typename ServiceT>
class AgnocastService : public Service<ServiceT>
{
  typename agnocast::Service<ServiceT>::SharedPtr service_;

public:
  AgnocastService(
    agnocast::Node * node, const std::string & service_name,
    typename Service<ServiceT>::CallbackType callback, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  {
    service_ = node->template create_service<ServiceT>(
      service_name,
      [cb = std::move(callback)](
        const agnocast::ipc_shared_ptr<const typename ServiceT::Request> & req,
        agnocast::ipc_shared_ptr<typename ServiceT::Response> & res) {
        // Non-owning aliases so the callback can read/write through shared_ptr while the
        // underlying ipc_shared_ptr continues to own the memory.
        auto req_alias =
          std::shared_ptr<const typename ServiceT::Request>(std::shared_ptr<void>{}, req.get());
        auto res_alias =
          std::shared_ptr<typename ServiceT::Response>(std::shared_ptr<void>{}, res.get());
        cb(req_alias, res_alias);
      },
      qos, group);
  }
};

template <typename ServiceT>
class ROS2Service : public Service<ServiceT>
{
  typename rclcpp::Service<ServiceT>::SharedPtr service_;

public:
  ROS2Service(
    rclcpp::Node * node, const std::string & service_name,
    typename Service<ServiceT>::CallbackType callback, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  {
    service_ = node->template create_service<ServiceT>(
      service_name,
      [cb = std::move(callback)](
        const std::shared_ptr<typename ServiceT::Request> req,
        std::shared_ptr<typename ServiceT::Response> res) { cb(req, res); },
      qos.get_rmw_qos_profile(), group);
  }
};

template <typename ServiceT>
typename Service<ServiceT>::SharedPtr create_service(
  rclcpp::Node * node, const std::string & service_name,
  typename Service<ServiceT>::CallbackType callback,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2Service<ServiceT>>(
    node, service_name, std::move(callback), qos, group);
}

template <typename ServiceT>
typename Service<ServiceT>::SharedPtr create_service(
  agnocast::Node * node, const std::string & service_name,
  typename Service<ServiceT>::CallbackType callback,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<AgnocastService<ServiceT>>(
    node, service_name, std::move(callback), qos, group);
}
#endif  // Client/Service wrappers disabled

}  // namespace autoware::agnocast_wrapper

#else

#include "autoware_utils_rclcpp/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl/timer.h>
#include <rclcpp/exceptions/exceptions.hpp>

#include <chrono>
#include <memory>
#include <type_traits>

namespace autoware::agnocast_wrapper
{

class Timer
{
public:
  using SharedPtr = std::shared_ptr<Timer>;
  virtual ~Timer() = default;

  virtual void cancel() = 0;
  virtual void reset() = 0;
  virtual bool is_canceled() const = 0;
  virtual void set_period(std::chrono::nanoseconds period) = 0;
  virtual std::chrono::nanoseconds time_until_trigger() const = 0;
};

class ROS2Timer : public Timer
{
  rclcpp::TimerBase::SharedPtr timer_;

public:
  explicit ROS2Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}

  void cancel() override { timer_->cancel(); }
  void reset() override { timer_->reset(); }
  bool is_canceled() const override { return timer_->is_canceled(); }
  // rclcpp::TimerBase does not expose a set_period API; fall back to the rcl C API and
  // convert the rcl_ret_t to an rclcpp::exceptions::RCLError (matching the throw style used
  // by the other rclcpp timer methods such as cancel/reset/time_until_trigger).
  void set_period(std::chrono::nanoseconds period) override
  {
    int64_t old_period = 0;
    const rcl_ret_t ret =
      rcl_timer_exchange_period(timer_->get_timer_handle().get(), period.count(), &old_period);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't exchange_period");
    }
  }
  std::chrono::nanoseconds time_until_trigger() const override
  {
    return timer_->time_until_trigger();
  }
};

template <typename DurationRepT, typename DurationT, typename CallbackT>
Timer::SharedPtr create_timer(
  rclcpp::Node * node, std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2Timer>(rclcpp::create_timer(
    node, node->get_clock(), period, std::forward<CallbackT>(callback), group));
}

template <typename DurationRepT, typename DurationT, typename CallbackT>
Timer::SharedPtr create_wall_timer(
  rclcpp::Node * node, std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2Timer>(
    node->create_wall_timer(period, std::forward<CallbackT>(callback), group));
}

}  // namespace autoware::agnocast_wrapper

#define AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) std::unique_ptr<MessageT>
// For publisher (mutable message)
#define AUTOWARE_MESSAGE_SHARED_PTR(MessageT) std::shared_ptr<MessageT>
// For subscription (read-only message)
#define AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) std::shared_ptr<const MessageT>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) typename rclcpp::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) typename rclcpp::Publisher<MessageT>::SharedPtr
#define AUTOWARE_POLLING_SUBSCRIBER_PTR(MessageT) \
  typename autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT>::SharedPtr
#define AUTOWARE_TIMER_PTR autoware::agnocast_wrapper::Timer::SharedPtr
#if 0  // Client/Service wrappers disabled for the initial port.
#define AUTOWARE_CLIENT_PTR(ServiceT) typename rclcpp::Client<ServiceT>::SharedPtr
#define AUTOWARE_SERVICE_PTR(ServiceT) typename rclcpp::Service<ServiceT>::SharedPtr
#endif

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  this->create_subscription<message_type>(topic, qos, callback, options)
#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  this->create_publisher<message_type>(arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  this->create_publisher<message_type>(arg1, arg2, arg3)
#define AUTOWARE_CREATE_POLLING_SUBSCRIBER(message_type, topic, qos)                       \
  autoware_utils_rclcpp::InterProcessPollingSubscriber<message_type>::create_subscription( \
    this, topic, qos)
#define AUTOWARE_CREATE_TIMER(period, callback) \
  autoware::agnocast_wrapper::create_timer(this, period, callback)
#define AUTOWARE_CREATE_WALL_TIMER(period, callback) \
  autoware::agnocast_wrapper::create_wall_timer(this, period, callback)

#define AUTOWARE_SUBSCRIPTION_OPTIONS rclcpp::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS rclcpp::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) \
  std::make_unique<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) \
  std::make_shared<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()

#endif
