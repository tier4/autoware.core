// Copyright 2024 TIER IV, Inc.
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

#include <autoware/planning_factor_interface/planning_factor_builder.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::planning_factor_interface::PlanningFactorBuilder;
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware::planning_factor_interface::Pose;
using autoware::planning_factor_interface::SafetyFactorArray;

namespace
{

Pose make_pose(double x, double y)
{
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = 0.0;
  return p;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> make_straight_path(
  double length, double step = 1.0)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> pts;
  for (double x = 0.0; x <= length + 1e-6; x += step) {
    autoware_planning_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = x;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 0.0;
    pts.push_back(pt);
  }
  return pts;
}

}  // namespace

// ── PlanningFactorBuilder (rclcpp-free) ─────────────────────────────────────

TEST(PlanningFactorBuilderTest, NameIsSetCorrectly)
{
  PlanningFactorBuilder b("my_module");
  EXPECT_EQ(b.name(), "my_module");
}

TEST(PlanningFactorBuilderTest, InitiallyEmpty)
{
  PlanningFactorBuilder b("m");
  EXPECT_TRUE(b.get_factors().empty());
}

TEST(PlanningFactorBuilderTest, AddSinglePointNonTemplate)
{
  PlanningFactorBuilder b("m");
  SafetyFactorArray sf;
  b.add(3.0, make_pose(3.0, 0.0), 0u, sf);
  const auto factors = b.get_factors();
  ASSERT_EQ(factors.size(), 1u);
  EXPECT_EQ(factors[0].module, "m");
  ASSERT_EQ(factors[0].control_points.size(), 1u);
  EXPECT_FLOAT_EQ(factors[0].control_points[0].distance, 3.0f);
}

TEST(PlanningFactorBuilderTest, AddTwoPointsNonTemplate)
{
  PlanningFactorBuilder b("m");
  SafetyFactorArray sf;
  b.add(1.0, 4.0, make_pose(1.0, 0.0), make_pose(4.0, 0.0), 0u, sf);
  const auto factors = b.get_factors();
  ASSERT_EQ(factors.size(), 1u);
  ASSERT_EQ(factors[0].control_points.size(), 2u);
  EXPECT_FLOAT_EQ(factors[0].control_points[0].distance, 1.0f);
  EXPECT_FLOAT_EQ(factors[0].control_points[1].distance, 4.0f);
}

TEST(PlanningFactorBuilderTest, MultipleAddAccumulates)
{
  PlanningFactorBuilder b("m");
  SafetyFactorArray sf;
  b.add(1.0, make_pose(1.0, 0.0), 0u, sf);
  b.add(2.0, make_pose(2.0, 0.0), 0u, sf);
  b.add(3.0, make_pose(3.0, 0.0), 0u, sf);
  EXPECT_EQ(b.get_factors().size(), 3u);
}

TEST(PlanningFactorBuilderTest, MakeArraySetsHeaderAndFactors)
{
  PlanningFactorBuilder b("m");
  SafetyFactorArray sf;
  b.add(1.0, make_pose(1.0, 0.0), 0u, sf);

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 0u;
  const auto arr = b.make_array(stamp);

  EXPECT_EQ(arr.header.frame_id, "map");
  EXPECT_EQ(arr.header.stamp.sec, 42);
  ASSERT_EQ(arr.factors.size(), 1u);
}

TEST(PlanningFactorBuilderTest, MakeArrayDoesNotClear)
{
  PlanningFactorBuilder b("m");
  SafetyFactorArray sf;
  b.add(1.0, make_pose(1.0, 0.0), 0u, sf);

  builtin_interfaces::msg::Time stamp;
  b.make_array(stamp);

  EXPECT_EQ(b.get_factors().size(), 1u);
}

TEST(PlanningFactorBuilderTest, ClearEmptiesFactors)
{
  PlanningFactorBuilder b("m");
  SafetyFactorArray sf;
  b.add(1.0, make_pose(1.0, 0.0), 0u, sf);
  b.clear();
  EXPECT_TRUE(b.get_factors().empty());
}

TEST(PlanningFactorBuilderTest, TemplateAddSinglePointComputesDistance)
{
  PlanningFactorBuilder b("m");
  const auto path = make_straight_path(10.0);
  SafetyFactorArray sf;
  b.add(path, make_pose(0.0, 0.0), make_pose(5.0, 0.0), 0u, sf);
  const auto factors = b.get_factors();
  ASSERT_EQ(factors.size(), 1u);
  ASSERT_EQ(factors[0].control_points.size(), 1u);
  EXPECT_NEAR(factors[0].control_points[0].distance, 5.0f, 0.1f);
}

TEST(PlanningFactorBuilderTest, TemplateAddTwoPointsComputesDistances)
{
  PlanningFactorBuilder b("m");
  const auto path = make_straight_path(10.0);
  SafetyFactorArray sf;
  b.add(path, make_pose(0.0, 0.0), make_pose(2.0, 0.0), make_pose(7.0, 0.0), 0u, sf);
  const auto factors = b.get_factors();
  ASSERT_EQ(factors.size(), 1u);
  ASSERT_EQ(factors[0].control_points.size(), 2u);
  EXPECT_NEAR(factors[0].control_points[0].distance, 2.0f, 0.1f);
  EXPECT_NEAR(factors[0].control_points[1].distance, 7.0f, 0.1f);
}

// ── PlanningFactorInterface (rclcpp wrapper) ─────────────────────────────────

class PlanningFactorInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_node");
    interface_ = std::make_unique<PlanningFactorInterface>(node_.get(), "test");
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<PlanningFactorInterface> interface_;
};

TEST_F(PlanningFactorInterfaceTest, AddForwardsToBuilder)
{
  SafetyFactorArray sf;
  interface_->add(2.0, make_pose(2.0, 0.0), 0u, sf);
  const auto factors = interface_->get_factors();
  ASSERT_EQ(factors.size(), 1u);
  EXPECT_FLOAT_EQ(factors[0].control_points[0].distance, 2.0f);
}

TEST_F(PlanningFactorInterfaceTest, PublishClearsFactors)
{
  SafetyFactorArray sf;
  interface_->add(1.0, make_pose(1.0, 0.0), 0u, sf);
  ASSERT_EQ(interface_->get_factors().size(), 1u);
  interface_->publish();
  EXPECT_TRUE(interface_->get_factors().empty());
}

TEST_F(PlanningFactorInterfaceTest, TemplateAddForwardsToBuilder)
{
  const auto path = make_straight_path(10.0);
  SafetyFactorArray sf;
  interface_->add(path, make_pose(0.0, 0.0), make_pose(3.0, 0.0), 0u, sf);
  const auto factors = interface_->get_factors();
  ASSERT_EQ(factors.size(), 1u);
  EXPECT_NEAR(factors[0].control_points[0].distance, 3.0f, 0.1f);
}

TEST_F(PlanningFactorInterfaceTest, GetFactorsReturnsEmptyAfterClear)
{
  SafetyFactorArray sf;
  interface_->add(1.0, make_pose(1.0, 0.0), 0u, sf);
  interface_->add(2.0, make_pose(2.0, 0.0), 0u, sf);
  interface_->publish();
  EXPECT_TRUE(interface_->get_factors().empty());
}

TEST_F(PlanningFactorInterfaceTest, PublishExternalBuilderClearsIt)
{
  PlanningFactorBuilder external("external_module");
  SafetyFactorArray sf;
  external.add(5.0, make_pose(5.0, 0.0), 0u, sf);
  ASSERT_EQ(external.get_factors().size(), 1u);
  interface_->publish(external);
  EXPECT_TRUE(external.get_factors().empty());
}

TEST_F(PlanningFactorInterfaceTest, PublishExternalBuilderDoesNotAffectInternal)
{
  SafetyFactorArray sf;
  interface_->add(1.0, make_pose(1.0, 0.0), 0u, sf);

  PlanningFactorBuilder external("external_module");
  external.add(5.0, make_pose(5.0, 0.0), 0u, sf);
  interface_->publish(external);

  // internal builder_ is untouched
  EXPECT_EQ(interface_->get_factors().size(), 1u);
}

