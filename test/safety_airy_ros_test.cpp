// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "source/source_pointcloud_ros.hpp"

using namespace robosense::lidar;

TEST(SafetyAiryRos, CloudPreservesRingsTimesAndRawFlagsWithAlignedLayout)
{
  LidarPointCloudMsg cloud;
  cloud.timestamp = 10.0; cloud.height = 1; cloud.width = 2; cloud.is_dense = true;
  LidarPointCloudMsg::PointT point{};
  point.x = 1.0; point.y = 2.0; point.z = 3.0; point.intensity = 17;
  point.ring = 95; point.timestamp = 10.001; point.feature = 2;
  cloud.points = {point, point};
  auto message = toRosMsg(cloud, "test", false, true, true);
  ASSERT_EQ(message.fields.size(), 7u);
  EXPECT_EQ(message.fields[5].name, "t"); EXPECT_EQ(message.fields[5].offset, 20u);
  EXPECT_EQ(message.fields[6].name, "feature"); EXPECT_EQ(message.fields[6].offset, 24u);
  EXPECT_EQ(message.point_step, 28u);
  ASSERT_EQ(message.data.size(), 56u);
  sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
  EXPECT_FLOAT_EQ(*x, 1.0f); ++x; EXPECT_FLOAT_EQ(*x, 1.0f);
  sensor_msgs::PointCloud2ConstIterator<uint16_t> ring(message, "ring");
  EXPECT_EQ(*ring, 95);
  sensor_msgs::PointCloud2ConstIterator<uint32_t> t(message, "t");
  EXPECT_NEAR(*t, 1000000, 1);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> flags(message, "feature");
  EXPECT_EQ(*flags, 2);
  auto legacy = toRosMsg(cloud, "test", false, true);
  EXPECT_EQ(legacy.point_step, 22u);
  EXPECT_EQ(legacy.fields[5].offset, 18u);
}

TEST(SafetyAiryRos, ImuKeepsSiUnitsAndMarksOrientationUnavailable)
{
  auto input = std::make_shared<ImuData>();
  input->timestamp = 10.0;
  input->linear_acceleration_z = 9.80665f;
  input->angular_velocity_z = 0.1f;
  auto message = toRosMsg(input, "raw_imu");
  EXPECT_NEAR(message.linear_acceleration.z, 9.80665, 1e-6);
  EXPECT_NEAR(message.angular_velocity.z, 0.1, 1e-7);
  EXPECT_EQ(message.orientation_covariance[0], -1);
  EXPECT_EQ(message.header.frame_id, "raw_imu");
}

TEST(SafetyAiryRos, DiagnosticsDistinguishMissingStaleFaultAndOperationalTelemetry)
{
  using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
  SafetyAiryStatus status;
  EXPECT_EQ(toSafetyAiryDiagnostics(status, -1, "test").status[0].level, Diagnostic::STALE);
  status.valid = status.calibration_valid = true;
  status.system_state = 0x33;
  status.ossd.fill(0xaa);
  EXPECT_EQ(toSafetyAiryDiagnostics(status, 0.01, "test").status[0].level, Diagnostic::OK);
  status.faults[10] = 4;  // Reserved bytes are not decoded as faults.
  EXPECT_EQ(toSafetyAiryDiagnostics(status, 0.01, "test").status[0].level, Diagnostic::OK);
  status.ossd[0] = 0;
  EXPECT_EQ(toSafetyAiryDiagnostics(status, 0.01, "test").status[0].level, Diagnostic::WARN);
  status.faults[0] = 4;
  EXPECT_EQ(toSafetyAiryDiagnostics(status, 0.01, "test").status[0].level, Diagnostic::ERROR);
  EXPECT_EQ(toSafetyAiryDiagnostics(status, 0.51, "test").status[0].level, Diagnostic::STALE);
}
