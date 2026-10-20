// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rs_driver/msg/safety_airy_status.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace robosense { namespace lidar {

inline std::string safetyAiryHex(const uint8_t* bytes, size_t count)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < count; ++i) out << std::setw(2) << unsigned(bytes[i]);
  return out.str();
}

inline diagnostic_msgs::msg::DiagnosticArray toSafetyAiryDiagnostics(
    const SafetyAiryStatus& status, double age, const std::string& frame)
{
  using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
  diagnostic_msgs::msg::DiagnosticArray message;
  const int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  message.header.stamp.sec = ns / 1000000000;
  message.header.stamp.nanosec = ns % 1000000000;
  message.header.frame_id = frame;
  Diagnostic diag;
  diag.name = frame + "/safety_airy";
  diag.hardware_id = status.valid ? safetyAiryHex(status.serial.data(), 6) : "unknown";
  diag.level = Diagnostic::OK;
  diag.message = "Operational diagnostic telemetry";
  bool fault = status.fault_level != 0 || status.system_state == 0xc3;
  // Appendix B.24 defines only the first ten bytes as faults; retain the
  // remaining reserved bytes in raw telemetry without assigning a meaning.
  for (size_t i = 0; i < 10; ++i) fault |= status.faults[i] != 0;
  bool all_ossd_on = true;
  for (const auto byte : status.ossd) all_ossd_on &= byte == 0xaa;
  if (!status.valid || age > 0.5)
  {
    diag.level = Diagnostic::STALE;
    diag.message = status.valid ? "No valid DIFOP received for more than 0.5 s" : "Waiting for valid DIFOP";
  }
  else if (fault || !status.calibration_valid || status.protected_intrusion == 0xaa)
  {
    diag.level = Diagnostic::ERROR;
    diag.message = fault ? "Sensor reports fault" :
                   !status.calibration_valid ? "Calibration unavailable or invalid" : "Protected field intrusion";
  }
  else if (status.system_state != 0x33 || !all_ossd_on ||
           status.warning_intrusion != 0 || status.protected_intrusion != 0 || status.occlusion != 0)
  {
    diag.level = Diagnostic::WARN;
    diag.message = "Inspect raw state, OSSD and field indicators";
  }
  const auto add = [&diag](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key; entry.value = value; diag.values.emplace_back(entry);
  };
  add("diagnostic_only", "true; not a safety permit");
  add("difop_valid", status.valid ? "true" : "false");
  add("difop_age_seconds", std::to_string(age));
  add("calibration_valid", status.calibration_valid ? "true" : "false");
  add("imu_extrinsic_valid", status.imu_extrinsic_valid ? "true" : "false");
  add("system_state_hex", safetyAiryHex(&status.system_state, 1));
  add("fault_level_hex", safetyAiryHex(&status.fault_level, 1));
  add("active_group_raw", std::to_string(status.active_group));
  add("sync_mode_raw", std::to_string(status.sync_mode));
  add("sync_status_raw", std::to_string(status.sync_status));
  add("warning_intrusion_hex", safetyAiryHex(&status.warning_intrusion, 1));
  add("protected_intrusion_hex", safetyAiryHex(&status.protected_intrusion, 1));
  add("occlusion_hex", safetyAiryHex(&status.occlusion, 1));
  add("fault_bytes_hex", safetyAiryHex(status.faults.data(), status.faults.size()));
  add("rpm_setting", std::to_string(status.rpm));
  add("rpm_actual", std::to_string(status.actual_rpm));
  const char* ossd_names[] = {"ossd_1a_hex", "ossd_2a_hex", "ossd_1b_hex", "ossd_2b_hex"};
  for (size_t i = 0; i < 4; ++i) add(ossd_names[i], safetyAiryHex(&status.ossd[i], 1));
  const char* firmware_names[] = {"firmware_overall", "firmware_top", "firmware_bottom",
                                "firmware_app", "firmware_motor", "firmware_cgi"};
  for (size_t i = 0; i < 6; ++i) add(firmware_names[i], safetyAiryHex(&status.firmware[4 * i], 4));
  const char* extrinsic_names[] = {"imu_qx", "imu_qy", "imu_qz", "imu_qw", "imu_x", "imu_y", "imu_z"};
  for (size_t i = 0; i < 7; ++i) add(extrinsic_names[i], std::to_string(status.imu_extrinsic[i]));
  add("accepted_msop_packets", std::to_string(status.msop_packets));
  add("accepted_difop_packets", std::to_string(status.difop_packets));
  add("accepted_imu_packets", std::to_string(status.imu_packets));
  add("crc_errors", std::to_string(status.crc_errors));
  add("rejected_packet_contents", std::to_string(status.rejected_packets));
  add("msop_sequence_gaps", std::to_string(status.sequence_gaps));
  message.status.emplace_back(std::move(diag));
  return message;
}

}}  // namespace robosense::lidar
