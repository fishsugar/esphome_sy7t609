#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/datatypes.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <optional>
#include <span>
#include <vector>
#include "sy7t609_def.h"

namespace esphome::sy7t609 {

struct BatchReadItem {
  RegAddr addr;
  size_t offset;
};

struct BatchReadInfo {
  size_t resp_len;
  std::vector<uint8_t> req_buffer;
  std::vector<BatchReadItem> items;
};

class SY7T609Component : public PollingComponent, public uart::UARTDevice {
  SUB_SENSOR(chip_temperature)
  SUB_SENSOR(voltage)
  SUB_SENSOR(vavg)
  SUB_SENSOR(vfund)
  SUB_SENSOR(vharm)
  SUB_SENSOR(vpeak)
  SUB_SENSOR(current)
  SUB_SENSOR(iavg)
  SUB_SENSOR(ifund)
  SUB_SENSOR(iharm)
  SUB_SENSOR(ipeak)
  SUB_SENSOR(active_power)
  SUB_SENSOR(avgpower)
  SUB_SENSOR(pfund)
  SUB_SENSOR(pharm)
  SUB_SENSOR(reactive_power)
  SUB_SENSOR(qfund)
  SUB_SENSOR(qharm)
  SUB_SENSOR(apparent_power)
  SUB_SENSOR(vafund)
  SUB_SENSOR(vaharm)
  SUB_SENSOR(line_frequency)
  SUB_SENSOR(power_factor)
  SUB_SENSOR(positive_active_energy)
  SUB_SENSOR(negative_active_energy)
  SUB_SENSOR(net_active_energy)
  SUB_SENSOR(net_reactive_energy)
  SUB_SENSOR(net_apparent_energy);

 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_fsv(float value) { this->scale_parameters_.fsv = value; }
  void set_fsi(float value) { this->scale_parameters_.fsi = value; }
  void set_batch_size(float value) { this->batch_size_ = value; }
  void set_calibration_timeout_ms(float value) { this->calibration_timeout_ms_ = value; }

  void save_to_flash();
  void set_auto_report(bool on_off);
  void clear_energy_counters();
  void soft_reset();
  bool calibrate_voltage_gain(float vrms_target_volts);
  bool calibrate_current_gain(float irms_target_amps);
  bool calibrate_voltage_offset(float vavg_target_volts);
  bool calibrate_current_offset(float iavg_target_amps);
  bool calibrate_temperature(float temp_celsius);

  std::optional<RegValue24> ssi_read_reg_sync(RegAddr addr);
  bool ssi_write_reg_sync(RegAddr addr, RegValue24 value);

 protected:
  BatchReadInfo build_batch_read_(std::span<const RegAddr> regs);
  bool ssi_batch_read_sync_(const BatchReadInfo &batch, std::vector<uint8_t> &resp_buffer);
  bool invoke_command_sync_(CommandRegistryCode cmd);
  bool load_scale_registers_();
  void wait_active_query_end_();
  void discard_rx_buffer_();
  void update_scale_cache_();

 private:
  void sensor_query_start_();
  void sensor_query_loop_();
  bool sensor_query_receive_packet_(const BatchReadInfo &batch);
  bool sensor_query_advance_();
  void sensor_query_done_();
  void sensor_query_fail_();

 protected:
  struct {
    uint32_t iscale;
    uint32_t vscale;
    uint32_t pscale;
    uint32_t pfscale;
    uint32_t tscale;
    uint32_t fscale;
    ControlRegister control;
    double bucket;
  } scale_registers_{};

  struct {
    float fsv;  ///< Full scale voltage
    float fsi;  ///< Full scale current
  } scale_parameters_{};

  struct {
    float ifactor;
    float vfactor;
    float pfactor;
    float pffactor;
    float tfactor;
    float ffactor;
    double efactor;
  } scale_cache_{};

  /// @brief Max number of registers to read in one batch.
  ///
  /// Setting this value too high may cause SY7T609 fail to respond the request.
  /// Behavior not documented in datasheet.
  size_t batch_size_{14};

  /// @brief Calibration timeout in milliseconds.
  ///
  /// See the datasheet section "5.5.5 Calibration Command" for details about the calibration process.
  uint32_t calibration_timeout_ms_{20 * 1000};

 private:
  struct SensorQueryState {
    bool active;
    int step;
    uint32_t time_track;
  };

  std::vector<BatchReadInfo> batch_sensor_query_;
  SensorQueryState sensor_query_state_;
};

template<typename... Ts> class ClearEnergyCountersAction : public Action<Ts...>, public Parented<SY7T609Component> {
 public:
  void play(const Ts &...x) override { this->parent_->clear_energy_counters(); }
};

template<typename... Ts> class SoftResetAction : public Action<Ts...>, public Parented<SY7T609Component> {
 public:
  void play(const Ts &...x) override { this->parent_->soft_reset(); }
};

}  // namespace esphome::sy7t609
