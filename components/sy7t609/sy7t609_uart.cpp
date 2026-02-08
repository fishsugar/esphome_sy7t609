#include "sy7t609_uart.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include <array>
#include <cinttypes>
#include <cmath>
#include <numeric>
#include <span>
#include <utility>

namespace esphome::sy7t609 {

static const char *const TAG = "sy7t609";

namespace {

RegValue24 read_reg24(const uint8_t *array) {
  uint32_t raw = ((uint32_t) array[2] << 16) | ((uint32_t) array[1] << 8) | (uint32_t) array[0];
  return RegValue24{raw};
}

uint8_t sy7t609_checksum(std::span<const uint8_t> data) {
  // Sum all bytes except the last one (reserved for the checksum itself)
  uint8_t sum = std::accumulate(data.begin(), data.end() - 1, static_cast<uint8_t>(0));

  // Two's complement: Invert and add 1 (same as negative sign in 2's complement)
  uint8_t checksum = static_cast<uint8_t>(-sum);
  return checksum;
}

}  // namespace

void SY7T609Component::setup() {
  if (!this->load_scale_registers_()) {
    this->mark_failed(LOG_STR("Failed to load scale registers"));
    return;
  }

  static constexpr std::pair<esphome::sensor::Sensor * SY7T609Component::*, RegAddr> SENSOR_REG_MAP[]{
      {&SY7T609Component::chip_temperature_sensor_, REG_CTEMP},
      {&SY7T609Component::voltage_sensor_, REG_VRMS},
      {&SY7T609Component::vavg_sensor_, REG_VAVG},
      {&SY7T609Component::vfund_sensor_, REG_VFUND},
      {&SY7T609Component::vharm_sensor_, REG_VHARM},
      {&SY7T609Component::vpeak_sensor_, REG_VPEAK},
      {&SY7T609Component::current_sensor_, REG_IRMS},
      {&SY7T609Component::iavg_sensor_, REG_IAVG},
      {&SY7T609Component::ifund_sensor_, REG_IFUND},
      {&SY7T609Component::iharm_sensor_, REG_IHARM},
      {&SY7T609Component::ipeak_sensor_, REG_IPEAK},
      {&SY7T609Component::active_power_sensor_, REG_POWER},
      {&SY7T609Component::avgpower_sensor_, REG_AVGPOWER},
      {&SY7T609Component::pfund_sensor_, REG_PFUND},
      {&SY7T609Component::pharm_sensor_, REG_PHARM},
      {&SY7T609Component::reactive_power_sensor_, REG_VAR},
      {&SY7T609Component::qfund_sensor_, REG_QFUND},
      {&SY7T609Component::qharm_sensor_, REG_QHARM},
      {&SY7T609Component::apparent_power_sensor_, REG_VA},
      {&SY7T609Component::vafund_sensor_, REG_VAFUND},
      {&SY7T609Component::vaharm_sensor_, REG_VAHARM},
      {&SY7T609Component::line_frequency_sensor_, REG_FREQUENCY},
      {&SY7T609Component::power_factor_sensor_, REG_PF},
      {&SY7T609Component::positive_active_energy_sensor_, REG_EPPCNT},
      {&SY7T609Component::negative_active_energy_sensor_, REG_EPMCNT},
      {&SY7T609Component::net_active_energy_sensor_, REG_EPNCNT},
      {&SY7T609Component::net_reactive_energy_sensor_, REG_EQNCNT},
      {&SY7T609Component::net_apparent_energy_sensor_, REG_ESNCNT},
  };

  std::vector<RegAddr> regs;
  for (const auto &p : SENSOR_REG_MAP) {
    if (this->*(p.first) != nullptr) {
      regs.push_back(p.second);
    }
  }

  // build batches
  std::vector<BatchReadInfo> batches;
  for (size_t i = 0; i < regs.size(); i += this->batch_size_) {
    size_t count = std::min(this->batch_size_, regs.size() - i);
    batches.push_back(this->build_batch_read_(std::span(regs).subspan(i, count)));
  }
  this->batch_sensor_query_ = std::move(batches);
}

void SY7T609Component::update() { this->sensor_query_start_(); }

void SY7T609Component::loop() { this->sensor_query_loop_(); }

BatchReadInfo SY7T609Component::build_batch_read_(std::span<const RegAddr> regs) {
  const size_t req_len = SSI_REQ_HEADER_SIZE +
                         (SSI_REQ_PAYLOAD_SIZE_SELECT + SSI_REQ_PAYLOAD_SIZE_READ_3BYTE) * regs.size() +
                         SSI_REQ_TRAILING_SIZE;
  if (req_len > SSI_REQ_MAX_PACKET_SIZE) {
    ESP_LOGE(TAG, "req_len exceed max length: %u > %u", req_len, SSI_REQ_MAX_PACKET_SIZE);
    return {};
  }

  std::vector<uint8_t> buffer(req_len);
  std::vector<BatchReadItem> items(regs.size());

  auto it = buffer.begin();
  *it++ = SSI_REQ_HEADER_CODE;
  *it++ = req_len;

  size_t resp_offset = SSI_RESP_WITHDATA_HEADER_SIZE;
  for (size_t i = 0; i < regs.size(); i++) {
    const auto &reg = regs[i];
    *it++ = SSI_CMD_SELECT_REGISTER_ADDRESS;
    *it++ = reg & 0xFF;
    *it++ = (reg >> 8) & 0xFF;
    *it++ = SSI_CMD_READ_REGITSTER_3BYTES;

    items[i] = {.addr = reg, .offset = resp_offset};

    resp_offset += SSI_RESP_DATA_SIZE_3BYTE;
  }
  *it++ = sy7t609_checksum(buffer);

  size_t resp_len = resp_offset + SSI_RESP_WITHDATA_TAILING_SIZE;
  if (resp_len > SSI_REQ_MAX_PACKET_SIZE) {
    ESP_LOGE(TAG, "resp_len exceed max length: %u > %u", resp_len, SSI_REQ_MAX_PACKET_SIZE);
    return {};
  }

  ESP_LOGV(TAG, "build_batch_read: %u regs, req_len: %u, resp_len: %u", regs.size(), req_len, resp_len);
  return BatchReadInfo{.resp_len = resp_len, .req_buffer = std::move(buffer), .items = std::move(items)};
}

bool SY7T609Component::ssi_batch_read_sync_(const BatchReadInfo &batch, std::vector<uint8_t> &resp_buffer) {
  this->wait_active_query_end_();

  auto t1 = millis();
  this->write_array(batch.req_buffer);
  auto t2 = millis();
  this->flush();
  auto t3 = millis();
  resp_buffer.resize(batch.resp_len);
  if (!this->read_array(resp_buffer.data(), batch.resp_len)) {
    ESP_LOGE(TAG, "Failed to read reply");
    char buffer[format_hex_size(SSI_REQ_MAX_PACKET_SIZE)];
    ESP_LOGVV(TAG, "resp buffer: %s", format_hex_to(buffer, resp_buffer));
    return false;
  }
  auto t4 = millis();

  auto *p = resp_buffer.data();
  auto reply_code = *p++;
  if (reply_code != REPLY_CODE_ACK_WITH_DATA) {
    ESP_LOGE(TAG, "Bad reply code: 0x%02X", reply_code);
    return false;
  }

  auto len = *p++;
  if (len != batch.resp_len) {
    ESP_LOGE(TAG, "Bad reply len: %hhu, expect: %u", len, batch.resp_len);
    return false;
  }
  ESP_LOGV(TAG, "batch_read_sync timing: write: %u+%u ms, read: %u ms, total: %u ms", t2 - t1, t3 - t2, t4 - t3,
           t4 - t1);

  char buffer[format_hex_size(SSI_REQ_MAX_PACKET_SIZE)];
  ESP_LOGVV(TAG, "batch_read_sync resp: (%d bytes) %s", len, format_hex_to(buffer, resp_buffer));
  return true;
}

std::optional<RegValue24> SY7T609Component::ssi_read_reg_sync(RegAddr addr) {
  auto batch = this->build_batch_read_(std::to_array({addr}));

  std::vector<uint8_t> resp_buffer(batch.resp_len);
  if (!this->ssi_batch_read_sync_(batch, resp_buffer)) {
    return std::nullopt;
  }

  return read_reg24(resp_buffer.data() + SSI_RESP_WITHDATA_HEADER_SIZE);
}

bool SY7T609Component::ssi_write_reg_sync(RegAddr addr, RegValue24 value) {
  constexpr size_t packet_size =
      SSI_REQ_HEADER_SIZE + SSI_REQ_PAYLOAD_SIZE_SELECT + SSI_REQ_PAYLOAD_SIZE_WRITE_3BYTE + SSI_REQ_TRAILING_SIZE;

  this->wait_active_query_end_();

  std::array<uint8_t, packet_size> data{};
  data[0] = SSI_REQ_HEADER_CODE;
  data[1] = packet_size;
  data[2] = SSI_CMD_SELECT_REGISTER_ADDRESS;
  data[3] = addr & 0xFF;
  data[4] = (addr >> 8) & 0xFF;
  data[5] = SSI_CMD_WRITE_RETISTER_3BYTES;
  data[6] = value.raw & 0xFF;
  data[7] = (value.raw >> 8) & 0xFF;
  data[8] = (value.raw >> 16) & 0xFF;
  data[9] = sy7t609_checksum(data);
  this->write_array(data);
  this->flush();

  uint8_t reply_code;
  if (!this->read_byte(&reply_code)) {
    ESP_LOGE(TAG, "read reply_code failed");
    return false;
  }
  if (reply_code == REPLY_CODE_ACK_WITHOUT_DATA) {
    ESP_LOGV(TAG, "write register 0x%03X with value 0x%06X done", addr, value.raw);
    return true;
  } else if (addr == REG_COMMAND && value.raw == CMD_REG_SOFT_RESET && reply_code == 0xED) {  // undocumented reply code
    ESP_LOGV(TAG, "command soft_reset(0x%06X) done with reply 0xED", CMD_REG_SOFT_RESET);
    return true;
  } else {
    ESP_LOGE(TAG, "unexpected reply code: 0x%02X", reply_code);
    return false;
  }
}

bool SY7T609Component::load_scale_registers_() {
  constexpr RegAddr scale_regs[]{REG_ISCALE, REG_VSCALE,  REG_PSCALE,  REG_PFSCALE, REG_TSCALE,
                                 REG_FSCALE, REG_CONTROL, REG_BUCKETH, REG_BUCKETL};
  auto batch = this->build_batch_read_(scale_regs);
  std::vector<uint8_t> resp_buffer;
  if (!this->ssi_batch_read_sync_(batch, resp_buffer)) {
    return false;
  }

  double bucket = 0;
  for (const auto &item : batch.items) {
    RegValue24 v = read_reg24(resp_buffer.data() + item.offset);
    switch (item.addr) {
      case REG_ISCALE:
        this->scale_registers_.iscale = v.u24_value();
        break;
      case REG_VSCALE:
        this->scale_registers_.vscale = v.u24_value();
        break;
      case REG_PSCALE:
        this->scale_registers_.pscale = v.u24_value();
        break;
      case REG_PFSCALE:
        this->scale_registers_.pfscale = v.u24_value();
        break;
      case REG_TSCALE:
        this->scale_registers_.tscale = v.u24_value();
        break;
      case REG_FSCALE:
        this->scale_registers_.fscale = v.u24_value();
        break;
      case REG_CONTROL: {
        ControlRegister control;
        control.raw = v.u24_value();
        this->scale_registers_.control = control;
      } break;
      case REG_BUCKETH:
        bucket += v.u24_value();
        break;
      case REG_BUCKETL:
        bucket += v.u24_value<24>();
        break;
      default:
        break;
    }
  }
  this->scale_registers_.bucket = bucket;
  this->update_scale_cache_();
  ESP_LOGD(TAG, "Scale registers loaded");
  return true;
}

void SY7T609Component::sensor_query_start_() {
  // Blocking wait if previous query not finish in time.
  // Note: read all 28 sensor registers in 2 batches takes ~230ms with UART@9600Hz.
  this->wait_active_query_end_();

  if (this->batch_sensor_query_.empty()) {
    ESP_LOGV(TAG, "skip update: no batch");
    return;
  }
  this->sensor_query_state_ = SensorQueryState{
      .active = true,
      .step = -1,
      .time_track = App.get_loop_component_start_time(),
  };
  ESP_LOGV(TAG, "sensor query task started");
  this->sensor_query_advance_();
}

bool SY7T609Component::sensor_query_advance_() {
  auto *st = &this->sensor_query_state_;
  if (!st->active) {
    ESP_LOGE(TAG, "bad state");
    return false;
  }
  auto i = st->step + 1;
  if (i >= static_cast<int>(this->batch_sensor_query_.size())) {
    return false;
  }
  this->write_array(this->batch_sensor_query_[i].req_buffer);
  st->step = i;
  ESP_LOGV(TAG, "sensor query batch %d/%d sent (%d bytes)", i + 1, this->batch_sensor_query_.size(),
           this->batch_sensor_query_[i].req_buffer.size());
  return true;
}

void SY7T609Component::sensor_query_done_() {
  ESP_LOGV(TAG, "sensor query done. (%u ms)", millis() - this->sensor_query_state_.time_track);
  this->sensor_query_state_.active = false;
}

void SY7T609Component::sensor_query_fail_() {
  ESP_LOGE(TAG, "sensor query end with error");
  this->sensor_query_state_.active = false;
  this->discard_rx_buffer_();
}

void SY7T609Component::sensor_query_loop_() {
  if (!this->sensor_query_state_.active) {
    return;  // skip loop
  }

  int step = this->sensor_query_state_.step;
  assert(step >= 0 && step < static_cast<int>(this->batch_sensor_query_.size()));

  const auto &query = this->batch_sensor_query_[step];
  if (this->available() < static_cast<int>(query.resp_len)) {
    return;  // skip loop
  }

  if (this->sensor_query_receive_packet_(query)) {
    if (this->sensor_query_advance_()) {
      return;
    } else {
      this->sensor_query_done_();
      return;
    }
  } else {
    this->sensor_query_fail_();
  }
}

bool SY7T609Component::sensor_query_receive_packet_(const BatchReadInfo &batch) {
  uint8_t reply_code;
  if (!this->read_byte(&reply_code)) {
    ESP_LOGE(TAG, "UART read failed");
    return false;
  }
  if (reply_code != REPLY_CODE_ACK_WITH_DATA) {
    ESP_LOGE(TAG, "bad reply code: 0x%02hhX", reply_code);
    return false;
  }

  size_t resp_len = batch.resp_len;
  std::vector<uint8_t> resp(resp_len);
  resp[0] = reply_code;
  if (!this->read_array(resp.data() + 1, resp_len - 1)) {
    ESP_LOGE(TAG, "UART read failed");
    return false;
  }
  char buffer[format_hex_size(SSI_REQ_MAX_PACKET_SIZE)];
  ESP_LOGVV(TAG, "sensor query reply: %s", format_hex_to(buffer, resp));

  uint8_t sum = sy7t609_checksum(resp);
  if (sum != resp[resp_len - 1]) {
    ESP_LOGE(TAG, "bad reply checksum: 0x%02hhX != 0x%02hhX", sum, resp[resp_len - 1]);
    return false;
  }

  for (const auto &item : batch.items) {
    RegValue24 v = read_reg24(resp.data() + item.offset);
    switch (item.addr) {
      case REG_CTEMP:
        this->chip_temperature_sensor_->publish_state(v.u24_value() * this->scale_cache_.tfactor);
        break;
      case REG_VRMS:
        this->voltage_sensor_->publish_state(v.u24_value() * this->scale_cache_.vfactor);
        break;
      case REG_VAVG:
        this->vavg_sensor_->publish_state(v.s24_value() * this->scale_cache_.vfactor);
        break;
      case REG_VFUND:
        this->vfund_sensor_->publish_state(v.u24_value() * this->scale_cache_.vfactor);
        break;
      case REG_VHARM:
        this->vharm_sensor_->publish_state(v.u24_value() * this->scale_cache_.vfactor);
        break;
      case REG_VPEAK:
        this->vpeak_sensor_->publish_state(v.u24_value() * this->scale_cache_.vfactor);
        break;
      case REG_IRMS:
        this->current_sensor_->publish_state(v.u24_value() * this->scale_cache_.ifactor);
        break;
      case REG_IAVG:
        this->iavg_sensor_->publish_state(v.s24_value() * this->scale_cache_.ifactor);
        break;
      case REG_IFUND:
        this->ifund_sensor_->publish_state(v.u24_value() * this->scale_cache_.ifactor);
        break;
      case REG_IHARM:
        this->iharm_sensor_->publish_state(v.u24_value() * this->scale_cache_.ifactor);
        break;
      case REG_IPEAK:
        this->ipeak_sensor_->publish_state(v.u24_value() * this->scale_cache_.ifactor);
        break;
      case REG_POWER:
        this->active_power_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_AVGPOWER:
        this->avgpower_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_PFUND:
        this->pfund_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_PHARM:
        this->pharm_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_VAR:
        this->reactive_power_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_QFUND:
        this->qfund_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_QHARM:
        this->qharm_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_VA:
        this->apparent_power_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_VAFUND:
        this->vafund_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_VAHARM:
        this->vaharm_sensor_->publish_state(v.s24_value() * this->scale_cache_.pfactor);
        break;
      case REG_FREQUENCY:
        this->line_frequency_sensor_->publish_state(v.u24_value() * this->scale_cache_.ffactor);
        break;
      case REG_PF:
        this->power_factor_sensor_->publish_state(v.s24_value() * this->scale_cache_.pffactor);
        break;
      case REG_EPPCNT:
        this->positive_active_energy_sensor_->publish_state(v.u24_value() * this->scale_cache_.efactor);
        break;
      case REG_EPMCNT:
        this->negative_active_energy_sensor_->publish_state(v.u24_value() * this->scale_cache_.efactor);
        break;
      case REG_EPNCNT:
        this->net_active_energy_sensor_->publish_state(v.u24_value() * this->scale_cache_.efactor);
        break;
      case REG_EQNCNT:
        this->net_reactive_energy_sensor_->publish_state(v.s24_value() * this->scale_cache_.efactor);
        break;
      case REG_ESNCNT:
        this->net_apparent_energy_sensor_->publish_state(v.u24_value() * this->scale_cache_.efactor);
        break;
      default:
        ESP_LOGE(TAG, "unexpected register: 0x%03X", item.addr);
        break;
    }
  }
  return true;
}

void SY7T609Component::wait_active_query_end_() {
  if (!this->sensor_query_state_.active) {
    return;
  }

  ESP_LOGD(TAG, "waiting active query end");
  while (true) {
    int step = this->sensor_query_state_.step;

    assert(step >= 0 && step < static_cast<int>(this->batch_sensor_query_.size()));
    const auto &query = this->batch_sensor_query_[step];

    this->flush();

    if (this->sensor_query_receive_packet_(query)) {
      if (this->sensor_query_advance_()) {
        continue;
      } else {
        this->sensor_query_done_();
        return;
      }
    } else {
      this->sensor_query_fail_();
      return;
    }
  }
}

void SY7T609Component::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SY7T609_UART:\n"
                "  Update Interval: %u\n",
                this->update_interval_);
  ESP_LOGCONFIG(TAG,
                "  Scale Parameters:\n"
                "    FSV: %f\n"
                "    FSI: %f",
                this->scale_parameters_.fsv, this->scale_parameters_.fsi);
  const auto &s = this->scale_registers_;
  ESP_LOGCONFIG(TAG,
                "  Scale Registers:\n"
                "    ISCALE: %u\n"
                "    VSCALE: %u\n"
                "    PSCALE: %u\n"
                "    PFSCALE: %u\n"
                "    TSCALE: %u\n"
                "    FSCALE: %u",
                s.iscale, s.vscale, s.pscale, s.pfscale, s.tscale, s.fscale);
  ESP_LOGCONFIG(TAG,
                "    control: 0x%06X\n"
                "    bucket: %f",
                s.control.raw, s.bucket);

  LOG_SENSOR("  ", "Chip Temperature", this->chip_temperature_sensor_);
  LOG_SENSOR("  ", "RMS Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Average Voltage", this->vavg_sensor_);
  LOG_SENSOR("  ", "Fundamental RMS Voltage", this->vfund_sensor_);
  LOG_SENSOR("  ", "Harmonic RMS Voltage", this->vharm_sensor_);
  LOG_SENSOR("  ", "Peak Voltage", this->vpeak_sensor_);
  LOG_SENSOR("  ", "RMS Current", this->current_sensor_);
  LOG_SENSOR("  ", "Average Current", this->iavg_sensor_);
  LOG_SENSOR("  ", "Fundamental RMS Current", this->ifund_sensor_);
  LOG_SENSOR("  ", "Harmonic RMS Current", this->iharm_sensor_);
  LOG_SENSOR("  ", "Peak Current", this->ipeak_sensor_);
  LOG_SENSOR("  ", "Active Power", this->active_power_sensor_);
  LOG_SENSOR("  ", "Average Active Power", this->avgpower_sensor_);
  LOG_SENSOR("  ", "Fundamental Active Power", this->pfund_sensor_);
  LOG_SENSOR("  ", "Harmonic Active Power", this->pharm_sensor_);
  LOG_SENSOR("  ", "Reactive Power", this->reactive_power_sensor_);
  LOG_SENSOR("  ", "Fundamental Reactive Power", this->qfund_sensor_);
  LOG_SENSOR("  ", "Harmonic Reactive Power", this->qharm_sensor_);
  LOG_SENSOR("  ", "Apparent Power", this->apparent_power_sensor_);
  LOG_SENSOR("  ", "Fundamental Apparent Power", this->vafund_sensor_);
  LOG_SENSOR("  ", "Harmonic Apparent Power", this->vaharm_sensor_);
  LOG_SENSOR("  ", "Line Frequency", this->line_frequency_sensor_);
  LOG_SENSOR("  ", "Power Factor", this->power_factor_sensor_);
  LOG_SENSOR("  ", "Positive Active Energy", this->positive_active_energy_sensor_);
  LOG_SENSOR("  ", "Negative Active Energy", this->negative_active_energy_sensor_);
  LOG_SENSOR("  ", "Net Active Energy", this->net_active_energy_sensor_);
  LOG_SENSOR("  ", "Net Reactive Energy", this->net_reactive_energy_sensor_);
  LOG_SENSOR("  ", "Net Apparent Energy", this->net_apparent_energy_sensor_);
}

bool SY7T609Component::invoke_command_sync_(CommandRegistryCode cmd) {
  this->wait_active_query_end_();

  if (!this->ssi_write_reg_sync(REG_COMMAND, {cmd})) {
    ESP_LOGE(TAG, "command 0x%06X failed: write command reg failed", cmd);
    return false;
  }

  if ((cmd & 0xFF0000) == 0xCA0000) {  // “Calibrate” Command
    // Wait calibration process complete.
    // Long running process.
    // Run over CalItr iterations that are each CalCyc accumulation intervals long.
    // Observed ~15s with default register values.

    uint32_t wait_start = millis();
    while (true) {
      auto reg = this->ssi_read_reg_sync(REG_COMMAND);
      if (!reg.has_value()) {
        ESP_LOGE(TAG, "calibrate command 0x%06X failed: read Command reg failed", cmd);
        return false;
      }

      // When the calibration process completes, bits 23:16 are cleared
      if ((reg->raw & 0xFF0000) == 0) {
        // If the calibration has completed without a detected error bits 15:0 are also cleared.
        if (reg->raw == 0) {
          ESP_LOGV(TAG, "calibrate command 0x%06X complete after %u ms", cmd, millis() - wait_start);
          break;
        } else {
          ESP_LOGE(TAG, "calibrate command 0x%06X completed with error. Command reg value: 0x%06X", cmd, reg->raw);
          return false;
        }
      } else if (millis() - wait_start > this->calibration_timeout_ms_) {
        ESP_LOGE(TAG, "calibrate command 0x%06X timeout after %u ms", cmd, millis() - wait_start);
        return false;
      } else {
        continue;
      }
    }
  }

  ESP_LOGD(TAG, "command 0x%06X done", cmd);
  return true;
}

void SY7T609Component::save_to_flash() { this->invoke_command_sync_(CMD_REG_SAVE_TO_FLASH); }

void SY7T609Component::set_auto_report(bool on_off) {
  this->invoke_command_sync_(on_off ? CMD_REG_AUTO_REPORT_ON : CMD_REG_AUTO_REPORT_OFF);
}

void SY7T609Component::clear_energy_counters() { this->invoke_command_sync_(CMD_REG_CLEAR_ENGERGY_COUNTERS); }

void SY7T609Component::soft_reset() { this->invoke_command_sync_(CMD_REG_SOFT_RESET); }

void SY7T609Component::update_scale_cache_() {
  const auto &sp = this->scale_parameters_;
  const auto &sr = this->scale_registers_;

  this->scale_cache_.vfactor = 1.0 * sp.fsv / sr.vscale;
  this->scale_cache_.ifactor = 1.0 * sp.fsi / sr.iscale / (sr.control.ishift ? 256 : 1);
  this->scale_cache_.pfactor = 1.0 * sp.fsv * sp.fsi / sr.pscale / (sr.control.pshift ? 256 : 1);
  this->scale_cache_.pffactor = 1.0 / sr.pfscale;
  this->scale_cache_.tfactor = 1.0 / sr.tscale;
  this->scale_cache_.ffactor = 1.0 / sr.fscale;
  // bucket / (3600s * 6702sps / fsi / fsv)
  this->scale_cache_.efactor = 1.0 * sr.bucket / (3600.0 * 6702 / sp.fsv / sp.fsi);
}

bool SY7T609Component::calibrate_voltage_gain(float vrms_target_volts) {
  uint32_t scaling_value = static_cast<uint32_t>(std::round(vrms_target_volts / this->scale_cache_.vfactor));
  if (!this->ssi_write_reg_sync(REG_VRMSTARGET, RegValue24::from_u24(scaling_value))) {
    ESP_LOGE(TAG, "write vrmstarget failed");
    return false;
  }
  if (!this->invoke_command_sync_(CMD_REG_CALIBRATE_VRMS)) {
    return false;
  }
  ESP_LOGI(TAG, "calibrate_voltage_gain done");
  return true;
}

bool SY7T609Component::calibrate_current_gain(float irms_target_amps) {
  uint32_t scaling_value = static_cast<uint32_t>(std::round(irms_target_amps / this->scale_cache_.ifactor));
  if (!this->ssi_write_reg_sync(REG_IRMSTARGET, RegValue24::from_u24(scaling_value))) {
    ESP_LOGE(TAG, "write irmstarget failed");
    return false;
  }
  if (!this->invoke_command_sync_(CMD_REG_CALIBRATE_IRMS)) {
    return false;
  }
  ESP_LOGI(TAG, "calibrate_current_gain done");
  return true;
}

bool SY7T609Component::calibrate_voltage_offset(float vavg_target_volts) {
  uint32_t scaling_value = static_cast<uint32_t>(std::round(vavg_target_volts / this->scale_cache_.vfactor));
  if (!this->ssi_write_reg_sync(REG_VAVGTARGET, RegValue24::from_u24(scaling_value))) {
    ESP_LOGE(TAG, "write vavgtarget failed");
    return false;
  }
  if (!this->invoke_command_sync_(CMD_REG_CALIBRATE_VOFFS)) {
    return false;
  }
  ESP_LOGI(TAG, "calibrate_voltage_offset done");
  return true;
}

bool SY7T609Component::calibrate_current_offset(float iavg_target_amps) {
  uint32_t scaling_value = static_cast<uint32_t>(std::round(iavg_target_amps / this->scale_cache_.ifactor));
  if (!this->ssi_write_reg_sync(REG_IAVGTARGET, RegValue24::from_u24(scaling_value))) {
    ESP_LOGE(TAG, "write iavgtarget failed");
    return false;
  }
  if (!this->invoke_command_sync_(CMD_REG_CALIBRATE_IOFFS)) {
    return false;
  }
  ESP_LOGI(TAG, "calibrate_current_offset done");
  return true;
}

bool SY7T609Component::calibrate_temperature(float temp_celsius) {
  // Load control register
  auto control_raw = this->ssi_read_reg_sync(REG_CONTROL);
  if (!control_raw.has_value()) {
    ESP_LOGE(TAG, "read control register failed");
    return false;
  }
  ControlRegister control;
  control.raw = control_raw->u24_value();

  // Set ct bit to prevent firmware from updating CTemp.
  // See datasheet 4.12.6 On-Chip Temperature Calibration
  control.ct = true;
  if (!this->ssi_write_reg_sync(REG_CONTROL, {control.raw})) {
    ESP_LOGE(TAG, "set ct bit failed");
    return false;
  }

  // Write CTemp
  uint32_t scaling_value = static_cast<uint32_t>(std::round(temp_celsius / this->scale_cache_.tfactor));
  if (!this->ssi_write_reg_sync(REG_CTEMP, RegValue24::from_u24(scaling_value))) {
    ESP_LOGE(TAG, "write iavgtarget failed");
    return false;
  }

  // Call calibration
  if (!this->invoke_command_sync_(CMD_REG_CALIBRATE_CTEMP)) {
    control.ct = false;
    ssi_write_reg_sync(REG_CONTROL, {control.raw});
    return false;
  }

  // Clear ct bit
  control.ct = false;
  if (!this->ssi_write_reg_sync(REG_CONTROL, {control.raw})) {
    ESP_LOGE(TAG, "clear ct bit failed");
    return false;
  }

  ESP_LOGI(TAG, "calibrate_temperature done");
  return true;
}

void SY7T609Component::discard_rx_buffer_() {
  int n = 0;
  while (this->available() > 0) {
    this->read();
    n++;
  }
  if (n > 0) {
    ESP_LOGW(TAG, "discard %d rx bytes", n);
  }
}

}  // namespace esphome::sy7t609
