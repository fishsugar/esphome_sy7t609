# SY7T609 component for ESPHome

The `sy7t609` component allows you to use your SY7T609+S1 energy sensor with ESPHome. This sensor is found in the CMPOWER W1 power strip.

The SY7T609+S1 is an energy measurement processor (EMP) designed for monitoring any 2-wire circuit.

[UART](https://esphome.io/components/uart/) is required for this component. The SY7T609+S1 supports UART and SPI interface options, only UART implemented in this component. The SY7T609 features a UART interface with a data rate ranging from 2400 up to 115k Baud (9600 by default). The UART interface has a fixed configuration supporting: 8-bit, one start bit, one stop bit and no-parity.

```yaml
uart:
  tx_pin: GPIO01
  rx_pin: GPIO03
  baud_rate: 9600
  stop_bits: 1
  data_bits: 8
  parity: NONE
```


### Configuration variables

- **id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#id)): Manually specify the ID for this component.
- **uart_id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#id)): Manually specify the ID of the [UART Component](/components/uart) to use.
  Required if you have multiple UARTs configured.
- **update_interval** (*Optional*, [Time](https://esphome.io/guides/configuration-types#time)): The interval to check the sensor. Defaults to 60s.
- **fsv** (*Optional*, float): Full scale voltage input. For voltage inputs, full scale (FSV) is defined as the peak system voltage or current that results in +/-250 mVpk at the ADC input. See the [datasheet] section "5.3.1 VSCALE" for details. Defaults to `667 Vpk`.
- **fsi** (*Optional*, float): Full scale current input. For current inputs, full scale (FSI) is defined as the peak system current that results in +/-250 mVpk at the ADC input. See the [datasheet] section "5.3.2 ISCALE" for details. Defaults to `62 Apk`.
- **batch_size** (*Optional*, int): Limit the number of sensor registers to read per request packet, set too high may cause SY7T609+S1 fail to respond the request (behavior not documented in datasheet). Defaults to `14`.
- **calibration_timeout** (*Optional*, [Time](https://esphome.io/guides/configuration-types#time)): Timeout for the calibration process when calling calibration actions. Observed ~15s with default register values. See the [datasheet] section "5.5.5 Calibration Command" for details about the calibration process. Defaults to `20s`.

#### Sensor variables

| Category         | Sensor Variables                                                                                                |
| ---------------- | --------------------------------------------------------------------------------------------------------------- |
| Chip Temperature | chip_temperature                                                                                                |
| Voltage          | voltage, vavg, vfund, vharm, vpeak                                                                              |
| Current          | current, iavg, ifund, iharm, ipeak                                                                              |
| Active Power     | active_power, avgpower, pfund, pharm                                                                            |
| Reactive Power   | reactive_power, qfund, qharm                                                                                    |
| Apparent Power   | apparent_power, vafund, vaharm                                                                                  |
| Line Frequency   | line_frequency                                                                                                  |
| Power Factor     | power_factor                                                                                                    |
| Energy           | positive_active_energy, negative_active_energy, net_active_energy,<br/>net_reactive_energy, net_apparent_energy |

> [!NOTE]
> See the [datasheet] "4.7 Fundamental and Harmonics Calculations" for fundamental and harmonics values.

- **chip_temperature** (*Optional*): Chip temperature. All options from [Sensor](https://esphome.io/components/sensor).
- **voltage** (*Optional*): RMS voltage. All options from [Sensor](https://esphome.io/components/sensor).
- **vavg** (*Optional*): Average voltage. All options from [Sensor](https://esphome.io/components/sensor).
- **vfund** (*Optional*): Fundamental RMS voltage. All options from [Sensor](https://esphome.io/components/sensor).
- **vharm** (*Optional*): Harmonic RMS voltage. All options from [Sensor](https://esphome.io/components/sensor).
- **vpeak** (*Optional*): The highest instantaneous voltage measured during an accumulation interval. All options from [Sensor](https://esphome.io/components/sensor).
- **current** (*Optional*): RMS current. All options from [Sensor](https://esphome.io/components/sensor).
- **iavg** (*Optional*): Average current. All options from [Sensor](https://esphome.io/components/sensor).
- **ifund** (*Optional*): Fundamental RMS current. All options from [Sensor](https://esphome.io/components/sensor).
- **iharm** (*Optional*): Harmonic RMS current. All options from [Sensor](https://esphome.io/components/sensor).
- **ipeak** (*Optional*): The highest instantaneous current measured during an accumulation interval.All options from [Sensor](https://esphome.io/components/sensor).
- **active_power** (*Optional*): Active power. All options from [Sensor](https://esphome.io/components/sensor).
- **avgpower** (*Optional*): Average active power. All options from [Sensor](https://esphome.io/components/sensor).
- **pfund** (*Optional*): Fundamental active power. All options from [Sensor](https://esphome.io/components/sensor).
- **pharm** (*Optional*): Harmonic active power. All options from [Sensor](https://esphome.io/components/sensor).
- **reactive_power** (*Optional*): Reactive power. All options from [Sensor](https://esphome.io/components/sensor).
- **qfund** (*Optional*): Fundamental reactive power. All options from [Sensor](https://esphome.io/components/sensor).
- **qharm** (*Optional*): Harmonic reactive power. All options from [Sensor](https://esphome.io/components/sensor).
- **apparent_power** (*Optional*): Apparent power. All options from [Sensor](https://esphome.io/components/sensor).
- **vafund** (*Optional*): Fundamental apparent power. All options from [Sensor](https://esphome.io/components/sensor).
- **vaharm** (*Optional*): Harmonic apparent power. All options from [Sensor](https://esphome.io/components/sensor).
- **line_frequency** (*Optional*): Line frequency. All options from [Sensor](https://esphome.io/components/sensor).
- **power_factor** (*Optional*): Power factor. All options from [Sensor](https://esphome.io/components/sensor).
- **positive_active_energy** (*Optional*): Positive active energy. All options from [Sensor](https://esphome.io/components/sensor).
- **negative_active_energy** (*Optional*): Negative active energy. All options from [Sensor](https://esphome.io/components/sensor).
- **net_active_energy** (*Optional*): Net active energy. All options from [Sensor](https://esphome.io/components/sensor).
- **net_reactive_energy** (*Optional*): Net reactive energy. All options from [Sensor](https://esphome.io/components/sensor).
- **net_apparent_energy** (*Optional*): Net apparent energy. All options from [Sensor](https://esphome.io/components/sensor).


## Automation

### `sy7t609.clear_energy_counters` Action

This is an [Action](/automations/actions#all-actions) for clear all energy counters.

```yaml
- sy7t609.clear_energy_counters: id_sensor_sy7t609
```

Configuration variables:

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types#id)): The ID of the component.

### `sy7t609.soft_reset` Action

This is an [Action](/automations/actions#all-actions) for soft-reset.

```yaml
- sy7t609.soft_reset: id_sensor_sy7t609
```

Configuration variables:

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types#id)): The ID of the component.

### Calibration Routines

Use [lambda](https://esphome.io/automations/templates#config-lambda) to calibrate voltage/current/temperature.

```cpp
    // Within lambda, calibrate voltage gain using vrmstarget 220V.
    id(id_sensor_sy7t609).calibrate_voltage_gain(220.0f);
    id(id_sensor_sy7t609).save_to_flash();
```

- `calibrate_voltage_gain(float vrmstarget)`: Calibrate Voltage Gain using `vrmstarget` (Volt). A stable supply must be applied to analog input.
- `calibrate_current_gain(float irmstarget)`: Calibrate Current Gain using `irmstarget` (Amp). A stable load must be applied to the channel to be calibrated.
- `calibrate_voltage_offset(float vavgtarget)`: Calibrate Voltage Offset using `vavgtarget` (Volt). A a supply with a stable DC voltage must be applied to the channel to be
calibrated.
- `calibrate_current_offset(float iavgtarget)`: Calibrate Current Offset using `iavgtarget` (Amp). A supply with a stable DC current must be applied to the channel to be
calibrated.
- `calibrate_temperature(float temperature)`: Calibrate Chip Temperature using `temperature` (°C).
- `save_to_flash()`: Save to flash the calibration coefficients and system defaults.

> [!NOTE]
> See the [datasheet] "4.12 On-Chip Calibration Routines" for details.

> [!WARNING]
> Calibration functions are blocking and take long time to return.
> See the [datasheet] "5.5.5 Calibration Command" for details.
> Observed ~15s with default register values.


## Example configuration

```yaml
external_components:
  - source: github://fishsugar/esphome_sy7t609
    components: [ sy7t609 ]

uart:
  tx_pin: GPIO01
  rx_pin: GPIO03
  baud_rate: 9600
  stop_bits: 1
  data_bits: 8
  parity: NONE

sensor:
  - platform: sy7t609
    chip_temperature:
      name: "Chip Temperature"
    voltage:
      name: "RMS Voltage"
    vavg:
      name: "Average Voltage"
    vfund:
      name: "Fundamental RMS Voltage"
    vharm:
      name: "Harmonic RMS Voltage"
    vpeak:
      name: "Peak Voltage"
    current:
      name: "RMS Current"
    iavg:
      name: "Average Current"
    ifund:
      name: "Fundamental RMS Current"
    iharm:
      name: "Harmonic RMS Current"
    ipeak:
      name: "Peak Current"
    active_power:
      name: "Active Power"
    avgpower:
      name: "Average Active Power"
    pfund:
      name: "Fundamental Active Power"
    pharm:
      name: "Harmonic Active Power"
    reactive_power:
      name: "Reactive Power"
    qfund:
      name: "Fundamental Reactive Power"
    qharm:
      name: "Harmonic Reactive Power"
    apparent_power:
      name: "Apparent Power"
    vafund:
      name: "Fundamental Apparent Power"
    vaharm:
      name: "Harmonic Apparent Power"
    line_frequency:
      name: "Line Frequency"
    power_factor:
      name: "Power Factor"
    positive_active_energy:
      name: "Positive Active Energy"
    negative_active_energy:
      name: "Negative Active Energy"
    net_active_energy:
      name: "Net Active Energy"
    net_reactive_energy:
      name: "Net Reactive Energy"
    net_apparent_energy:
      name: Net Apparent Energy
```


## Debug

[User-defined Actions](https://esphome.io/components/api/#user-defined-actions) for read or write any register.
To use these actions, go "Developer tools - Actions" in HomeAssistant and search for `debug_read_register`/`debug_write_register`.

```yaml
api:
  actions:
    - action: debug_read_register
      variables:
        addr: string
      supports_response: only
      then:
        - api.respond:
            data: !lambda |-
              auto parsed_addr = static_cast<sy7t609::RegAddr>(std::stoul(addr, nullptr, 0));
              auto result = id(id_sensor_sy7t609).ssi_read_reg_sync(parsed_addr);
              auto v = result.value();
              root["addr"] = str_sprintf("0x%03X", parsed_addr);
              auto node = root["value"].to<ArduinoJson::JsonObject>();
              node["Hex"] = str_sprintf("0x%06X", v.raw);
              node["U24"] = v.u24_value();
              node["S24"] = v.s24_value();
              node["U.21"] = v.u24_value<21>();
              node["U.23"] = v.u24_value<23>();
              node["U.24"] = v.u24_value<24>();
              node["S.21"] = v.s24_value<21>();
              node["S.23"] = v.s24_value<23>();
    - action: debug_write_register
      variables:
        addr: string
        u24_value: string
      then:
        - api.respond:
            success: !lambda |-
              auto parsed_addr = static_cast<sy7t609::RegAddr>(std::stoul(addr, nullptr, 0));
              uint32_t parsed_value = std::stoul(u24_value, nullptr, 0);
              id(id_sensor_sy7t609).ssi_write_reg_sync(parsed_addr, {parsed_value});
              return true;
```


## See Also

- [SY7T609+S1 Datasheet][datasheet]
- https://bbs.hassbian.com/thread-23734-1-1.html
- https://github.com/shzlww/esphome_custom_components/tree/main/components/sy7t609

[datasheet]: https://datasheet4u.com/pdf-down/S/Y/7/SY7T609+S1-Silergy.pdf
