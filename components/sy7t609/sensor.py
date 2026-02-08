from esphome import automation
import esphome.codegen as cg
from esphome.components import sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACTIVE_POWER,
    CONF_APPARENT_POWER,
    CONF_CURRENT,
    CONF_ENERGY,
    CONF_ID,
    CONF_LINE_FREQUENCY,
    CONF_POWER_FACTOR,
    CONF_REACTIVE_POWER,
    CONF_VOLTAGE,
    DEVICE_CLASS_APPARENT_POWER,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_REACTIVE_ENERGY,
    DEVICE_CLASS_REACTIVE_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_VOLT,
    UNIT_VOLT_AMPS,
    UNIT_VOLT_AMPS_HOURS,
    UNIT_VOLT_AMPS_REACTIVE,
    UNIT_VOLT_AMPS_REACTIVE_HOURS,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)

CONF_CHIP_TEMPERATURE = "chip_temperature"
CONF_VAVG = "vavg"
CONF_VFUND = "vfund"
CONF_VHARM = "vharm"
CONF_VPEAK = "vpeak"
CONF_IAVG = "iavg"
CONF_IFUND = "ifund"
CONF_IHARM = "iharm"
CONF_IPEAK = "ipeak"
CONF_AVGPOWER = "avgpower"
CONF_PFUND = "pfund"
CONF_PHARM = "pharm"
CONF_QFUND = "qfund"
CONF_QHARM = "qharm"
CONF_VAFUND = "vafund"
CONF_VAHARM = "vaharm"
CONF_POSITIVE_ACTIVE_ENERGY = "positive_active_energy"
CONF_NEGATIVE_ACTIVE_ENERGY = "negative_active_energy"
CONF_NET_ACTIVE_ENERGY = "net_active_energy"
CONF_NET_REACTIVE_ENERGY = "net_reactive_energy"
CONF_NET_APPARENT_ENERGY = "net_apparent_energy"

CONF_FSV = "fsv"
CONF_FSI = "fsi"
CONF_BATCH_SIZE = "batch_size"
CONF_CALIBRATION_TIMEOUT = "calibration_timeout"

CODEOWNERS = ["@fishsugar"]
DEPENDENCIES = ["uart"]

sy7t609_ns = cg.esphome_ns.namespace("sy7t609")
SY7T609Component = sy7t609_ns.class_(
    "SY7T609Component", cg.PollingComponent, uart.UARTDevice
)

# Actions
ClearEnergyCountersAction = sy7t609_ns.class_(
    "ClearEnergyCountersAction", automation.Action
)
SoftResetAction = sy7t609_ns.class_("SoftResetAction", automation.Action)

NO_ARGS_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(SY7T609Component),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SY7T609Component),
            cv.Optional(CONF_FSV, default="667Vpk"): cv.All(
                cv.float_with_unit("peak voltage", "(Vpk)")
            ),
            cv.Optional(CONF_FSI, default="62Apk"): cv.All(
                cv.float_with_unit("peak current", "(Apk)")
            ),
            cv.Optional(CONF_BATCH_SIZE, default="14"): cv.int_range(min=1),
            cv.Optional(
                CONF_CALIBRATION_TIMEOUT, default="20s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CHIP_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VAVG): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VFUND): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VHARM): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VPEAK): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_IAVG): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_IFUND): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_IHARM): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_IPEAK): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ACTIVE_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_AVGPOWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PFUND): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PHARM): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_REACTIVE_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_REACTIVE_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_QFUND): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_REACTIVE_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_QHARM): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_REACTIVE_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_APPARENT_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_APPARENT_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VAFUND): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_APPARENT_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VAHARM): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_APPARENT_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_LINE_FREQUENCY): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER_FACTOR): sensor.sensor_schema(
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER_FACTOR,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POSITIVE_ACTIVE_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_NEGATIVE_ACTIVE_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_NET_ACTIVE_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL,
            ),
            cv.Optional(CONF_NET_REACTIVE_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_REACTIVE_ENERGY,
                state_class=STATE_CLASS_TOTAL,
            ),
            cv.Optional(CONF_NET_APPARENT_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "sy7t609",
    require_rx=True,
    require_tx=True,
    data_bits=8,
    stop_bits=1,
    parity="NONE",
)


@automation.register_action(
    "sy7t609.clear_energy_counters",
    ClearEnergyCountersAction,
    NO_ARGS_ACTION_SCHEMA,
)
@automation.register_action(
    "sy7t609.soft_reset",
    SoftResetAction,
    NO_ARGS_ACTION_SCHEMA,
)
async def no_args_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_fsv(config[CONF_FSV]))
    cg.add(var.set_fsi(config[CONF_FSI]))
    cg.add(var.set_batch_size(config[CONF_BATCH_SIZE]))
    cg.add(var.set_calibration_timeout_ms(config[CONF_CALIBRATION_TIMEOUT]))

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for key in [
        CONF_CHIP_TEMPERATURE,
        CONF_VOLTAGE,
        CONF_VAVG,
        CONF_VFUND,
        CONF_VHARM,
        CONF_VPEAK,
        CONF_CURRENT,
        CONF_IAVG,
        CONF_IFUND,
        CONF_IHARM,
        CONF_IPEAK,
        CONF_ACTIVE_POWER,
        CONF_AVGPOWER,
        CONF_PFUND,
        CONF_PHARM,
        CONF_REACTIVE_POWER,
        CONF_QFUND,
        CONF_QHARM,
        CONF_APPARENT_POWER,
        CONF_VAFUND,
        CONF_VAHARM,
        CONF_LINE_FREQUENCY,
        CONF_POWER_FACTOR,
        CONF_ENERGY,
        CONF_POSITIVE_ACTIVE_ENERGY,
        CONF_NEGATIVE_ACTIVE_ENERGY,
        CONF_NET_ACTIVE_ENERGY,
        CONF_NET_REACTIVE_ENERGY,
        CONF_NET_APPARENT_ENERGY,
    ]:
        if key not in config:
            continue
        conf = config[key]
        sens = await sensor.new_sensor(conf)
        cg.add(getattr(var, f"set_{key}_sensor")(sens))
