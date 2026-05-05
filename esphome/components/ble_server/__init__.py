import esphome.codegen as cg
from esphome.components import esp32_ble
from esphome.components.esp32_ble import BTLoggers
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MODEL

AUTO_LOAD = ["esp32_ble_server"]
DEPENDENCIES = ["api", "esp32"]

CONF_DEVICE_NAME = "device_name"
CONF_MANUFACTURER = "manufacturer"
CONF_FIRMWARE_VERSION = "firmware_version"

ble_server_ns = cg.esphome_ns.namespace("ble_server")
BleServer = ble_server_ns.class_("BleServer", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BleServer),
        cv.Optional(CONF_DEVICE_NAME, default="Homeassistant_Home"): cv.string,
        cv.Optional(CONF_MANUFACTURER, default="Home Assistant"): cv.string,
        cv.Optional(CONF_MODEL, default="ESP32-HA-Gateway"): cv.string,
        cv.Optional(CONF_FIRMWARE_VERSION, default="1.0.0"): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    esp32_ble.register_bt_logger(BTLoggers.GATT, BTLoggers.SMP)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add_define("USE_BLE_SERVER")

    cg.add(var.set_device_name(config[CONF_DEVICE_NAME]))
    cg.add(var.set_manufacturer(config[CONF_MANUFACTURER]))
    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_firmware_version(config[CONF_FIRMWARE_VERSION]))
