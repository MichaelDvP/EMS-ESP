/*
 * EMS-ESP - https://github.com/emsesp/EMS-ESP
 * Copyright 2020  Paul Derbyshire
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "switch.h"

namespace emsesp {

REGISTER_FACTORY(Switch, EMSdevice::DeviceType::SWITCH);

uuid::log::Logger Switch::logger_ {
    F_(switch), uuid::log::Facility::CONSOLE
};

Switch::Switch(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand)
    : EMSdevice(device_type, device_id, product_id, version, name, flags, brand) {
    LOG_DEBUG(F("Adding new Switch with device ID 0x%02X"), device_id);

    register_telegram_type(0x9C, F("WM10MonitorMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_WM10MonitorMessage(t); });
    register_telegram_type(0x9D, F("WM10SetMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_WM10SetMessage(t); });
    register_telegram_type(0x1E, F("WM10TempMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_WM10TempMessage(t); });
}

// fetch the values into a JSON document for display in the web
void Switch::device_info_web(JsonArray & root, uint8_t & part) {
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_SMALL);
    JsonObject                                     json = doc.to<JsonObject>();
    if (export_values(json)) {
        // doc.shrinkToFit();
        create_value_json(root, F_(activated), nullptr, F_(activated_), nullptr, json);
        create_value_json(root, F_(flowtemphc), nullptr, F_(flowtemphc_), F_(degrees), json);
        create_value_json(root, F_(status), nullptr, F_(status_), nullptr, json);
    }
}

// publish values via MQTT
void Switch::publish_values(JsonObject & json, bool force) {
    if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
        if (!mqtt_ha_config_ || force) {
            register_mqtt_ha_config();
            return;
        }
    }

    // StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
    // JsonObject                                     json_data = doc.to<JsonObject>();
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_SMALL);
    JsonObject          json_data = doc.to<JsonObject>();
    if (export_values(json_data)) {
        // doc.shrinkToFit();
        Mqtt::publish(F("switch_data"), doc.as<JsonObject>());
    }
}

// export values to JSON
bool Switch::export_values(JsonObject & json, int8_t id) {
    Helpers::json_boolean(json, F_(activated), activated_);

    if (Helpers::hasValue(flowTempHc_)) {
        json[F_(flowtemphc)] = (float)flowTempHc_ / 10;
    }

    if (Helpers::hasValue(status_)) {
        json[F_(status)] = status_;
    }

    return true;
}

// check to see if values have been updated
bool Switch::updated_values() {
    if (changed_) {
        changed_ = false;
        return true;
    }
    return false;
}

// publish config topic for HA MQTT Discovery
void Switch::register_mqtt_ha_config() {
    if (!Mqtt::connected()) {
        return;
    }

    // if we don't have valid values for this HC don't add it ever again
    if (!Helpers::hasValue(flowTempHc_)) {
        return;
    }

    // Create the Master device
    StaticJsonDocument<EMSESP_MAX_JSON_SIZE_HA_CONFIG> doc;

    char name[10];
    snprintf_P(name, sizeof(name), PSTR("Switch"));
    doc[F_(name)] = name;

    char uniq_id[10];
    snprintf_P(uniq_id, sizeof(uniq_id), PSTR("switch"));
    doc[F_(uniq_id)] = uniq_id;

    doc[F_(ic)] = F_(iconthermostat);

    char stat_t[128];
    snprintf_P(stat_t, sizeof(stat_t), PSTR("%s/%s"), Mqtt::base().c_str(), Fc_(switch_data));
    doc[F_(stat_t)] = stat_t;

    doc[F_(val_tpl)] = F("{{value_json.type}}"); // HA needs a single value. We take the type which is wwc or hc

    JsonObject dev = doc.createNestedObject(F_(dev));
    dev[F_(name)]  = F("EMS-ESP Switch");
    dev[F_(sw)]    = EMSESP_APP_VERSION;
    dev[F_(mf)]    = brand_to_string();
    dev[F_(mdl)]   = this->name();
    JsonArray ids  = dev.createNestedArray(F_(ids));
    ids.add(F("ems-esp-switch"));

    std::string topic(128, '\0');
    snprintf_P(&topic[0], topic.capacity() + 1, PSTR("%s%s/%s/%s"),Fc_(hasensor), Mqtt::base().c_str(), Fc_(switch), Fc_(config));
    Mqtt::publish_ha(topic, doc.as<JsonObject>()); // publish the config payload with retain flag

    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(activated_), device_type(), F_(activated), nullptr, nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(flowtemphc_), device_type(), F_(flowtemphc), F_(degrees), F_(iconwatertemp));
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(status_), device_type(), F_(status), nullptr, nullptr);

    mqtt_ha_config_ = true; // done
}

// message 0x9D switch on/off
// Thermostat(0x10) -> Switch(0x11), ?(0x9D), data: 00
void Switch::process_WM10SetMessage(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(activated_, 0);
}

// message 0x9C holds flowtemp and unknown status value
// Switch(0x11) -> All(0x00), ?(0x9C), data: 01 BA 00 01 00
void Switch::process_WM10MonitorMessage(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(flowTempHc_, 0); // is * 10
    changed_ |= telegram->read_value(status_, 2);
    // changed_ |= telegram->read_value(status2_, 3); // unknown
}

// message 0x1E flow temperature, same as in 9C, published often, republished also by boiler UBAFast 0x18
// Switch(0x11) -> Boiler(0x08), ?(0x1E), data: 01 BA
void Switch::process_WM10TempMessage(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(flowTempHc_, 0); // is * 10
}

} // namespace emsesp