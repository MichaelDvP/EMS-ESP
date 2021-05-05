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

#include "mixer.h"

namespace emsesp {

REGISTER_FACTORY(Mixer, EMSdevice::DeviceType::MIXER);

uuid::log::Logger Mixer::logger_{F_(mixer), uuid::log::Facility::CONSOLE};

Mixer::Mixer(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand)
    : EMSdevice(device_type, device_id, product_id, version, name, flags, brand) {
    LOG_DEBUG(F("Adding new Mixer with device ID 0x%02X"), device_id);

    if (flags == EMSdevice::EMS_DEVICE_FLAG_MMPLUS) {
        if (device_id <= 0x27) {
            // telegram handlers 0x20 - 0x27 for HC
            register_telegram_type(device_id - 0x20 + 0x02D7, F("MMPLUSStatusMessage_HC"), true, [&](std::shared_ptr<const Telegram> t) {
                process_MMPLUSStatusMessage_HC(t);
            });
        } else {
            // telegram handlers for warm water/DHW 0x28, 0x29
            register_telegram_type(device_id - 0x28 + 0x0331, F("MMPLUSStatusMessage_WWC"), true, [&](std::shared_ptr<const Telegram> t) {
                process_MMPLUSStatusMessage_WWC(t);
            });
        }
    }

    // EMS 1.0
    if (flags == EMSdevice::EMS_DEVICE_FLAG_MM10) {
        // register_telegram_type(0x00AA, F("MMConfigMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_MMConfigMessage(t); });
        register_telegram_type(0x00AB, F("MMStatusMessage"), true, [&](std::shared_ptr<const Telegram> t) { process_MMStatusMessage(t); });
        // register_telegram_type(0x00AC, F("MMSetMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_MMSetMessage(t); });
    }

    // HT3
    if (flags == EMSdevice::EMS_DEVICE_FLAG_IPM) {
        register_telegram_type(0x010C, F("IPMStatusMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_IPMStatusMessage(t); });
        register_telegram_type(0x001E, F("IPMTempMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_IPMTempMessage(t); });
    }
}

// output json to web UI
void Mixer::device_info_web(JsonArray & root, uint8_t & part) {
    if (type() == Type::NONE) {
        return; // don't have any values yet
    }

    // fetch the values into a JSON document
    // StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
    // JsonObject                                     json = doc.to<JsonObject>();
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_SMALL);
    JsonObject          json = doc.to<JsonObject>();

    if (!export_values_format(Mqtt::Format::SINGLE, json)) {
        return; // empty
    }

    char prefix_str[10];
    if (type() == Type::HC) {
        snprintf_P(prefix_str, sizeof(prefix_str), PSTR("(hc %d) "), hc_);
        // create_value_json(root, F("flowTempLowLoss"), FPSTR(prefix_str), F_(flowTempLowLoss), F_(degrees), json);
        create_value_json(root, F_(flowsettemp), FPSTR(prefix_str), F_(flowsettemp_), F_(degrees), json);
        create_value_json(root, F_(flowtemphc), FPSTR(prefix_str), F_(flowtemphc_), F_(degrees), json);
        create_value_json(root, F_(pumpstatus), FPSTR(prefix_str), F_(pumpstatus_), nullptr, json);
        create_value_json(root, F_(valvestatus), FPSTR(prefix_str), F_(valvestatus_), F_(percent), json);
    } else {
        snprintf_P(prefix_str, sizeof(prefix_str), PSTR("(wwc %d) "), hc_);
        create_value_json(root, F_(wwtemp), FPSTR(prefix_str), F_(wwtemp_), F_(degrees), json);
        create_value_json(root, F_(pumpstatus), FPSTR(prefix_str), F_(pumpstatus_), nullptr, json);
        create_value_json(root, F_(tempstatus), FPSTR(prefix_str), F_(tempstatus_), nullptr, json);
    }
}

// check to see if values have been updated
bool Mixer::updated_values() {
    if (changed_) {
        changed_ = false;
        return true;
    }
    return false;
}

// publish values via MQTT
// topic is mixer_data<id>
void Mixer::publish_values(JsonObject & json, bool force) {
    // handle HA first
    if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
        if (!mqtt_ha_config_ || force) {
            register_mqtt_ha_config();
            return;
        }
    }

    if (Mqtt::mqtt_format() == Mqtt::Format::SINGLE) {
        StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
        JsonObject                                     json_data = doc.to<JsonObject>();
        if (export_values_format(Mqtt::mqtt_format(), json_data)) {
            char topic[30];
            if (type() == Type::HC) {
                snprintf_P(topic, 30, PSTR("mixer_data_hc%d"), hc_);
            } else {
                snprintf_P(topic, 30, PSTR("mixer_data_wwc%d"), hc_);
            }
            Mqtt::publish(topic, doc.as<JsonObject>());
        }
    } else {
        // format is HA or Nested. This is bundled together and sent in emsesp.cpp
        export_values_format(Mqtt::mqtt_format(), json);
    }
}

// publish config topic for HA MQTT Discovery
void Mixer::register_mqtt_ha_config() {
    if (!Mqtt::connected()) {
        return;
    }

    // if we don't have valid values for this HC don't add it ever again
    if (!Helpers::hasValue(pumpStatus_)) {
        return;
    }

    // Create the Master device
    StaticJsonDocument<EMSESP_MAX_JSON_SIZE_HA_CONFIG> doc;

    char name[20];
    snprintf_P(name, sizeof(name), PSTR("Mixer %02X"), device_id() - 0x20 + 1);
    doc[F_(name)] = name;

    char uniq_id[20];
    snprintf_P(uniq_id, sizeof(uniq_id), PSTR("mixer%02X"), device_id() - 0x20 + 1);
    doc[F_(uniq_id)] = uniq_id;

    doc[F_(ic)] = F_(iconthermostat);

    char stat_t[128];
    snprintf_P(stat_t, sizeof(stat_t), PSTR("%s/%s"), Mqtt::base().c_str(), Fc_(mixer_data));
    doc[F_(stat_t)] = stat_t;

    char tpl[30];
    if (type_ == Type::HC) {
        snprintf_P(tpl, sizeof(tpl), PSTR("{{value_json.hc%d.type}}"), device_id() - 0x20 + 1);
     } else {
        snprintf_P(tpl, sizeof(tpl), PSTR("{{value_json.wwc%d.type}}"), device_id() - 0x28 + 1);
    }
    doc[F_(val_tpl)] = tpl;

    JsonObject dev = doc.createNestedObject(F_(dev));
    dev[F_(name)]  = F("EMS-ESP Mixer");
    dev[F_(sw)]    = EMSESP_APP_VERSION;
    dev[F_(mf)]    = brand_to_string();
    dev[F_(mdl)]   = this->name();
    JsonArray ids  = dev.createNestedArray(F_(ids));
    ids.add(F("ems-esp-mixer"));

    std::string topic(128, '\0');
    if (type() == Type::HC) {
        snprintf_P(&topic[0], topic.capacity() + 1, PSTR("%s%s/%s_hc%d/%s"),Fc_(hasensor), Mqtt::base().c_str(), Fc_(mixer), hc_, Fc_(config));
        Mqtt::publish_ha(topic, doc.as<JsonObject>()); // publish the config payload with retain flag
        char hc_name[10];
        snprintf_P(hc_name, sizeof(hc_name), PSTR("hc%d"), hc_);
        // Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(flowTempLowLoss), device_type(), F("flowTempLowLoss"), F_(degrees), nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(flowsettemp_), device_type(), F_(flowsettemp), F_(degrees), nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(flowtemphc_), device_type(), F_(flowtemphc), F_(degrees), nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(flowtempvf_), device_type(), F_(flowtempvf), F_(degrees), nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(pumpstatus_), device_type(), F_(pumpstatus), nullptr, F_(iconpump));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(valvestatus_), device_type(), F_(valvestatus), F_(percent), F_(iconpercent));
    } else {
        // WWC
        snprintf_P(&topic[0], topic.capacity() + 1, PSTR("%s%s/%s_wwc%d/%s"),Fc_(hasensor), Mqtt::base().c_str(), Fc_(mixer), hc_, Fc_(config));
        Mqtt::publish_ha(topic, doc.as<JsonObject>()); // publish the config payload with retain flag
        char wwc_name[10];
        snprintf_P(wwc_name, sizeof(wwc_name), PSTR("wwc%d"), hc_);
        Mqtt::register_mqtt_ha_sensor(wwc_name, nullptr, F_(wwtemp_), device_type(), F_(wwtemp), F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(wwc_name, nullptr, F_(pumpstatus_), device_type(), F_(pumpstatus), nullptr, F_(iconpump));
        Mqtt::register_mqtt_ha_sensor(wwc_name, nullptr, F_(tempstatus_), device_type(), F_(tempstatus), nullptr, nullptr);
    }

    mqtt_ha_config_ = true; // done
}

bool Mixer::export_values(JsonObject & json, int8_t id) {
    if ((id <= 0) || (type() == Type::HC && id == hc_) || (type() == Type::WWC && id == hc_ + 8)) {
        return export_values_format(Mqtt::Format::NESTED, json);
    }
    return false;
}

// creates JSON doc from values
// returns false if empty
bool Mixer::export_values_format(uint8_t mqtt_format, JsonObject & json) {
    // check if there is data for the mixer unit
    if (type() == Type::NONE) {
        return 0;
    }

    JsonObject json_hc;
    char       hc_name[10]; // hc{1-4}

    if (type() == Type::HC) {
        snprintf_P(hc_name, sizeof(hc_name), PSTR("hc%d"), hc_);
        if (mqtt_format == Mqtt::Format::SINGLE) {
            json_hc      = json;
            json[F_(type)] = F_(hc);
        } else if (mqtt_format == Mqtt::Format::HA) {
            json_hc         = json.createNestedObject(hc_name);
            json_hc[F_(type)] = F_(hc);
        } else {
            json_hc = json.createNestedObject(hc_name);
        }
        // T0: flow temperature on the low loss header
        // if (Helpers::hasValue(flowTempLowLoss_)) {
        //     json_hc[F("flowTempLowLoss")] = flowTempLowLoss_;
        // }
        // Setpoint for heating circuit
        if (Helpers::hasValue(flowSetTemp_)) {
            json_hc[F_(flowsettemp)] = flowSetTemp_;
        }
        // TC1: flow temperature in assigned hc or tank temperature in assigned tank primary circuit
        if (Helpers::hasValue(flowTempHc_)) {
            json_hc[F_(flowtemphc)] = (float)flowTempHc_ / 10;
        }
        // VF: flow temperature in Header
        if (Helpers::hasValue(flowTempVf_)) {
            json_hc[F_(flowtempvf)] = (float)flowTempVf_ / 10;
        }
        // PC1: heating pump in assigned hc -or- PW1: tank primary pump in assigned tank primary circuit (code switch 9 or 10)
        Helpers::json_boolean(json_hc, F_(pumpstatus), pumpStatus_);
        // VC1: mixing valve actuator in the assigned hc with mixer -or- PW2: DHW circulation pump with connection to module (code switch 9 or 10)
        if (Helpers::hasValue(status_)) {
            json_hc[F_(valvestatus)] = status_;
        }

        return json_hc.size();
    }

    // WWC
    snprintf_P(hc_name, sizeof(hc_name), PSTR("wwc%d"), hc_);
    if (mqtt_format == Mqtt::Format::SINGLE) {
        json_hc      = json;
        json[F_(type)] = F_(wwc);
    } else if (mqtt_format == Mqtt::Format::HA) {
        json_hc         = json.createNestedObject(hc_name);
        json_hc[F_(type)] = F_(wwc);
    } else {
        json_hc = json.createNestedObject(hc_name);
    }
    if (Helpers::hasValue(flowTempHc_)) {
        json_hc[F_(wwtemp)] = (float)flowTempHc_ / 10;
    }
    Helpers::json_boolean(json_hc, F_(pumpstatus), pumpStatus_);
    if (Helpers::hasValue(status_)) {
        json_hc[F_(tempstatus)] = status_;
    }

    return json_hc.size();
}

/*
// heating circuits 0x02E1, 0x02E2 etc...
// e.g.  Thermostat(0x10) -> Mixing Module(0x20), (0x2E1), data: 01 00 00 00 01
// other single byte message 0x2EB,..:  Thermostat(0x10) -> Mixing Module(0x20), (0x2EB), data: 00
void Mixer::process_MMPLUSStetMessage_HC(std::shared_ptr<const Telegram> telegram) {
    type(Type::HC);
    hc_ = telegram->type_id - 0x02E1 + 1;
    changed_ |= telegram->read_value(flowSetTemp_, ?);
    changed_ |= telegram->read_bitvalue(pumpStatus_, ?, 0);
}

*/
/*
// SetMessage for ww circuits:
// e.g. Thermostat(0x10) -> Mixing Module(0x28), (0x33B), data: 00 02 00
*/

// heating circuits 0x02D7, 0x02D8 etc...
// e.g.  A0 00 FF 00 01 D7 00 00 00 80 00 00 00 00 03 C5
//       A0 0B FF 00 01 D7 00 00 00 80 00 00 00 00 03 80
void Mixer::process_MMPLUSStatusMessage_HC(std::shared_ptr<const Telegram> telegram) {
    type(Type::HC);
    hc_ = telegram->type_id - 0x02D7 + 1;              // determine which circuit this is
    changed_ |= telegram->read_value(flowSetTemp_, 5); // Requested Flow temperature (see Norberts list)
    changed_ |= telegram->read_value(flowTempHc_, 3);  // TC1, is * 10
    changed_ |= telegram->read_bitvalue(pumpStatus_, 0, 0);
    changed_ |= telegram->read_value(status_, 2); // valve status
}

// Mixer warm water loading/DHW - 0x0331, 0x0332
// e.g. A9 00 FF 00 02 32 02 6C 00 3C 00 3C 3C 46 02 03 03 00 3C // on 0x28
//      A8 00 FF 00 02 31 02 35 00 3C 00 3C 3C 46 02 03 03 00 3C // in 0x29
void Mixer::process_MMPLUSStatusMessage_WWC(std::shared_ptr<const Telegram> telegram) {
    type(Type::WWC);
    hc_ = telegram->type_id - 0x0331 + 1;             // determine which circuit this is. There are max 2.
    changed_ |= telegram->read_value(flowTempHc_, 0); // TC1, is * 10
    changed_ |= telegram->read_bitvalue(pumpStatus_, 2, 0);
    changed_ |= telegram->read_value(status_, 11); // temp status
}

// Mixer IPM - 0x010C
// e.g.  A0 00 FF 00 00 0C 01 00 00 00 00 00 54
//       A1 00 FF 00 00 0C 02 04 00 01 1D 00 82
void Mixer::process_IPMStatusMessage(std::shared_ptr<const Telegram> telegram) {
    type(Type::HC);
    hc_ = device_id() - 0x20 + 1;

    // check if circuit is active, 0-off, 1-unmixed, 2-mixed
    uint8_t ismixed = 0;
    telegram->read_value(ismixed, 0);
    if (ismixed == 0) {
        return;
    }

    // do we have a mixed circuit
    if (ismixed == 2) {
        changed_ |= telegram->read_value(flowTempHc_, 3); // TC1, is * 10
        changed_ |= telegram->read_value(status_, 2);     // valve status
    }

    changed_ |= telegram->read_bitvalue(pumpStatus_, 1, 0); // pump is also in unmixed circuits
    changed_ |= telegram->read_value(flowSetTemp_, 5);      // is also in unmixed circuits, see #711
}

// Mixer IPM - 0x001E Temperature Message for unmixed ccircuits (switch temperature)
// unmixed FlowTemp in 10C is zero, the flowtemp on header is published here
// republished from boiler in 0x19 as switchTemp.
void Mixer::process_IPMTempMessage(std::shared_ptr<const Telegram> telegram) {
    type(Type::HC);
    hc_ = device_id() - 0x20 + 1;

    changed_ |= telegram->read_value(flowTempVf_, 0); // VF, is * 10
    // no other values
}

// Mixer on a MM10 - 0xAB
// e.g. Mixer Module -> All, type 0xAB, telegram: 21 00 AB 00 2D 01 BE 64 04 01 00 (CRC=15) #data=7
// see also https://github.com/emsesp/EMS-ESP/issues/386
void Mixer::process_MMStatusMessage(std::shared_ptr<const Telegram> telegram) {
    type(Type::HC);

    // the heating circuit is determine by which device_id it is, 0x20 - 0x23
    // 0x21 is position 2. 0x20 is typically reserved for the WM10 switch module
    // see https://github.com/emsesp/EMS-ESP/issues/270 and https://github.com/emsesp/EMS-ESP/issues/386#issuecomment-629610918
    hc_ = device_id() - 0x20 + 1;
    changed_ |= telegram->read_value(flowSetTemp_, 0);      // Setpoint from MMSetMessage
    changed_ |= telegram->read_value(flowTempHc_, 1);       // FV, is * 10
    changed_ |= telegram->read_bitvalue(pumpStatus_, 3, 2); // is 0 or 0x64 (100%), check only bit 2
    changed_ |= telegram->read_value(status_, 4);           // valve status -100 to 100
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// Mixer on a MM10 - 0xAA
// e.g. Thermostat -> Mixer Module, type 0xAA, telegram: 10 21 AA 00 FF 0C 0A 11 0A 32 xx
void Mixer::process_MMConfigMessage(std::shared_ptr<const Telegram> telegram) {
    hc_ = device_id() - 0x20 + 1;
    // pos 0: active FF = on
    // pos 1: valve runtime 0C = 120 sec in units of 10 sec
}

// Mixer on a MM10 - 0xAC
// e.g. Thermostat -> Mixer Module, type 0xAC, telegram: 10 21 AC 00 1E 64 01 AB
void Mixer::process_MMSetMessage(std::shared_ptr<const Telegram> telegram) {
    hc_ = device_id() - 0x20 + 1;
    // pos 0: flowtemp setpoint 1E = 30°C
    // pos 1: position in %
}

#pragma GCC diagnostic pop

} // namespace emsesp
