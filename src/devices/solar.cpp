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

#include "solar.h"

namespace emsesp {

REGISTER_FACTORY(Solar, EMSdevice::DeviceType::SOLAR);

uuid::log::Logger Solar::logger_{F_(solar), uuid::log::Facility::CONSOLE};

Solar::Solar(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand)
    : EMSdevice(device_type, device_id, product_id, version, name, flags, brand) {
    LOG_DEBUG(F("Adding new Solar module with device ID 0x%02X"), device_id);

    // telegram handlers
    if (flags == EMSdevice::EMS_DEVICE_FLAG_SM10) {
        register_telegram_type(0x0097, F("SM10Monitor"), true, [&](std::shared_ptr<const Telegram> t) { process_SM10Monitor(t); });
    }

    if (flags == EMSdevice::EMS_DEVICE_FLAG_SM100) {
        if (device_id == 0x2A) {
            register_telegram_type(0x07D6, F("SM100wwTemperature"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100wwTemperature(t); });
            register_telegram_type(0x07AA, F("SM100wwStatus"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100wwStatus(t); });
            register_telegram_type(0x07AB, F("SM100wwCommand"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100wwCommand(t); });
        } else {
            register_telegram_type(0xF9, F("ParamCfg"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100ParamCfg(t); });
            register_telegram_type(0x0358, F("SM100SystemConfig"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100SystemConfig(t); });
            register_telegram_type(0x035A, F("SM100SolarCircuitConfig"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100SolarCircuitConfig(t); });
            register_telegram_type(0x0362, F("SM100Monitor"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Monitor(t); });
            register_telegram_type(0x0363, F("SM100Monitor2"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Monitor2(t); });
            register_telegram_type(0x0366, F("SM100Config"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Config(t); });
            register_telegram_type(0x0364, F("SM100Status"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100Status(t); });
            register_telegram_type(0x036A, F("SM100Status2"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100Status2(t); });
            register_telegram_type(0x0380, F("SM100CollectorConfig"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100CollectorConfig(t); });
            register_telegram_type(0x038E, F("SM100Energy"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Energy(t); });
            register_telegram_type(0x0391, F("SM100Time"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Time(t); });

            register_mqtt_cmd(F_(tankbottommaxtemp), [&](const char * value, const int8_t id) { return set_SM100TankBottomMaxTemp(value, id); });
        }
    }

    if (flags == EMSdevice::EMS_DEVICE_FLAG_ISM) {
        register_telegram_type(0x0103, F("ISM1StatusMessage"), true, [&](std::shared_ptr<const Telegram> t) { process_ISM1StatusMessage(t); });
        register_telegram_type(0x0101, F("ISM1Set"), false, [&](std::shared_ptr<const Telegram> t) { process_ISM1Set(t); });
    }
}

// print to web
void Solar::device_info_web(JsonArray & root, uint8_t & part) {
    // fetch the values into a JSON document
    // StaticJsonDocument<EMSESP_MAX_JSON_SIZE_MEDIUM> doc;
    // JsonObject                                      json = doc.to<JsonObject>();
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_MEDIUM);
    JsonObject          json = doc.to<JsonObject>();
    if (!export_values(json)) {
        return; // empty
    }
    doc.shrinkToFit();
    create_value_json(root, F_(collectortemp), nullptr, F_(collectortemp_), F_(degrees), json);
    create_value_json(root, F_(collectormaxtemp), nullptr, F_(collectormaxtemp_), F_(degrees), json);
    create_value_json(root, F_(tankbottomtemp), nullptr, F_(tankbottomtemp_), F_(degrees), json);
    create_value_json(root, F_(tank2bottomtemp), nullptr, F_(tank2bottomtemp_), F_(degrees), json);
    create_value_json(root, F_(tankbottommaxtemp), nullptr, F_(tankbottommaxtemp_), F_(degrees), json);
    create_value_json(root, F_(heatexchangertemp), nullptr, F_(heatexchangertemp_), F_(degrees), json);
    create_value_json(root, F_(solarpumpmodulation), nullptr, F_(solarpumpmodulation_), F_(percent), json);
    create_value_json(root, F_(cylinderpumpmodulation), nullptr, F_(cylinderpumpmodulation_), F_(percent), json);
    create_value_json(root, F_(valvestatus), nullptr, F_(valvestatus_), nullptr, json);
    create_value_json(root, F_(solarpump), nullptr, F_(solarpump_), nullptr, json);
    create_value_json(root, F_(tankheated), nullptr, F_(tankheated_), nullptr, json);
    create_value_json(root, F_(collectorshutdown), nullptr, F_(collectorshutdown_), nullptr, json);
    create_value_json(root, F_(energylasthour), nullptr, F_(energylasthour_), F_(wh), json);
    create_value_json(root, F_(energytoday), nullptr, F_(energytoday_), F_(wh), json);
    create_value_json(root, F_(energytotal), nullptr, F_(energytotal_), F_(kwh), json);
    // create_value_json(root, F_(pumpworktime), nullptr, F_(pumpworktime_), F_(min), json);
    create_value_json(root, F_(pumpworktimetext), nullptr, F_(pumpworktimetext_), nullptr, json);
}

// publish values via MQTT
void Solar::publish_values(JsonObject & json, bool force) {
    // handle HA first
    if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
        if ((!mqtt_ha_config_ || force)) {
            register_mqtt_ha_config();
            return;
        }
    }

    // StaticJsonDocument<EMSESP_MAX_JSON_SIZE_MEDIUM> doc;
    // JsonObject                                      json_payload = doc.to<JsonObject>();
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_MEDIUM);
    JsonObject          json_payload = doc.to<JsonObject>();
    if (export_values(json_payload)) {
        doc.shrinkToFit();
        if (device_id() == 0x2A) {
            Mqtt::publish(F("ww_data"), doc.as<JsonObject>());
        } else {
            Mqtt::publish(F("solar_data"), doc.as<JsonObject>());
        }
    }
}

// publish config topic for HA MQTT Discovery
void Solar::register_mqtt_ha_config() {
    if (!Mqtt::connected()) {
        return;
    }

    // Create the Master device
    StaticJsonDocument<EMSESP_MAX_JSON_SIZE_HA_CONFIG> doc;
    doc[F_(name)]    = F_(EMSESP);
    doc[F_(uniq_id)] = F_(solar);
    doc[F_(ic)]      = F_(iconthermostat);

    char stat_t[128];
    snprintf_P(stat_t, sizeof(stat_t), PSTR("%s/%s"), Mqtt::base().c_str(), Fc_(solar_data));
    doc[F_(stat_t)] = stat_t;

    doc[F_(val_tpl)] = F("{{value_json.solarpump}}");
    JsonObject dev   = doc.createNestedObject(F_(dev));
    dev[F_(name)]    = F("EMS-ESP Solar");
    dev[F_(sw)]      = EMSESP_APP_VERSION;
    dev[F_(mf)]      = brand_to_string();
    dev[F_(mdl)]     = name();
    JsonArray ids    = dev.createNestedArray(F_(ids));
    ids.add(F("ems-esp-solar"));

    std::string topic(128, '\0');
    snprintf_P(&topic[0], topic.capacity() + 1, PSTR("%s%s/%s/%s"),Fc_(hasensor), Mqtt::base().c_str(), Fc_(solar), Fc_(config));
    Mqtt::publish_ha(topic, doc.as<JsonObject>()); // publish the config payload with retain flag

    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(collectortemp_), device_type(), F_(collectortemp), F_(degrees), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(collectormaxtemp_), device_type(), F_(collectormaxtemp), F_(degrees), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(collectormintemp_), device_type(), F_(collectormintemp), F_(degrees), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(tankbottomtemp_), device_type(), F_(tankbottomtemp), F_(degrees), nullptr);
    // Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(tankMiddleTemp), device_type(), F_(tankmiddletemp), F_(degrees), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(tank2bottomtemp_), device_type(), F_(tank2bottomtemp), F_(degrees), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(tankbottommaxtemp_), device_type(), F_(tankbottommaxtemp), F_(degrees), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(heatexchangertemp_), device_type(), F_(heatexchangertemp), F_(degrees), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(solarpumpmodulation_), device_type(), F_(solarpumpmodulation), F_(percent), F_(iconpercent));
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(cylinderpumpmodulation_), device_type(), F_(cylinderpumpmodulation), F_(percent), F_(iconpercent));
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(pumpworktime_), device_type(), F_(pumpworktime), F_(min), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(energylasthour_), device_type(), F_(energylasthour), F_(wh), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(energytoday_), device_type(), F_(energytoday), F_(wh), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(energytotal_), device_type(), F_(energytotal), F_(kwh), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(solarpump_), device_type(), F_(solarpump), nullptr, F_(iconpump));
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(valvestatus_), device_type(), F_(valvestatus), nullptr, F_(iconvalve));
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(tankheated_), device_type(), F_(tankheated), nullptr, nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(collectorshutdown_), device_type(), F_(collectorshutdown), nullptr, nullptr);

    mqtt_ha_config_ = true; // done
}

// creates JSON doc from values
// returns false if empty
bool Solar::export_values(JsonObject & json, int8_t id) {
    // collector array temperature (TS1)
    if (Helpers::hasValue(collectorTemp_)) {
        json[F_(collectortemp)] = (float)collectorTemp_ / 10;
    }
    // tank bottom temperature (TS2)
    if (Helpers::hasValue(tankBottomTemp_)) {
        json[F_(tankbottomtemp)] = (float)tankBottomTemp_ / 10;
    }
    // tank middle temperature (TS3)
    // if (Helpers::hasValue(tankMiddleTemp_)) {
    // json["tankMiddleTemp"] = (float)tankMiddleTemp_ / 10;
    // }
    // second tank bottom temperature or swimming pool (TS5)
    if (Helpers::hasValue(tank2BottomTemp_)) {
        json[F_(tank2bottomtemp)] = (float)tank2BottomTemp_ / 10;
    }
    // temperature heat exchanger (TS6)
    if (Helpers::hasValue(heatExchangerTemp_)) {
        json[F_(heatexchangertemp)] = (float)heatExchangerTemp_ / 10;
    }

    if (Helpers::hasValue(tankBottomMaxTemp_)) {
        json[F_(tankbottommaxtemp)] = tankBottomMaxTemp_;
    }

    if (Helpers::hasValue(collectorMaxTemp_)) {
        json[F_(collectormaxtemp)] = collectorMaxTemp_;
    }

    if (Helpers::hasValue(collectorMinTemp_)) {
        json[F_(collectormintemp)] = collectorMinTemp_;
    }

    if (Helpers::hasValue(solarPumpModulation_)) {
        json[F_(solarpumpmodulation)] = solarPumpModulation_;
    }

    if (Helpers::hasValue(cylinderPumpModulation_)) {
        json[F_(cylinderpumpmodulation)] = cylinderPumpModulation_;
    }

    Helpers::json_boolean(json, F_(solarpump), solarPump_);

    Helpers::json_boolean(json, F_(valvestatus), valveStatus_);

    Helpers::json_time(json, F_(pumpworktimetext), pumpWorkTime_, true);
    if (Helpers::hasValue(pumpWorkTime_)) {
        json[F_(pumpworktime)] = pumpWorkTime_;
    }

    Helpers::json_boolean(json, F_(tankheated), tankHeated_);

    Helpers::json_boolean(json, F_(collectorshutdown), collectorShutdown_);

    if (Helpers::hasValue(energyLastHour_)) {
        json[F_(energylasthour)] = (float)energyLastHour_ / 10;
    }

    if (Helpers::hasValue(energyToday_)) {
        json[F_(energytoday)] = energyToday_;
    }

    if (Helpers::hasValue(energyTotal_)) {
        json[F_(energytotal)] = (float)energyTotal_ / 10;
    }

    return json.size();
}

// check to see if values have been updated
bool Solar::updated_values() {
    if (changed_) {
        changed_ = false;
        return true;
    }
    return false;
}

// SM10Monitor - type 0x97
void Solar::process_SM10Monitor(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(collectorTemp_, 2);       // is *10 - TS1: collector temp from SM10
    changed_ |= telegram->read_value(tankBottomTemp_, 5);      // is *10 - TS2: Temperature sensor tank bottom
    changed_ |= telegram->read_value(solarPumpModulation_, 4); // modulation solar pump
    changed_ |= telegram->read_bitvalue(solarPump_, 7, 1);     // PS1: solar pump on (1) or off (0)
    changed_ |= telegram->read_value(pumpWorkTime_, 8, 3);
}

/*
 * process_SM100SystemConfig - type 0x0358 EMS+ - for MS/SM100 and MS/SM200
 * e.g. B0 0B FF 00 02 58 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF 00 FF 01 00 00
 */
void Solar::process_SM100SystemConfig(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(heatTransferSystem_, 5, 1);
    changed_ |= telegram->read_value(externalTank_, 9, 1);
    changed_ |= telegram->read_value(thermalDisinfect_, 10, 1);
    changed_ |= telegram->read_value(heatMetering_, 14, 1);
    changed_ |= telegram->read_value(solarIsEnabled_, 19, 1);
}

/*
 * process_SM100SolarCircuitConfig - type 0x035A EMS+ - for MS/SM100 and MS/SM200
 * e.g. B0 0B FF 00 02 5A 64 05 00 58 14 01 01 32 64 00 00 00 5A 0C
 */
void Solar::process_SM100SolarCircuitConfig(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(collectorMaxTemp_, 0, 1);
    changed_ |= telegram->read_value(tankBottomMaxTemp_, 3, 1);
    changed_ |= telegram->read_value(collectorMinTemp_, 4, 1);
    changed_ |= telegram->read_value(solarPumpMode_, 5, 1);
    changed_ |= telegram->read_value(solarPumpMinRPM_, 6, 1);
    changed_ |= telegram->read_value(solarPumpTurnoffDiff_, 7, 1);
    changed_ |= telegram->read_value(solarPumpTurnonDiff_, 8, 1);
    changed_ |= telegram->read_value(solarPumpKick_, 9, 1);
    changed_ |= telegram->read_value(plainWaterMode_, 10, 1);
    changed_ |= telegram->read_value(doubleMatchFlow_, 11, 1);
}

/* process_SM100ParamCfg - type 0xF9 EMS 1.0
 * This telegram is used to inquire the min, default, max, and current values of a value that is usually read and written with another telegram ID
 * The CS200 uses this method extensively to find out which values may be set in the SM100
 * e.g. B0 10 F9 00 FF 02 5A 03 17 00 00 00 14 00 00 00 3C 00 00 00 5A 00 00 00 59 29 - requested with 90 B0 F9 00 11 FF 02 5A 03 AF
 * byte 0 = 0xFF
 * byte 1-2 = telegram ID used to write this value
 * byte 3 = offset in telegram used to write this value
 * byte 4 = unknown
 * bytes 5..8 = minimum value
 * bytes 9..12 = default value
 * bytes 13..16 = maximum value
 * bytes 17..20 = current value
 *
 * e.g. B0 0B F9 00 00 02 5A 00 00 6E
 */
void Solar::process_SM100ParamCfg(std::shared_ptr<const Telegram> telegram) {
    uint16_t t_id;
    uint8_t  of;
    int32_t  min, def, max, cur;
    telegram->read_value(t_id, 1);
    telegram->read_value(of, 3);
    telegram->read_value(min, 5);
    telegram->read_value(def, 9);
    telegram->read_value(max, 13);
    telegram->read_value(cur, 17);

    // LOG_DEBUG(F("SM100ParamCfg param=0x%04X, offset=%d, min=%d, default=%d, max=%d, current=%d"), t_id, of, min, def, max, cur);
}

/*
 * SM100Monitor - type 0x0362 EMS+ - for MS/SM100 and MS/SM200
 * e.g. B0 0B FF 00 02 62 00 77 01 D4 80 00 80 00 80 00 80 00 80 00 80 00 80 00 80 00 00 F9 80 00 80 9E - for heat exchanger temp
 * e.g, 30 00 FF 00 02 62 01 AC
 *      30 00 FF 18 02 62 80 00
 *      30 00 FF 00 02 62 01 A1 - for bottom temps
 * bytes 0+1 = TS1 Temperature sensor for collector
 * bytes 2+3 = TS2 Temperature sensor tank 1 bottom
 * bytes 16+17 = TS5 Temperature sensor tank 2 bottom or swimming pool
 * bytes 20+21 = TS6 Temperature sensor external heat exchanger
 */
void Solar::process_SM100Monitor(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(collectorTemp_, 0);      // is *10 - TS1: Temperature sensor for collector array 1
    changed_ |= telegram->read_value(tankBottomTemp_, 2);     // is *10 - TS2: Temperature sensor 1st cylinder, bottom
    changed_ |= telegram->read_value(tank2BottomTemp_, 16);   // is *10 - TS5: Temperature sensor 2nd cylinder, bottom, or swimming pool
    changed_ |= telegram->read_value(heatExchangerTemp_, 20); // is *10 - TS6: Heat exchanger temperature sensor
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// SM100Monitor2 - 0x0363
// e.g. B0 00 FF 00 02 63 80 00 80 00 00 00 80 00 80 00 80 00 00 80 00 5A
void Solar::process_SM100Monitor2(std::shared_ptr<const Telegram> telegram) {
    // not implemented yet
}

// SM100wwTemperature - 0x07D6
// Solar Module(0x2A) -> (0x00), (0x7D6), data: 01 C1 00 00 02 5B 01 AF 01 AD 80 00 01 90
void Solar::process_SM100wwTemperature(std::shared_ptr<const Telegram> telegram) {
    // changed_ |= telegram->read_value(wwTemp_1_, 0);
    // changed_ |= telegram->read_value(wwTemp_3_, 4);
    // changed_ |= telegram->read_value(wwTemp_4_, 6);
    // changed_ |= telegram->read_value(wwTemp_5_, 8);
    // changed_ |= telegram->read_value(wwTemp_7_, 12);
}

// SM100wwStatus - 0x07AA
// Solar Module(0x2A) -> (0x00), (0x7AA), data: 64 00 04 00 03 00 28 01 0F
void Solar::process_SM100wwStatus(std::shared_ptr<const Telegram> telegram) {
    // changed_ |= telegram->read_value(wwPump_, 0);
}

// SM100wwCommand - 0x07AB
// Thermostat(0x10) -> Solar Module(0x2A), (0x7AB), data: 01 00 01
void Solar::process_SM100wwCommand(std::shared_ptr<const Telegram> telegram) {
    // not implemented yet
}


#pragma GCC diagnostic pop

// SM100Config - 0x0366
// e.g. B0 00 FF 00 02 66     01 62 00 13 40 14
void Solar::process_SM100Config(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(availabilityFlag_, 0);
    changed_ |= telegram->read_value(configFlag_, 1);
    changed_ |= telegram->read_value(userFlag_, 2);
}

/*
 * SM100Status - type 0x0364 EMS+ for pump modulations - for MS/SM100 and MS/SM200
 - PS1: Solar circuit pump for collector array 1
 - PS5: Cylinder primary pump when using an external heat exchanger
 * e.g. 30 00 FF 09 02 64 64 = 100%
 */
void Solar::process_SM100Status(std::shared_ptr<const Telegram> telegram) {
    uint8_t solarpumpmod    = solarPumpModulation_;
    uint8_t cylinderpumpmod = cylinderPumpModulation_;
    changed_ |= telegram->read_value(cylinderPumpModulation_, 8);
    changed_ |= telegram->read_value(solarPumpModulation_, 9);

    if (solarpumpmod == 0 && solarPumpModulation_ == 100) { // mask out boosts
        solarPumpModulation_ = 15;                          // set to minimum
    }

    if (cylinderpumpmod == 0 && cylinderPumpModulation_ == 100) { // mask out boosts
        cylinderPumpModulation_ = 15;                             // set to minimum
    }
    changed_ |= telegram->read_bitvalue(tankHeated_, 3, 1);        // issue #422
    changed_ |= telegram->read_bitvalue(collectorShutdown_, 3, 0); // collector shutdown
}

/*
 * SM100Status2 - type 0x036A EMS+ for pump on/off at offset 0x0A - for SM100 and SM200
 * e.g. B0 00 FF 00 02 6A 03 03 03 03 01 03 03 03 03 03 01 03
 * byte 4 = VS2 3-way valve for cylinder 2 : test=01, on=04 and off=03
 * byte 10 = PS1 Solar circuit pump for collector array 1: test=b0001(1), on=b0100(4) and off=b0011(3)
 */
void Solar::process_SM100Status2(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_bitvalue(valveStatus_, 4, 2); // on if bit 2 set
    changed_ |= telegram->read_bitvalue(solarPump_, 10, 2);  // PS1: solar circuit pump on (1) or off (0), on if bit 2 set
}

/*
 * SM100CollectorConfig - type 0x0380 EMS+  - for SM100 and SM200
 * e.g. B0 0B FF 00 02 80 50 64 00 00 29 01 00 00 01
 */
void Solar::process_SM100CollectorConfig(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(climateZone_, 0, 1);
    changed_ |= telegram->read_value(collector1Area_, 3, 2);
    changed_ |= telegram->read_value(collector1Type_, 5, 1);
}

/*
 * SM100Energy - type 0x038E EMS+ for energy readings
 * e.g. 30 00 FF 00 02 8E 00 00 00 00 00 00 06 C5 00 00 76 35
 */
void Solar::process_SM100Energy(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(energyLastHour_, 0); // last hour / 10 in Wh
    changed_ |= telegram->read_value(energyToday_, 4);    // todays in Wh
    changed_ |= telegram->read_value(energyTotal_, 8);    // total / 10 in kWh
}

/*
 * SM100Time - type 0x0391 EMS+ for pump working time
 */
void Solar::process_SM100Time(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(pumpWorkTime_, 1, 3);
}

/*
 * Junkers ISM1 Solar Module - type 0x0103 EMS+ for energy readings
 *  e.g. B0 00 FF 00 00 03 32 00 00 00 00 13 00 D6 00 00 00 FB D0 F0
 */
void Solar::process_ISM1StatusMessage(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(collectorTemp_, 4);  // is *10 - TS1: Temperature sensor for collector array 1
    changed_ |= telegram->read_value(tankBottomTemp_, 6); // is *10 - TS2: Temperature sensor 1st cylinder, bottom
    uint16_t Wh = 0xFFFF;
    changed_ |= telegram->read_value(Wh, 2); // Solar Energy produced in last hour only ushort, is not * 10

    if (Wh != 0xFFFF) {
        energyLastHour_ = Wh * 10; // set to *10
    }

    changed_ |= telegram->read_bitvalue(solarPump_, 8, 0);         // PS1: solar circuit pump on (1) or off (0)
    changed_ |= telegram->read_value(pumpWorkTime_, 10, 3);        // force to 3 bytes
    changed_ |= telegram->read_bitvalue(collectorShutdown_, 9, 0); // collector shutdown on/off
    changed_ |= telegram->read_bitvalue(tankHeated_, 9, 2);        // tankBottomTemp reached tankBottomMaxTemp
}

/*
 * Junkers ISM1 Solar Module - type 0x0101 EMS+ for setting values
 */
void Solar::process_ISM1Set(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(setpoint_tankBottomMaxTemp_, 6);
}

// set temperature for maximum tankBottomTemp
bool Solar::set_SM100TankBottomMaxTemp(const char * value, const int8_t id) {
    int temperature;
    if (!Helpers::value2number(value, temperature)) {
        return false;
    }

    // write value
    // 90 30 FF 03 02 5A 59 B3
    // note: optionally add the validate to 0x035A which will pick up the adjusted tankBottomMaxTemp_
    write_command(0x35A, 0x03, (uint8_t)temperature);

    return true;
}

} // namespace emsesp
