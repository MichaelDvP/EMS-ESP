/*
 * EMS-ESP - https://github.com/proddy/EMS-ESP
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

#ifndef EMSESP_HEATPUMP_H
#define EMSESP_HEATPUMP_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include <uuid/log.h>

#include "emsdevice.h"
#include "emsesp.h"
#include "telegram.h"
#include "helpers.h"
#include "mqtt.h"

namespace emsesp {

class Heatpump : public EMSdevice {
  public:
    Heatpump(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand);

    virtual void publish_values(JsonObject & json, bool force);
    virtual bool export_values(JsonObject & json);
    virtual void device_info_web(JsonArray & root, uint8_t & part);
    virtual bool updated_values();

  private:
    static uuid::log::Logger logger_;

    void register_mqtt_ha_config();

    uint8_t airHumidity_    = EMS_VALUE_UINT_NOTSET;
    uint8_t dewTemperature_ = EMS_VALUE_UINT_NOTSET;

    bool changed_        = false;
    bool mqtt_ha_config_ = false; // for HA MQTT Discovery

    void process_HPMonitor1(std::shared_ptr<const Telegram> telegram);
    void process_HPMonitor2(std::shared_ptr<const Telegram> telegram);
};

} // namespace emsesp

#endif
