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

#include "shower.h"

namespace emsesp {

uuid::log::Logger Shower::logger_{F_(shower), uuid::log::Facility::CONSOLE};

void Shower::start() {
    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        shower_timer_ = settings.shower_timer;
        shower_alert_ = settings.shower_alert;
    });

    if (Mqtt::enabled()) {
        send_mqtt_stat(false); // send first MQTT publish
    }
}

void Shower::loop() {
    if (!shower_timer_) {
        return;
    }

    uint32_t time_now = uuid::get_uptime();

    // if already in cold mode, ignore all this logic until we're out of the cold blast
    if (!doing_cold_shot_) {
        // is the hot water running?
        if (EMSESP::tap_water_active()) {
            // if heater was previously off, start the timer
            if (timer_start_ == 0) {
                // hot water just started...
                timer_start_     = time_now;
                timer_pause_     = 0; // remove any last pauses
                doing_cold_shot_ = false;
                duration_        = 0;
                shower_on_       = false;
            } else {
                // hot water has been  on for a while
                // first check to see if hot water has been on long enough to be recognized as a Shower/Bath
                if (!shower_on_ && (time_now - timer_start_) > SHOWER_MIN_DURATION) {
                    shower_on_ = true;
                    send_mqtt_stat(true);
                    LOG_DEBUG(F("[Shower] hot water still running, starting shower timer"));
                }
                // check if the shower has been on too long
                else if ((((time_now - timer_start_) > SHOWER_MAX_DURATION) && !doing_cold_shot_) && shower_alert_) {
                    shower_alert_start();
                }
            }
        } else { // hot water is off
            // if it just turned off, record the time as it could be a short pause
            if ((timer_start_) && (timer_pause_ == 0)) {
                timer_pause_ = time_now;
            }

            // if shower has been off for longer than the wait time
            if ((timer_pause_) && ((time_now - timer_pause_) > SHOWER_PAUSE_TIME)) {
                // it is over the wait period, so assume that the shower has finished and calculate the total time and publish
                // because its unsigned long, can't have negative so check if length is less than OFFSET_TIME
                if ((timer_pause_ - timer_start_) > SHOWER_OFFSET_TIME) {
                    duration_ = (timer_pause_ - timer_start_ - SHOWER_OFFSET_TIME);
                    if (duration_ > SHOWER_MIN_DURATION) {
                        send_mqtt_stat(false);
                        LOG_DEBUG(F("[Shower] finished with duration %d"), duration_);
                        publish_values();
                    }
                }

                // reset everything
                timer_start_ = 0;
                timer_pause_ = 0;
                shower_on_   = false;
                shower_alert_stop();
            }
        }
    }
}

// send status of shower to MQTT
void Shower::send_mqtt_stat(bool state) {
    if (!shower_timer_ && !shower_alert_) {
        return;
    }

    //first sent out the HA MQTT Discovery config topic
    send_MQTT_discovery_config();

    char s[7];
    Mqtt::publish(F_(shower_active), Helpers::render_boolean(s, state));
}

// turn back on the hot water for the shower
void Shower::shower_alert_stop() {
    if (doing_cold_shot_) {
        LOG_DEBUG(F("Shower Alert stopped"));
        // Boiler::set_tapwarmwater_activated(true);
        doing_cold_shot_ = false;
        // showerColdShotStopTimer.detach(); // disable the timer
    }
}

// turn off hot water to send a shot of cold
void Shower::shower_alert_start() {
    if (shower_alert_) {
        LOG_DEBUG(F("Shower Alert started!"));
        // Boiler::set_tapwarmwater_activated(false);
        doing_cold_shot_ = true;
        // start the timer for n seconds which will reset the water back to hot
        // showerColdShotStopTimer.attach(SHOWER_COLDSHOT_DURATION, _showerColdShotStop);
    }
}

// Publish shower data
// returns true if added to MQTT queue went ok
void Shower::publish_values() {
    StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
    JsonObject                                     json = doc.to<JsonObject>();

    //first sent out the HA MQTT Discovery config topic
    send_MQTT_discovery_config();

    Helpers::json_boolean(json, F_(shower_timer), shower_timer_);
    Helpers::json_boolean(json, F_(shower_alert), shower_alert_);

    // only publish shower duration if there is a value
    if (duration_ > SHOWER_MIN_DURATION) {
        // char s[40];
        // snprintf_P(s, 40, PSTR("%d minutes and %d seconds"), (uint8_t)(duration_ / 60000), (uint8_t)((duration_ / 1000) % 60));
        json[F("duration")] = duration_ / 1000;
    }

    Mqtt::publish(F_(shower_data), doc.as<JsonObject>());
}

void Shower::send_MQTT_discovery_config() {
    if (mqtt_discovery_config_send_) {
        //nothing to do
        return;
    }

    //send the config depending on the MQTT format used
    if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
        StaticJsonDocument<EMSESP_MAX_JSON_SIZE_HA_CONFIG> doc;
        doc[F_(name)]        = F("Shower Data");
        doc[F_(uniq_id)]     = F_(shower_data);
        doc[F("~")]           = Mqtt::base();
        doc[F_(json_attr_t)] = F("~/shower_data");
        doc[F_(stat_t)]      = F("~/shower_data");
        doc[F_(val_tpl)]     = F("{{value_json['duration']}}");
        doc[F_(ic)]          = F("mdi:shower");
        JsonObject dev       = doc.createNestedObject(F_(dev));
        JsonArray  ids       = dev.createNestedArray(F_(ids));
        ids.add(F_(emsespboiler));

        std::string topic(128, '\0');
        snprintf_P(&topic[0], topic.capacity() + 1, PSTR("%s%s/%s/%s"),Fc_(hasensor), Mqtt::base().c_str(), Fc_(shower), Fc_(config));
        Mqtt::publish_ha(topic, doc.as<JsonObject>());

        Mqtt::register_mqtt_ha_binary_sensor(F("Shower Active"), EMSdevice::DeviceType::BOILER, F_(shower_active));

        mqtt_discovery_config_send_ = true;
    } else {
        //no valiid config defined
        mqtt_discovery_config_send_ = true;
    }
}

} // namespace emsesp
