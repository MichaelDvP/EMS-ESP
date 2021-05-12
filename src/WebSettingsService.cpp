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

#include "WebSettingsService.h"
#include "emsesp.h"

namespace emsesp {

uint8_t WebSettings::flags_;

WebSettingsService::WebSettingsService(AsyncWebServer * server, FS * fs, SecurityManager * securityManager)
    : _httpEndpoint(WebSettings::read, WebSettings::update, this, server, EMSESP_SETTINGS_SERVICE_PATH, securityManager)
    , _fsPersistence(WebSettings::read, WebSettings::update, this, fs, EMSESP_SETTINGS_FILE) {
    addUpdateHandler([&](const String & originId) { onUpdate(); }, false);
}

void WebSettings::read(WebSettings & settings, JsonObject & root) {
    root[F_(tx_mode)]               = settings.tx_mode;
    root[F_(tx_delay)]             = settings.tx_delay;
    root[F_(ems_bus_id)]           = settings.ems_bus_id;
    root[F_(syslog_enabled)]       = settings.syslog_enabled;
    root[F_(syslog_level)]         = settings.syslog_level;
    root[F_(trace_raw)]            = settings.trace_raw;
    root[F_(syslog_mark_interval)] = settings.syslog_mark_interval;
    root[F_(syslog_host)]          = settings.syslog_host;
    root[F_(syslog_port)]          = settings.syslog_port;
    root[F_(master_thermostat)]    = settings.master_thermostat;
    root[F_(shower_timer)]         = settings.shower_timer;
    root[F_(shower_alert)]         = settings.shower_alert;
    root[F_(rx_gpio)]              = settings.rx_gpio;
    root[F_(tx_gpio)]              = settings.tx_gpio;
    root[F_(dallas_gpio)]          = settings.dallas_gpio;
    root[F_(dallas_parasite)]      = settings.dallas_parasite;
    root[F_(led_gpio)]             = settings.led_gpio;
    root[F_(hide_led)]             = settings.hide_led;
    root[F_(notoken_api)]          = settings.notoken_api;
    root[F_(analog_enabled)]       = settings.analog_enabled;
    root[F_(solar_maxflow)]        = settings.solar_maxflow;
}

StateUpdateResult WebSettings::update(JsonObject & root, WebSettings & settings) {
    reset_flags();

    // tx_mode, rx and tx pins
    settings.tx_mode  = check_flag(settings.tx_mode, root[F_(tx_mode)] | EMSESP_DEFAULT_TX_MODE,ChangeFlags::UART);
    settings.tx_delay = check_flag(settings.tx_delay, root[F_(tx_delay)] | EMSESP_DEFAULT_TX_DELAY, ChangeFlags::UART);
    settings.rx_gpio  = check_flag(settings.rx_gpio, root[F_(rx_gpio)] | EMSESP_DEFAULT_RX_GPIO, ChangeFlags::UART);
    settings.tx_gpio  = settings.rx_gpio == 13 ? 15 : 1;

    // syslog
    settings.syslog_enabled       = check_flag(settings.syslog_enabled, root[F_(syslog_enabled)] | EMSESP_DEFAULT_SYSLOG_ENABLED, ChangeFlags::SYSLOG);
    settings.syslog_level         = check_flag(settings.syslog_level, root[F_(syslog_level)] | EMSESP_DEFAULT_SYSLOG_LEVEL, ChangeFlags::SYSLOG);
    settings.syslog_mark_interval = check_flag(settings.syslog_mark_interval, root[F_(syslog_mark_interval)] | EMSESP_DEFAULT_SYSLOG_MARK_INTERVAL, ChangeFlags::SYSLOG);
    settings.syslog_port          = check_flag(settings.syslog_port, root[F_(syslog_port)] | EMSESP_DEFAULT_SYSLOG_PORT, ChangeFlags::SYSLOG);
    settings.trace_raw            = check_flag(settings.trace_raw, root[F_(trace_raw)] | EMSESP_DEFAULT_TRACELOG_RAW, ChangeFlags::SYSLOG);
    EMSESP::trace_raw(settings.trace_raw);
    String host = settings.syslog_host;
    settings.syslog_host = root[F_(syslog_host)] | EMSESP_DEFAULT_SYSLOG_HOST;
    if (host != settings.syslog_host) {
        add_flags(ChangeFlags::SYSLOG);
    }

    // other
    settings.analog_enabled = check_flag(settings.analog_enabled, root[F_(analog_enabled)] | EMSESP_DEFAULT_ANALOG_ENABLED, ChangeFlags::OTHER);

    // dallas
    settings.dallas_gpio     = check_flag(settings.dallas_gpio, root[F_(dallas_gpio)] | EMSESP_DEFAULT_DALLAS_GPIO, ChangeFlags::DALLAS);
    settings.dallas_parasite = check_flag(settings.dallas_parasite, root[F_(dallas_parasite)] | EMSESP_DEFAULT_DALLAS_PARASITE, ChangeFlags::DALLAS);

    // shower
    settings.shower_timer = check_flag(settings.shower_timer, root[F_(shower_timer)] | EMSESP_DEFAULT_SHOWER_TIMER, ChangeFlags::SHOWER);
    settings.shower_alert = check_flag(settings.shower_alert, root[F_(shower_alert)] | EMSESP_DEFAULT_SHOWER_ALERT, ChangeFlags::SHOWER);

    // led
    settings.led_gpio = check_flag(settings.led_gpio, root[F_(led_gpio)] | EMSESP_DEFAULT_LED_GPIO, ChangeFlags::LED);
    settings.hide_led = check_flag(settings.hide_led, root[F_(hide_led)] | EMSESP_DEFAULT_HIDE_LED, ChangeFlags::LED);

    // these both need reboots to be applied
    settings.ems_bus_id        = root[F_(ems_bus_id)] | EMSESP_DEFAULT_EMS_BUS_ID;
    settings.master_thermostat = root[F_(master_thermostat)] | EMSESP_DEFAULT_MASTER_THERMOSTAT;

    // doesn't need any follow-up actions
    settings.notoken_api   = root[F_(notoken_api)] | EMSESP_DEFAULT_NOTOKEN_API;
    settings.solar_maxflow = root[F_(solar_maxflow)] | EMSESP_DEFAULT_SOLAR_MAXFLOW;

    return StateUpdateResult::CHANGED;
}

// this is called after any of the settings have been persisted to the filesystem
// either via the Web UI or via the Console
void WebSettingsService::onUpdate() {
    if (WebSettings::has_flags(WebSettings::ChangeFlags::SHOWER)) {
        EMSESP::shower_.start();
    }

    if (WebSettings::has_flags(WebSettings::ChangeFlags::DALLAS)) {
        EMSESP::dallassensor_.start();
    }

    if (WebSettings::has_flags(WebSettings::ChangeFlags::UART)) {
        EMSESP::init_tx();
    }

    if (WebSettings::has_flags(WebSettings::ChangeFlags::SYSLOG)) {
        System::syslog_init();
    }

    if (WebSettings::has_flags(WebSettings::ChangeFlags::OTHER)) {
        System::other_init();
    }

    if (WebSettings::has_flags(WebSettings::ChangeFlags::LED)) {
        System::led_init();
    }
}

void WebSettingsService::begin() {
    _fsPersistence.readFromFS();
}

void WebSettingsService::save() {
    _fsPersistence.writeToFS();
}

} // namespace emsesp