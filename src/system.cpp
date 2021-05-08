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

#include "system.h"
#include "emsesp.h"  // for send_raw_telegram() command
#include "version.h" // firmware version of EMS-ESP

#if defined(ESP32)
#include "driver/adc.h"
#include <esp_bt.h>
#endif

#if defined(EMSESP_TEST)
#include "test/test.h"
#endif

namespace emsesp {

uuid::log::Logger System::logger_{F_(system), uuid::log::Facility::KERN};

#ifndef EMSESP_STANDALONE
uuid::syslog::SyslogService System::syslog_;
#endif

// init statics
uint32_t System::heap_start_     = 0;
bool     System::upload_status_  = false;
bool     System::hide_led_       = false;
uint8_t  System::led_gpio_       = 0;
uint16_t System::analog_         = 0;
bool     System::analog_enabled_ = false;
bool     System::syslog_enabled_ = false;
PButton  System::myPButton_;

// send on/off to a gpio pin
// value: true = HIGH, false = LOW
// http://ems-esp/api?device=system&cmd=pin&data=1&id=2
bool System::command_pin(const char * value, const int8_t id) {
    if (id < 1) { // pin 0 used for PButton
        return false;
    }

    bool        v  = false;
    std::string v1 = {7, '\0'};
#if defined(ESP32)
    int v2 = 0;
    if (id == 25 && Helpers::value2number(value, v2)) {
        if (v2 >= 0 && v2 <= 255) {
            dacWrite(id, v12;
            return true;
        }
    } else
#endif
    if (Helpers::value2bool(value, v)) {
        pinMode(id, OUTPUT);
        digitalWrite(id, v);
        LOG_INFO(F("GPIO %d set to %s"), id, v ? "HIGH" : "LOW");
        return true;
    } else if (Helpers::value2string(value, v1)) {
        if (v1 == "input" || v1 == "in" || v1 == "-1") {
            pinMode(id, INPUT);
            v = digitalRead(id);
            LOG_INFO(F("GPIO %d set input, state %s"), id, v ? "HIGH" : "LOW");
            return true;
        }
    }

    return false;
}

// send raw to ems
bool System::command_send(const char * value, const int8_t id) {
    EMSESP::send_raw_telegram(value); // ignore id
    return true;
}

// fetch device values
bool System::command_fetch(const char * value, const int8_t id) {
    std::string value_s(14, '\0');
    if (Helpers::value2string(value, value_s)) {
        if (value_s == "all") {
            LOG_INFO(F("Requesting data from EMS devices"));
            EMSESP::fetch_device_values();
            return true;
        } else if (value_s == uuid::read_flash_string(F_(boiler))) {
            EMSESP::fetch_device_values_type(EMSdevice::DeviceType::BOILER);
            return true;
        } else if (value_s == uuid::read_flash_string(F_(thermostat))) {
            EMSESP::fetch_device_values_type(EMSdevice::DeviceType::THERMOSTAT);
            return true;
        } else if (value_s == uuid::read_flash_string(F_(solar))) {
            EMSESP::fetch_device_values_type(EMSdevice::DeviceType::SOLAR);
            return true;
        } else if (value_s == uuid::read_flash_string(F_(mixer))) {
            EMSESP::fetch_device_values_type(EMSdevice::DeviceType::MIXER);
            return true;
        }
    }
    EMSESP::fetch_device_values(); // default if no name or id is given
    return true;
}

// mqtt publish
bool System::command_publish(const char * value, const int8_t id) {
    std::string value_s(14, '\0');
    if (Helpers::value2string(value, value_s)) {
        if (value_s == "ha") {
            EMSESP::publish_all(true); // includes HA
            LOG_INFO(F("Publishing all data to MQTT, including HA configs"));
            return true;
        } else if (value_s == uuid::read_flash_string(F_(boiler))) {
            EMSESP::publish_device_values(EMSdevice::DeviceType::BOILER);
            return true;
        } else if (value_s == uuid::read_flash_string(F_(thermostat))) {
            EMSESP::publish_device_values(EMSdevice::DeviceType::THERMOSTAT);
            return true;
        } else if (value_s == uuid::read_flash_string(F_(solar))) {
            EMSESP::publish_device_values(EMSdevice::DeviceType::SOLAR);
            return true;
        } else if (value_s == uuid::read_flash_string(F_(mixer))) {
            EMSESP::publish_device_values(EMSdevice::DeviceType::MIXER);
            return true;
        } else if (value_s == uuid::read_flash_string(F_(other))) {
            EMSESP::publish_other_values();
            return true;
        } else if (value_s == uuid::read_flash_string(F_(dallassensor))) {
            EMSESP::publish_sensor_values(true);
            return true;
        }
    }

    EMSESP::publish_all(); // ignore value and id
    LOG_INFO(F("Publishing all data to MQTT"));
    return true;
}

// restart EMS-ESP
void System::restart() {
    LOG_INFO(F("Restarting system..."));
    Shell::loop_all();
    delay(1000); // wait a second
#if defined(ESP8266)
    ESP.reset();
#elif defined(ESP32)
    ESP.restart();
#endif
}

// saves all settings
void System::wifi_reconnect() {
    LOG_INFO(F("The wifi will reconnect..."));
    Shell::loop_all();
    delay(1000);                                                                // wait a second
    EMSESP::webSettingsService.save();                                          // local settings
    EMSESP::esp8266React.getWiFiSettingsService()->callUpdateHandlers("local"); // in case we've changed ssid or password
}

// format fs
// format the FS. Wipes everything.
void System::format(uuid::console::Shell & shell) {
    auto msg = F("Formatting file system. This will reset all settings to their defaults");
    shell.logger().warning(msg);
    shell.flush();

    EMSuart::stop();

#if defined(ESP8266)
    LittleFS.format();
#elif defined(ESP32)
    SPIFFS.format();
#endif

    System::restart();
}

// return free heap mem as a percentage
uint8_t System::free_mem() {
#ifndef EMSESP_STANDALONE
    uint32_t free_memory = ESP.getFreeHeap();
#else
    uint32_t free_memory = 1000;
#endif

    return (100 * free_memory / heap_start_);
}

void System::syslog_init() {
    int8_t   syslog_level_;
    uint32_t syslog_mark_interval_;
    String   syslog_host_;
    uint16_t syslog_port_;

    // fetch settings
    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        syslog_enabled_       = settings.syslog_enabled;
        syslog_level_         = settings.syslog_level;
        syslog_mark_interval_ = settings.syslog_mark_interval;
        syslog_host_          = settings.syslog_host;
        syslog_port_          = settings.syslog_port;
    });

#ifndef EMSESP_STANDALONE

    // check for empty hostname
    IPAddress addr;
    if (!addr.fromString(syslog_host_.c_str())) {
        syslog_enabled_ = false;
    }

    // in case service is still running, this flushes the queue - https://github.com/emsesp/EMS-ESP/issues/496
    if (!syslog_enabled_) {
        syslog_.log_level((uuid::log::Level)-1);
        syslog_.mark_interval(0);
        syslog_.destination((IPAddress)((uint32_t)0));
        return;
    }

    // start & configure syslog
    syslog_.start();
    syslog_.log_level((uuid::log::Level)syslog_level_);
    syslog_.mark_interval(syslog_mark_interval_);
    syslog_.destination(addr, syslog_port_);
    EMSESP::esp8266React.getWiFiSettingsService()->read([&](WiFiSettings & wifiSettings) { syslog_.hostname(wifiSettings.hostname.c_str()); });

    EMSESP::logger().info(F("Syslog started"));
#endif
}

// first call. Sets memory and starts up the UART Serial bridge
void System::start() {
    // set the inital free mem
    if (heap_start_ == 0) {
#ifndef EMSESP_STANDALONE
        heap_start_ = ESP.getFreeHeap();
#else
        heap_start_ = 2000;
#endif
    }

    // print boot message
    EMSESP::esp8266React.getWiFiSettingsService()->read(
        [&](WiFiSettings & wifiSettings) { LOG_INFO(F("System %s booted (EMS-ESP version %s)"), wifiSettings.hostname.c_str(), EMSESP_APP_VERSION); });

    // these commands respond to the topic "system" and take a payload like {cmd:"", data:"", id:""}
    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        Command::add(EMSdevice::DeviceType::SYSTEM, settings.ems_bus_id, F_(pin), System::command_pin);
        Command::add(EMSdevice::DeviceType::SYSTEM, settings.ems_bus_id, F_(send), System::command_send);
        Command::add(EMSdevice::DeviceType::SYSTEM, settings.ems_bus_id, F_(publish), System::command_publish);
        Command::add(EMSdevice::DeviceType::SYSTEM, settings.ems_bus_id, F_(fetch), System::command_fetch);
        Command::add_with_json(EMSdevice::DeviceType::SYSTEM, F_(info), System::command_info);
        Command::add_with_json(EMSdevice::DeviceType::SYSTEM, F_(settings), System::command_settings);

#if defined(EMSESP_TEST)
        Command::add(EMSdevice::DeviceType::SYSTEM, settings.ems_bus_id, F_(test), System::command_test);
#endif
    });

    init();
}

void System::other_init() {
    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        analog_enabled_ = settings.analog_enabled;
    });
#ifdef ESP32
    // Wifi power settings 2 - 19.5dBm, raw values 4/dBm (8-78)
    // WiFi.setTxPower(WIFI_POWER_13dBm);
    btStop();
    esp_bt_controller_disable();
    if (analog_enabled_) {
        adc_power_on();
    } else {
        adc_power_off();
    }
#endif
}

// button indefinite press
void System::button_OnVLongPress(PButton & b) {
    LOG_DEBUG(F("Button pressed - very long press"));
    LOG_WARNING(F("Performing factory reset..."));

#ifndef EMSESP_STANDALONE
    EMSuart::stop();
    EMSESP::esp8266React.factoryReset();
#endif
}

// push button
void System::button_init() {
    if (!myPButton_.init(0, HIGH)) {
        LOG_INFO(F("External multi-functional button not detected"));
    } else {
        LOG_INFO(F("External multi-functional button enabled"));
    }
#if defined(ESP8266)
    pinMode(4, OUTPUT);
    digitalWrite(4, 0); // set D2 to low for easy connecting D2/D3
#endif
    // myPButton_.onClick(BUTTON_Debounce, button_OnClick);
    // myPButton_.onDblClick(BUTTON_DblClickDelay, button_OnDblClick);
    // myPButton_.onLongPress(BUTTON_LongPressDelay, button_OnLongPress);
    myPButton_.onVLongPress(BUTTON_VLongPressDelay, button_OnVLongPress);
}

// init stuff. This is called when settings are changed in the web
void System::init() {
    led_init(); // init LED

    other_init();

    syslog_init(); // init SysLog
    button_init();

    EMSESP::init_tx(); // start UART
}

// set the LED to on or off when in normal operating mode
void System::led_init() {
    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        hide_led_ = settings.hide_led;
        led_gpio_ = settings.led_gpio;
        if (led_gpio_) {
            pinMode(led_gpio_, OUTPUT);                            // 0 means disabled
            digitalWrite(led_gpio_, hide_led_ ? !LED_ON : LED_ON); // LED on, for ever
        }
    });
}

// returns true if OTA is uploading
bool System::upload_status() {
#if defined(EMSESP_STANDALONE)
    return false;
#else
    return upload_status_ || Update.isRunning();
#endif
}

void System::upload_status(bool in_progress) {
    // if we've just started an upload
    if ((!upload_status_) && (in_progress)) {
        EMSuart::stop();
    }
    upload_status_ = in_progress;
}

// checks system health and handles LED flashing wizardry
void System::loop() {
#ifndef EMSESP_STANDALONE

    if (syslog_enabled_) {
        syslog_.loop();
    }
    myPButton_.check(); // check button press

    led_monitor();  // check status and report back using the LED
    system_check(); // check system health
    if (analog_enabled_) {
        measure_analog();
    }

    // send out heartbeat
    uint32_t currentMillis = uuid::get_uptime();
    if (!last_heartbeat_ || (currentMillis - last_heartbeat_ > SYSTEM_HEARTBEAT_INTERVAL)) {
        last_heartbeat_ = currentMillis;
        send_heartbeat();
    }

#if defined(ESP8266)
#if defined(EMSESP_DEBUG)
    static uint32_t last_memcheck_ = 0;
    if (currentMillis - last_memcheck_ > 10000) { // 10 seconds
        last_memcheck_ = currentMillis;
        show_mem("core");
    }
#endif
#endif

#endif
}

void System::show_mem(const char * note) {
#if defined(ESP8266)
#if defined(EMSESP_DEBUG)
    LOG_INFO(F("(%s) Free heap: %d%% (%lu), frag:%u%%"), note, free_mem(), (unsigned long)ESP.getFreeHeap(), ESP.getHeapFragmentation());
#endif
#endif
}

// send periodic MQTT message with system information
void System::send_heartbeat() {
    // don't send heartbeat if WiFi or MQTT is not connected
    if (!Mqtt::connected()) {
        return;
    }

    int8_t rssi = wifi_quality();
    if (rssi == -1) {
        return;
    }

    // StaticJsonDocument<EMSESP_MAX_JSON_SIZE_HA_CONFIG> doc;
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_HA_CONFIG);

    uint8_t ems_status = EMSESP::bus_status();
    if (ems_status == EMSESP::BUS_STATUS_TX_ERRORS) {
        doc[F("status")] = F("txerror");
    } else if (ems_status == EMSESP::BUS_STATUS_CONNECTED) {
        doc[F("status")] = F("connected");
    } else {
        doc[F("status")] = F("disconnected");
    }

    doc[F("rssi")]         = rssi;
    doc[F("uptime")]       = uuid::log::format_timestamp_ms(uuid::get_uptime_ms(), 3).substr(0, 12);
    doc[F("uptime_sec")]   = uuid::get_uptime_sec();
    doc[F("mqtt_fails")]   = Mqtt::publish_fails();
    doc[F("rx_received")]  = EMSESP::rxservice_.telegram_count();
    doc[F("tx_sent")]      = EMSESP::txservice_.telegram_read_count() + EMSESP::txservice_.telegram_write_count();
    doc[F("tx_fails")]     = EMSESP::txservice_.telegram_fail_count();
    doc[F("rx_fails")]     = EMSESP::rxservice_.telegram_error_count();
    doc[F("dallas_fails")] = EMSESP::sensor_fails();
    doc[F("freemem")]      = ESP.getFreeHeap();
#if defined(ESP8266)
    doc[F("max_alloc_heap")] = ESP.getMaxFreeBlockSize();
    doc[F("fragmem")]        = ESP.getHeapFragmentation();
#elif defined(ESP32)
    doc[F("max_alloc_heap")] = ESP.getMaxAllocHeap())
#endif
    if (analog_enabled_) {
        doc[F("adc")] = analog_;
#if defined(ESP8266)
        doc[F("io2")]  = digitalRead(2);
        doc[F("io4")]  = digitalRead(4);
        doc[F("io5")]  = digitalRead(5);
        doc[F("io12")] = digitalRead(12);
        doc[F("io14")] = digitalRead(14);
        doc[F("io16")] = digitalRead(16);
#elif defined(ESP32)
        doc[F("io16")]  = digitalRead(16);
        doc[F("io17")]  = digitalRead(17);
        doc[F("io18")]  = digitalRead(18);
        doc[F("io19")]  = digitalRead(19);
        doc[F("io21")]  = digitalRead(21);
        doc[F("io22")]  = digitalRead(22);
        doc[F("io26")]  = digitalRead(26);
#endif
    }
    doc.shrinkToFit();
    Mqtt::publish(F_(heartbeat), doc.as<JsonObject>()); // send to MQTT with retain off. This will add to MQTT queue.
}

// measure and moving average adc
void System::measure_analog() {
    static uint32_t measure_last_ = 0;

    if (!measure_last_ || (uint32_t)(uuid::get_uptime() - measure_last_) >= SYSTEM_MEASURE_ANALOG_INTERVAL) {
        measure_last_ = uuid::get_uptime();
#if defined(ESP8266)
        // uint16_t a = analogRead(A0); // 10 bit 3,2V
        uint16_t a = ((analogRead(A0) * 27) / 8); // scale to esp32 result in mV
#elif defined(ESP32)
        uint16_t a = analogRead(36); // arduino scale mV
#else
        uint16_t a = 0; // standalone
#endif
        static uint32_t sum_ = 0;

        if (!analog_) { // init first time
            analog_ = a;
            sum_    = a * 512;
        } else { // simple moving average filter
            sum_    = (sum_ * 511) / 512 + a;
            analog_ = sum_ / 512;
        }
    }
}

// sets rate of led flash
void System::set_led_speed(uint32_t speed) {
    led_flash_speed_ = speed;
    led_monitor();
}

// check health of system, done every few seconds
void System::system_check() {
    static uint32_t last_system_check_ = 0;

    if (!last_system_check_ || ((uint32_t)(uuid::get_uptime() - last_system_check_) >= SYSTEM_CHECK_FREQUENCY)) {
        last_system_check_ = uuid::get_uptime();

#ifndef EMSESP_STANDALONE
        if (WiFi.status() != WL_CONNECTED) {
            set_led_speed(LED_WARNING_BLINK_FAST);
            system_healthy_ = false;
            return;
        }
#endif

        // not healthy if bus not connected
        if (!EMSbus::bus_connected()) {
            if (system_healthy_) {
                LOG_ERROR(F("Error: No connection to the EMS bus"));
            }
            system_healthy_ = false;
            set_led_speed(LED_WARNING_BLINK); // flash every 1/2 second from now on
        } else {
            // if it was unhealthy but now we're better, make sure the LED is solid again cos we've been healed
            if (!system_healthy_) {
                system_healthy_ = true;
                if (led_gpio_) {
                    digitalWrite(led_gpio_, hide_led_ ? !LED_ON : LED_ON); // LED on, for ever
                }
            }
        }
    }
}

// flashes the LED
void System::led_monitor() {
    if (!led_gpio_) {
        return;
    }

    static uint32_t led_last_blink_ = 0;

    if (!led_last_blink_ || (uint32_t)(uuid::get_uptime() - led_last_blink_) >= led_flash_speed_) {
        led_last_blink_ = uuid::get_uptime();

        // if bus_not_connected or network not connected, start flashing
        if (!system_healthy_) {
            digitalWrite(led_gpio_, !digitalRead(led_gpio_));
        }
    }
}

// Return the quality (Received Signal Strength Indicator) of the WiFi network as a %. Or -1 if disconnected.
//  High quality: 90% ~= -55dBm
//  Medium quality: 50% ~= -75dBm
//  Low quality: 30% ~= -85dBm
//  Unusable quality: 8% ~= -96dBm
int8_t System::wifi_quality() {
#ifdef EMSESP_STANDALONE
    return 100;
#else
    if (WiFi.status() != WL_CONNECTED) {
        return -1;
    }
    int32_t dBm = WiFi.RSSI();
    if (dBm <= -100) {
        return 0;
    }

    if (dBm >= -50) {
        return 100;
    }
    return 2 * (dBm + 100);
#endif
}

// print users to console
void System::show_users(uuid::console::Shell & shell) {
    shell.printfln(F("Users:"));

#ifndef EMSESP_STANDALONE
    EMSESP::esp8266React.getSecuritySettingsService()->read([&](SecuritySettings & securitySettings) {
        for (User user : securitySettings.users) {
            shell.printfln(F(" username: %s, password: %s, is_admin: %s"), user.username.c_str(), user.password.c_str(), user.admin ? F("yes") : F("no"));
        }
    });
#endif

    shell.println();
}

void System::show_system(uuid::console::Shell & shell) {
    shell.printfln(F("Uptime:        %s"), uuid::log::format_timestamp_ms(uuid::get_uptime_ms(), 3).substr(0, 12).c_str());

#ifndef EMSESP_STANDALONE
#if defined(ESP8266)
    shell.printfln(F("Chip ID:       0x%08x"), ESP.getChipId());
    shell.printfln(F("SDK version:   %s"), ESP.getSdkVersion());
    shell.printfln(F("Core version:  %s"), ESP.getCoreVersion().c_str());
    shell.printfln(F("Full version:  %s"), ESP.getFullVersion().c_str());
    shell.printfln(F("Boot version:  %u"), ESP.getBootVersion());
    shell.printfln(F("Boot mode:     %u"), ESP.getBootMode());
    shell.printfln(F("CPU frequency: %u MHz"), ESP.getCpuFreqMHz());
    shell.printfln(F("Flash chip:    0x%08X (%u bytes)"), ESP.getFlashChipId(), ESP.getFlashChipRealSize());
    shell.printfln(F("Reset reason:  %s"), ESP.getResetReason().c_str());
    shell.printfln(F("Reset info:    %s"), ESP.getResetInfo().c_str());
#elif defined(ESP32)
    shell.printfln(F("SDK version:   %s"), ESP.getSdkVersion());
    shell.printfln(F("CPU frequency: %u MHz"), ESP.getCpuFreqMHz());
#endif
    shell.printfln(F("Sketch size:   %u bytes (%u bytes free)"), ESP.getSketchSize(), ESP.getFreeSketchSpace());
    shell.printfln(F("Free heap:                %lu bytes"), (unsigned long)ESP.getFreeHeap());
    shell.printfln(F("Free mem:                 %d  %%"), free_mem());
#if defined(ESP8266)
    shell.printfln(F("Heap fragmentation:       %u %%"), ESP.getHeapFragmentation());
    shell.printfln(F("Maximum free block size:  %lu bytes"), (unsigned long)ESP.getMaxFreeBlockSize());
    shell.printfln(F("Free continuations stack: %lu bytes"), (unsigned long)ESP.getFreeContStack());
#endif
    shell.println();

    switch (WiFi.status()) {
    case WL_IDLE_STATUS:
        shell.printfln(F("WiFi: Idle"));
        break;

    case WL_NO_SSID_AVAIL:
        shell.printfln(F("WiFi: Network not found"));
        break;

    case WL_SCAN_COMPLETED:
        shell.printfln(F("WiFi: Network scan complete"));
        break;

    case WL_CONNECTED: {
        shell.printfln(F("WiFi: Connected"));
        shell.printfln(F("SSID: %s"), WiFi.SSID().c_str());
        shell.printfln(F("BSSID: %s"), WiFi.BSSIDstr().c_str());
        shell.printfln(F("RSSI: %d dBm (%d %%)"), WiFi.RSSI(), wifi_quality());
        shell.printfln(F("MAC address: %s"), WiFi.macAddress().c_str());

#if defined(ESP8266)
        shell.printfln(F("Hostname: %s"), WiFi.hostname().c_str());
#elif defined(ESP32)
        shell.printfln(F("Hostname: %s"), WiFi.getHostname());
#endif
        shell.printfln(F("IPv4 address: %s/%s"), uuid::printable_to_string(WiFi.localIP()).c_str(), uuid::printable_to_string(WiFi.subnetMask()).c_str());
        shell.printfln(F("IPv4 gateway: %s"), uuid::printable_to_string(WiFi.gatewayIP()).c_str());
        shell.printfln(F("IPv4 nameserver: %s"), uuid::printable_to_string(WiFi.dnsIP()).c_str());
    } break;

    case WL_CONNECT_FAILED:
        shell.printfln(F("WiFi: Connection failed"));
        break;

    case WL_CONNECTION_LOST:
        shell.printfln(F("WiFi: Connection lost"));
        break;

    case WL_DISCONNECTED:
        shell.printfln(F("WiFi: Disconnected"));
        break;

    case WL_NO_SHIELD:
    default:
        shell.printfln(F("WiFi: Unknown"));
        break;
    }

    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        shell.println();

        if (!settings.syslog_enabled) {
            shell.printfln(F("Syslog: disabled"));
        } else {
            shell.printfln(F("Syslog:"));
            shell.print(F_(1space));
            shell.printfln(F_(host_fmt), !settings.syslog_host.isEmpty() ? settings.syslog_host.c_str() : uuid::read_flash_string(F_(unset)).c_str());
            shell.print(F_(1space));
            shell.printfln(F_(log_level_fmt), uuid::log::format_level_lowercase(static_cast<uuid::log::Level>(settings.syslog_level)));
            shell.print(F_(1space));
            shell.printfln(F_(mark_interval_fmt), settings.syslog_mark_interval);
        }
    });

#endif
}

/*/ console commands to add
void System::console_commands(Shell & shell, unsigned int context) {
    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::ADMIN,
                                       flash_string_vector{F_(restart)},
                                       [](Shell & shell __attribute__((unused)), const std::vector<std::string> & arguments __attribute__((unused))) {
                                           restart();
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::ADMIN,
                                       flash_string_vector{F_(wifi), F_(reconnect)},
                                       [](Shell & shell __attribute__((unused)), const std::vector<std::string> & arguments __attribute__((unused))) {
                                           wifi_reconnect();
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::ADMIN,
                                       flash_string_vector{F_(format)},
                                       [](Shell & shell, const std::vector<std::string> & arguments __attribute__((unused))) {
                                           shell.enter_password(F_(password_prompt), [=](Shell & shell, bool completed, const std::string & password) {
                                               if (completed) {
                                                   EMSESP::esp8266React.getSecuritySettingsService()->read([&](SecuritySettings & securitySettings) {
                                                       if (securitySettings.jwtSecret.equals(password.c_str())) {
                                                           format(shell);
                                                       } else {
                                                           shell.println(F("incorrect password"));
                                                       }
                                                   });
                                               }
                                           });
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::ADMIN,
                                       flash_string_vector{F_(passwd)},
                                       [](Shell & shell, const std::vector<std::string> & arguments __attribute__((unused))) {
                                           shell.enter_password(F_(new_password_prompt1), [](Shell & shell, bool completed, const std::string & password1) {
                                               if (completed) {
                                                   shell.enter_password(F_(new_password_prompt2),
                                                                        [password1](Shell & shell, bool completed, const std::string & password2) {
                                                                            if (completed) {
                                                                                if (password1 == password2) {
                                                                                    EMSESP::esp8266React.getSecuritySettingsService()->update(
                                                                                        [&](SecuritySettings & securitySettings) {
                                                                                            securitySettings.jwtSecret = password2.c_str();
                                                                                            return StateUpdateResult::CHANGED;
                                                                                        },
                                                                                        "local");
                                                                                    shell.println(F("su password updated"));
                                                                                } else {
                                                                                    shell.println(F("Passwords do not match"));
                                                                                }
                                                                            }
                                                                        });
                                               }
                                           });
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::USER,
                                       flash_string_vector{F_(show)},
                                       [=](Shell & shell, const std::vector<std::string> & arguments __attribute__((unused))) {
                                           show_system(shell);
                                           shell.println();
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::ADMIN,
                                       flash_string_vector{F_(set), F_(wifi), F_(hostname)},
                                       flash_string_vector{F_(name_mandatory)},
                                       [](Shell & shell __attribute__((unused)), const std::vector<std::string> & arguments) {
                                           shell.println("The wifi connection will be reset...");
                                           Shell::loop_all();
                                           delay(1000); // wait a second
                                           EMSESP::esp8266React.getWiFiSettingsService()->update(
                                               [&](WiFiSettings & wifiSettings) {
                                                   wifiSettings.hostname = arguments.front().c_str();
                                                   return StateUpdateResult::CHANGED;
                                               },
                                               "local");
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::ADMIN,
                                       flash_string_vector{F_(set), F_(wifi), F_(ssid)},
                                       flash_string_vector{F_(name_mandatory)},
                                       [](Shell & shell, const std::vector<std::string> & arguments) {
                                           EMSESP::esp8266React.getWiFiSettingsService()->updateWithoutPropagation([&](WiFiSettings & wifiSettings) {
                                               wifiSettings.ssid = arguments.front().c_str();
                                               return StateUpdateResult::CHANGED;
                                           });
                                           shell.println("Use `wifi reconnect` to apply the new settings");
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::ADMIN,
                                       flash_string_vector{F_(set), F_(wifi), F_(password)},
                                       [](Shell & shell, const std::vector<std::string> & arguments __attribute__((unused))) {
                                           shell.enter_password(F_(new_password_prompt1), [](Shell & shell, bool completed, const std::string & password1) {
                                               if (completed) {
                                                   shell.enter_password(F_(new_password_prompt2),
                                                                        [password1](Shell & shell, bool completed, const std::string & password2) {
                                                                            if (completed) {
                                                                                if (password1 == password2) {
                                                                                    EMSESP::esp8266React.getWiFiSettingsService()->updateWithoutPropagation(
                                                                                        [&](WiFiSettings & wifiSettings) {
                                                                                            wifiSettings.password = password2.c_str();
                                                                                            return StateUpdateResult::CHANGED;
                                                                                        });
                                                                                    shell.println("Use `wifi reconnect` to apply the new settings");
                                                                                } else {
                                                                                    shell.println(F("Passwords do not match"));
                                                                                }
                                                                            }
                                                                        });
                                               }
                                           });
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::USER,
                                       flash_string_vector{F_(set)},
                                       [](Shell & shell, const std::vector<std::string> & arguments __attribute__((unused))) {
                                           EMSESP::esp8266React.getWiFiSettingsService()->read([&](WiFiSettings & wifiSettings) {
                                               shell.print(F_(1space));
                                               shell.printfln(F_(hostname_fmt),
                                                              wifiSettings.hostname.isEmpty() ? uuid::read_flash_string(F_(unset)).c_str()
                                                                                              : wifiSettings.hostname.c_str());
                                           });

                                           EMSESP::esp8266React.getWiFiSettingsService()->read([&](WiFiSettings & wifiSettings) {
                                               shell.print(F_(1space));
                                               shell.printfln(F_(wifi_ssid_fmt),
                                                              wifiSettings.ssid.isEmpty() ? uuid::read_flash_string(F_(unset)).c_str()
                                                                                          : wifiSettings.ssid.c_str());
                                               shell.print(F_(1space));
                                               shell.printfln(F_(wifi_password_fmt), wifiSettings.ssid.isEmpty() ? F_(unset) : F_(asterisks));
                                           });
                                       });

    EMSESPShell::commands->add_command(ShellContext::SYSTEM,
                                       CommandFlags::ADMIN,
                                       flash_string_vector{F_(show), F_(users)},
                                       [](Shell & shell, const std::vector<std::string> & arguments __attribute__((unused))) { System::show_users(shell); });

    // enter the context
    Console::enter_custom_context(shell, context);
}
*/
// upgrade from previous versions of EMS-ESP, based on SPIFFS on an ESP8266
// returns true if an upgrade was done
// the logic is bit abnormal (loading both filesystems and testing) but this was the only way I could get it to work reliably
/*
bool System::check_upgrade() {
#if defined(ESP8266)
    LittleFSConfig l_cfg;
    l_cfg.setAutoFormat(false);
    LittleFS.setConfig(l_cfg); // do not auto format if it can't find LittleFS
    if (LittleFS.begin()) {
#if defined(EMSESP_FORCE_SERIAL)
        Serial.begin(115200);
        Serial.println(F("FS is Littlefs"));
        Serial.end();
#endif
        return false;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

    SPIFFSConfig cfg;
    cfg.setAutoFormat(false); // prevent formatting when opening SPIFFS filesystem
    SPIFFS.setConfig(cfg);
    if (!SPIFFS.begin()) {
#if defined(EMSESP_FORCE_SERIAL)
        Serial.begin(115200);
        Serial.println(F("No old SPIFFS found!"));
        Serial.end();
#endif
        // if there is neither SPIFFS or LittleFS we can assume the ESP8266 has been erased
        l_cfg.setAutoFormat(true); // reset to normal behaviour
        LittleFS.setConfig(l_cfg);
        return false;
    }

    Serial.begin(115200);

    bool                                           failed = false;
    File                                           file;
    JsonObject                                     network, general, mqtt, custom_settings;
    StaticJsonDocument<EMSESP_MAX_JSON_SIZE_LARGE> doc;

    // open the system settings:
    // {
    // "command":"configfile",
    // "network":{"ssid":"xxxx","password":"yyyy","wmode":1,"staticip":null,"gatewayip":null,"nmask":null,"dnsip":null},
    // "general":{"password":"admin","serial":false,"hostname":"ems-esp","log_events":false,"log_ip":null,"version":"1.9.5"},
    // "mqtt":{"enabled":false,"heartbeat":false,"ip":null,"user":null,"port":1883,"qos":0,"keepalive":60,"retain":false,"password":null,"base":null,"nestedjson":false},
    // "ntp":{"server":"pool.ntp.org","interval":720,"enabled":false,"timezone":2}
    // }
    file = SPIFFS.open("/myesp.json", "r");
    if (!file) {
        Serial.println(F("Unable to read the system config file"));
        failed = true;
    } else {
        DeserializationError error = deserializeJson(doc, file);
        if (error) {
            Serial.printf(PSTR("Error. Failed to deserialize system json, error %s\n"), error.c_str());
            failed = true;
        } else {
            Serial.println(F("Migrating settings from EMS-ESP v1.9..."));
#if defined(EMSESP_DEBUG)
            serializeJson(doc, Serial);
            Serial.println();
#endif
            network = doc[F("network")];
            general = doc[F("general")];
            mqtt    = doc[F("mqtt")];

            // start up LittleFS. If it doesn't exist it will format it
            l_cfg.setAutoFormat(true);
            LittleFS.setConfig(l_cfg);
            LittleFS.begin();
            EMSESP::esp8266React.begin();
            EMSESP::webSettingsService.begin();

            EMSESP::esp8266React.getWiFiSettingsService()->update(
                [&](WiFiSettings & wifiSettings) {
                    wifiSettings.hostname = general[F_(hostname)] | FACTORY_WIFI_HOSTNAME;
                    wifiSettings.ssid     = network[F_(ssid)] | FACTORY_WIFI_SSID;
                    wifiSettings.password = network[F_(password)] | FACTORY_WIFI_PASSWORD;

                    wifiSettings.staticIPConfig = false;
                    JsonUtils::readIP(network, "staticip", wifiSettings.localIP);
                    JsonUtils::readIP(network, "dnsip", wifiSettings.dnsIP1);
                    JsonUtils::readIP(network, "gatewayip", wifiSettings.gatewayIP);
                    JsonUtils::readIP(network, "nmask", wifiSettings.subnetMask);

                    return StateUpdateResult::CHANGED;
                },
                "local");

            EMSESP::esp8266React.getSecuritySettingsService()->update(
                [&](SecuritySettings & securitySettings) {
                    securitySettings.jwtSecret = general[F_(password)] | FACTORY_JWT_SECRET;

                    return StateUpdateResult::CHANGED;
                },
                "local");

            EMSESP::esp8266React.getMqttSettingsService()->update(
                [&](MqttSettings & mqttSettings) {
                    mqttSettings.host          = mqtt[F("ip")] | FACTORY_MQTT_HOST;
                    mqttSettings.mqtt_format   = mqtt[F("mqtt_format")] | Mqtt::Format::NESTED;
                    mqttSettings.bool_format   = mqtt[F("bool_format")] | BOOL_FORMAT_ONOFF;
                    mqttSettings.dallas_format = mqtt[F("dallas_format")] | Mqtt::Dallas_Format::NUMBER;
                    mqttSettings.mqtt_qos      = mqtt[F("qos")] | 0;
                    mqttSettings.mqtt_retain   = mqtt[F("retain")] | false;
                    mqttSettings.username      = mqtt[F("user")] | "";
                    mqttSettings.password      = mqtt[F_(password)] | "";
                    mqttSettings.port          = mqtt[F_(port)] | FACTORY_MQTT_PORT;
                    mqttSettings.clientId      = FACTORY_MQTT_CLIENT_ID;
                    mqttSettings.enabled       = mqtt[F_(enabled)];
                    mqttSettings.keepAlive     = FACTORY_MQTT_KEEP_ALIVE;
                    mqttSettings.cleanSession  = FACTORY_MQTT_CLEAN_SESSION;

                    return StateUpdateResult::CHANGED;
                },
                "local");
        }
    }
    file.close();

    if (failed) {
        Serial.println(F("Failed to read system config. Quitting."));
        SPIFFS.end();
        Serial.end();
        return false;
    }

    // open the custom settings file next:
    // {
    // "command":"custom_configfile",
    // "settings":{"led":true,"led_gpio":2,"dallas_gpio":14,"dallas_parasite":false,"listen_mode":false,"shower_timer":false,"shower_alert":false,"publish_time":0,"tx_mode":1,"bus_id":11,"master_thermostat":0,"known_devices":""}
    // }
    doc.clear();
    failed = false;
    file   = SPIFFS.open("/customconfig.json", "r");
    if (!file) {
        Serial.println(F("Unable to read custom config file"));
        failed = true;
    } else {
        DeserializationError error = deserializeJson(doc, file);
        if (error) {
            Serial.printf(PSTR("Error. Failed to deserialize custom json, error %s\n"), error.c_str());
            failed = true;
        } else {
            custom_settings = doc[F_(settings)];
            EMSESP::webSettingsService.update(
                [&](WebSettings & settings) {
                    settings.tx_mode              = custom_settings[F_(tx_mode)] | EMSESP_DEFAULT_TX_MODE;
                    settings.shower_alert         = custom_settings[F("shower_alert")] | EMSESP_DEFAULT_SHOWER_ALERT;
                    settings.shower_timer         = custom_settings[F("shower_timer")] | EMSESP_DEFAULT_SHOWER_TIMER;
                    settings.master_thermostat    = custom_settings[F("master_thermostat")] | EMSESP_DEFAULT_MASTER_THERMOSTAT;
                    settings.ems_bus_id           = custom_settings[F_(bus_id)] | EMSESP_DEFAULT_EMS_BUS_ID;
                    settings.syslog_enabled       = false;
                    settings.syslog_host          = EMSESP_DEFAULT_SYSLOG_HOST;
                    settings.syslog_level         = EMSESP_DEFAULT_SYSLOG_LEVEL;
                    settings.syslog_mark_interval = EMSESP_DEFAULT_SYSLOG_MARK_INTERVAL;
                    settings.dallas_gpio          = custom_settings[F("dallas_gpio")] | EMSESP_DEFAULT_DALLAS_GPIO;
                    settings.dallas_parasite      = custom_settings[F("dallas_parasite")] | EMSESP_DEFAULT_DALLAS_PARASITE;
                    settings.led_gpio             = custom_settings[F("led_gpio")] | EMSESP_DEFAULT_LED_GPIO;
                    settings.analog_enabled       = EMSESP_DEFAULT_ANALOG_ENABLED;

                    return StateUpdateResult::CHANGED;
                },
                "local");
        }
    }
    file.close();

    SPIFFS.end();

    if (failed) {
        Serial.println(F("Failed to read custom config. Quitting."));
        Serial.end();
        return false;
    }

#pragma GCC diagnostic pop

    Serial.println(F("Restarting..."));
    Serial.flush();
    delay(1000);
    Serial.end();
    delay(1000);
    restart();
    return true;
#else
    return false;
#endif
}
*/

// export all settings to JSON text
// http://ems-esp/api?device=system&cmd=settings
// value and id are ignored
// output with true/false, ignore boolean format
bool System::command_settings(const char * value, const int8_t id, JsonObject & json) {
#ifdef EMSESP_STANDALONE
    json[F("test")] = "testing system info command";
#else
    EMSESP::esp8266React.getWiFiSettingsService()->read([&](WiFiSettings & settings) {
        JsonObject node = json.createNestedObject("WIFI");
        node[F_(ssid)]   = settings.ssid;
        // node[F("password")]         = settings.password;
        node[F_(hostname)] = settings.hostname;
        // Helpers::json_boolean(node, "static_ip_config", settings.staticIPConfig);
        node[F("static_ip_config")] = settings.staticIPConfig;
        JsonUtils::writeIP(node, "local_ip", settings.localIP);
        JsonUtils::writeIP(node, "gateway_ip", settings.gatewayIP);
        JsonUtils::writeIP(node, "subnet_mask", settings.subnetMask);
        JsonUtils::writeIP(node, "dns_ip_1", settings.dnsIP1);
        JsonUtils::writeIP(node, "dns_ip_2", settings.dnsIP2);
    });

    EMSESP::esp8266React.getAPSettingsService()->read([&](APSettings & settings) {
        JsonObject node        = json.createNestedObject("AP");
        node[F("provision_mode")] = settings.provisionMode;
        node[F_(ssid)]            = settings.ssid;
        // node[F("password")]       = settings.password;
        node[F("local_ip")]    = settings.localIP.toString();
        node[F("gateway_ip")]  = settings.gatewayIP.toString();
        node[F("subnet_mask")] = settings.subnetMask.toString();
    });

    EMSESP::esp8266React.getMqttSettingsService()->read([&](MqttSettings & settings) {
        JsonObject node = json.createNestedObject("MQTT");
        // Helpers::json_boolean(node, "enabled", settings.enabled);
        node[F_(enabled)]   = settings.enabled;
        node[F_(host)]      = settings.host;
        node[F_(port)]      = settings.port;
        node[F("username")] = settings.username;
        // node[F("password")]                = settings.password;
        node[F("base")]       = settings.base;
        node[F("client_id")]  = settings.clientId;
        node[F("keep_alive")] = settings.keepAlive;
        // Helpers::json_boolean(node, "clean_session", settings.cleanSession);
        node[F("clean_session")]           = settings.cleanSession;
        node[F("publish_time_boiler")]     = settings.publish_time_boiler;
        node[F("publish_time_thermostat")] = settings.publish_time_thermostat;
        node[F("publish_time_solar")]      = settings.publish_time_solar;
        node[F("publish_time_mixer")]      = settings.publish_time_mixer;
        node[F("publish_time_other")]      = settings.publish_time_other;
        node[F("publish_time_sensor")]     = settings.publish_time_sensor;
        node[F("mqtt_format")]             = settings.mqtt_format;
        node[F("mqtt_qos")]                = settings.mqtt_qos;
        node[F("bool_format")]             = settings.bool_format;
        node[F("dallas_format")]           = settings.dallas_format;
        // Helpers::json_boolean(node, "mqtt_retain", settings.mqtt_retain);
        node[F("mqtt_retain")] = settings.mqtt_retain;
    });

    EMSESP::esp8266React.getNTPSettingsService()->read([&](NTPSettings & settings) {
        JsonObject node = json.createNestedObject("NTP");
        // Helpers::json_boolean(node, "enabled", settings.enabled);
        node[F_(enabled)]    = settings.enabled;
        node[F("server")]    = settings.server;
        node[F("tz_label")]  = settings.tzLabel;
        node[F("tz_format")] = settings.tzFormat;
    });

    EMSESP::esp8266React.getOTASettingsService()->read([&](OTASettings & settings) {
        JsonObject node = json.createNestedObject("OTA");
        // Helpers::json_boolean(node, "enabled", settings.enabled);
        node[F_(enabled)] = settings.enabled;
        node[F_(port)]    = settings.port;
        // node[F_(password)] = settings.password;
    });

    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        JsonObject node    = json.createNestedObject("Settings");
        node[F_(tx_mode)]     = settings.tx_mode;
        node[F("ems_bus_id")] = settings.ems_bus_id;
        // Helpers::json_boolean(node, "syslog_enabled", settings.syslog_enabled);
        node[F("syslog_enabled")]       = settings.syslog_enabled;
        node[F("syslog_level")]         = settings.syslog_level;
        node[F("syslog_mark_interval")] = settings.syslog_mark_interval;
        node[F("syslog_host")]          = settings.syslog_host;
        node[F("master_thermostat")]    = settings.master_thermostat;
        // Helpers::json_boolean(node, "shower_timer", settings.shower_timer);
        // Helpers::json_boolean(node, "shower_alert", settings.shower_alert);
        node[F("shower_timer")] = settings.shower_timer;
        node[F("shower_alert")] = settings.shower_alert;
        node[F("rx_gpio")]      = settings.rx_gpio;
        node[F("tx_gpio")]      = settings.tx_gpio;
        node[F("dallas_gpio")]  = settings.dallas_gpio;
        // Helpers::json_boolean(node, "dallas_parasite", settings.dallas_parasite);
        node[F("dallas_parasite")] = settings.dallas_parasite;
        node[F("led_gpio")]        = settings.led_gpio;
        // Helpers::json_boolean(node, "hide_led", settings.hide_led);
        node[F("hide_led")] = settings.hide_led;
        // Helpers::json_boolean(node, "notoken_api", settings.notoken_api);
        node[F("notoken_api")] = settings.notoken_api;
        // Helpers::json_boolean(node, "analog_enabled", settings.analog_enabled);
        node[F("analog_enabled")] = settings.analog_enabled;
    });

#endif
    return true;
}

// export status information including some basic settings
// http://ems-esp/api?device=system&cmd=info
bool System::command_info(const char * value, const int8_t id, JsonObject & json) {
    JsonObject node;

    node = json.createNestedObject("System");

    node[F_(version)] = EMSESP_APP_VERSION;
#if defined(ESP8266)
    node[F("platform")] = "esp8266";
#elif defined(ESP32)
    node[F("platform")] = "esp32";
#endif
    node[F("uptime")]     = uuid::log::format_timestamp_ms(uuid::get_uptime_ms(), 3);
    node[F("uptime_sec")] = uuid::get_uptime_sec();
    node[F("freemem")]    = free_mem();
#if defined(ESP8266)
    node[F("fragmem")] = ESP.getHeapFragmentation();
#endif

    /* Use call system settings for all settings
    node = json.createNestedObject("Settings");

    EMSESP::esp8266React.getMqttSettingsService()->read([&](MqttSettings & settings) {
        node[F("enabled")]                 = settings.enabled;
        node[F("publish_time_boiler")]     = settings.publish_time_boiler;
        node[F("publish_time_thermostat")] = settings.publish_time_thermostat;
        node[F("publish_time_solar")]      = settings.publish_time_solar;
        node[F("publish_time_mixer")]      = settings.publish_time_mixer;
        node[F("publish_time_other")]      = settings.publish_time_other;
        node[F("publish_time_sensor")]     = settings.publish_time_sensor;
        node[F("mqtt_format")]             = settings.mqtt_format;
        node[F("mqtt_qos")]                = settings.mqtt_qos;
        node[F("mqtt_retain")]             = settings.mqtt_retain;
    });

    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        node[F("tx_mode")]           = settings.tx_mode;
        node[F("ems_bus_id")]        = settings.ems_bus_id;
        node[F("master_thermostat")] = settings.master_thermostat;
        node[F("rx_gpio")]           = settings.rx_gpio;
        node[F("tx_gpio")]           = settings.tx_gpio;
        node[F("dallas_gpio")]       = settings.dallas_gpio;
        node[F("dallas_parasite")]   = settings.dallas_parasite;
        node[F("led_gpio")]          = settings.led_gpio;
        node[F("hide_led")]          = settings.hide_led;
        node[F("notoken_api")]       = settings.notoken_api;
        node[F("analog_enabled")]    = settings.analog_enabled;
    });
*/

    node = json.createNestedObject("Status");

    switch (EMSESP::bus_status()) {
    case EMSESP::BUS_STATUS_OFFLINE:
        node[F("bus")] = (F("disconnected"));
        break;
    case EMSESP::BUS_STATUS_TX_ERRORS:
        node[F("bus")] = (F("connected, instable tx"));
        break;
    case EMSESP::BUS_STATUS_CONNECTED:
    default:
        node[F("bus")] = (F_(connected));
        break;
    }

    if (EMSESP::bus_status() != EMSESP::BUS_STATUS_OFFLINE) {
        node[F("bus_protocol")] = EMSbus::is_ht3() ? F("HT3") : F("Buderus");
        node[F("rx_received")]  = EMSESP::rxservice_.telegram_count();
        node[F("tx_reads")]     = EMSESP::txservice_.telegram_read_count();
        node[F("tx_writes")]    = EMSESP::txservice_.telegram_write_count();
        node[F("rx_fails")]     = EMSESP::rxservice_.telegram_error_count();
        node[F("tx_fails")]     = EMSESP::txservice_.telegram_fail_count();
        node[F("rx_quality")]   = EMSESP::rxservice_.quality();
        node[F("tx_quality")]   = EMSESP::txservice_.quality();
        node[F("mqtt_fails")]   = Mqtt::publish_fails();
        if (EMSESP::sensor_devices().size()) {
            node[F("dallas_sensors")] = EMSESP::sensor_devices().size();
            node[F("dallas_reads")]   = EMSESP::sensor_reads();
            node[F("dallas_fails")]   = EMSESP::sensor_fails();
        }
    }

    JsonArray devices2 = json.createNestedArray("Devices");

    for (const auto & device_class : EMSFactory::device_handlers()) {
        for (const auto & emsdevice : EMSESP::emsdevices) {
            if ((emsdevice) && (emsdevice->device_type() == device_class.first)) {
                JsonObject obj = devices2.createNestedObject();
                obj[F_(type)]  = emsdevice->device_type_name();
                obj[F_(name)]  = emsdevice->to_string();
                char result[200];
                obj[F("handlers")] = emsdevice->show_telegram_handlers(result);
            }
        }
    }
    if (EMSESP::sensor_devices().size()) {
        JsonObject obj = devices2.createNestedObject();
        obj[F_(type)]  = F_(Dallassensor);
        obj[F_(name)]  = F_(Dallassensor);
    }

    return true;
}

#if defined(EMSESP_TEST)
// run a test
// e.g. http://ems-esp/api?device=system&cmd=test&data=boiler
bool System::command_test(const char * value, const int8_t id) {
    return (Test::run_test(value, id));
}
#endif

} // namespace emsesp
