#include <SystemStatus.h>

using namespace std::placeholders; // for `_1` etc

SystemStatus::SystemStatus(AsyncWebServer * server, SecurityManager * securityManager) {
    server->on(SYSTEM_STATUS_SERVICE_PATH,
               HTTP_GET,
               securityManager->wrapRequest(std::bind(&SystemStatus::systemStatus, this, _1), AuthenticationPredicates::IS_AUTHENTICATED));
}

void SystemStatus::systemStatus(AsyncWebServerRequest * request) {
    uint8_t free_mem_percent = emsesp::System::free_mem(); // added by proddy

    AsyncJsonResponse * response = new AsyncJsonResponse(false, MAX_ESP_STATUS_SIZE);
    JsonObject          root     = response->getRoot();
#ifdef ESP32
    root[F("esp_platform")]   = F("esp32");
    root[F("max_alloc_heap")] = ESP.getMaxAllocHeap();
    root[F("psram_size")]     = ESP.getPsramSize();
    root[F("free_psram")]     = ESP.getFreePsram();
#elif defined(ESP8266)
    root[F("esp_platform")]       = F("esp8266");
    root[F("max_alloc_heap")]     = ESP.getMaxFreeBlockSize();
    root[F("heap_fragmentation")] = ESP.getHeapFragmentation();
#endif
    root[F("cpu_freq_mhz")]      = ESP.getCpuFreqMHz();
    root[F("free_heap")]         = ESP.getFreeHeap();
    root[F("sketch_size")]       = ESP.getSketchSize();
    root[F("free_sketch_space")] = ESP.getFreeSketchSpace();
    root[F("sdk_version")]       = ESP.getSdkVersion();
    root[F("flash_chip_size")]   = ESP.getFlashChipSize();
    root[F("flash_chip_speed")]  = ESP.getFlashChipSpeed();

// ESP8266 and ESP32 do not have feature parity in FS.h which currently makes that difficult.
#ifdef ESP32
    root[F("fs_total")] = SPIFFS.totalBytes();
    root[F("fs_used")]  = SPIFFS.usedBytes();
#elif defined(ESP8266)
    FSInfo fs_info;
    LittleFS.info(fs_info); // proddy added
    root[F("fs_total")] = fs_info.totalBytes;
    root[F("fs_used")]  = fs_info.usedBytes;
#endif

    root[F("uptime")]   = uuid::log::format_timestamp_s(uuid::get_uptime_ms(), 3) + " (" + ESP.getResetInfo().c_str() + ")"; // proddy added
    root[F("free_mem")] = free_mem_percent; // proddy added

    response->setLength();
    request->send(response);
}
