#include <FeaturesService.h>

using namespace std::placeholders; // for `_1` etc

FeaturesService::FeaturesService(AsyncWebServer * server) {
    server->on(FEATURES_SERVICE_PATH, HTTP_GET, std::bind(&FeaturesService::features, this, _1));
}

void FeaturesService::features(AsyncWebServerRequest * request) {
    AsyncJsonResponse * response = new AsyncJsonResponse(false, MAX_FEATURES_SIZE);
    JsonObject          root     = response->getRoot();
#if FT_ENABLED(FT_PROJECT)
    root[F("project")] = true;
#else
    root[F("project")] = false;
#endif
#if FT_ENABLED(FT_SECURITY)
    root[F("security")] = true;
#else
    root[F("security")] = false;
#endif
#if FT_ENABLED(FT_MQTT)
    root[F("mqtt")] = true;
#else
    root[F("mqtt")] = false;
#endif
#if FT_ENABLED(FT_NTP)
    root[F("ntp")] = true;
#else
    root[F("ntp")] = false;
#endif
#if FT_ENABLED(FT_OTA)
    root[F("ota")] = true;
#else
    root[F("ota")] = false;
#endif
#if FT_ENABLED(FT_UPLOAD_FIRMWARE)
    root[F("upload_firmware")] = true;
#else
    root[F("upload_firmware")] = false;
#endif
    response->setLength();
    request->send(response);
}
