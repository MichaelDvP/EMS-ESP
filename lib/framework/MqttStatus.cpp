#include <MqttStatus.h>

using namespace std::placeholders; // for `_1` etc

MqttStatus::MqttStatus(AsyncWebServer * server, MqttSettingsService * mqttSettingsService, SecurityManager * securityManager)
    : _mqttSettingsService(mqttSettingsService) {
    server->on(MQTT_STATUS_SERVICE_PATH,
               HTTP_GET,
               securityManager->wrapRequest(std::bind(&MqttStatus::mqttStatus, this, _1), AuthenticationPredicates::IS_AUTHENTICATED));
}

void MqttStatus::mqttStatus(AsyncWebServerRequest * request) {
    AsyncJsonResponse * response = new AsyncJsonResponse(false, MAX_MQTT_STATUS_SIZE);
    JsonObject          root     = response->getRoot();

    root[F("enabled")]           = _mqttSettingsService->isEnabled();
    root[F("connected")]         = _mqttSettingsService->isConnected();
    root[F("client_id")]         = _mqttSettingsService->getClientId();
    root[F("disconnect_reason")] = (uint8_t)_mqttSettingsService->getDisconnectReason();

    root[F("mqtt_fails")] = emsesp::Mqtt::publish_fails(); // proddy added
    root[F("mqtt_count")] = emsesp::Mqtt::publish_count(); // Michael added
    root[F("mqtt_queue")] = emsesp::Mqtt::publish_queue(); // Michael added

    response->setLength();
    request->send(response);
}
