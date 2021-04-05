#include <NTPStatus.h>

using namespace std::placeholders; // for `_1` etc

NTPStatus::NTPStatus(AsyncWebServer* server, SecurityManager* securityManager) {
  server->on(NTP_STATUS_SERVICE_PATH,
             HTTP_GET,
             securityManager->wrapRequest(std::bind(&NTPStatus::ntpStatus, this, _1),
                                          AuthenticationPredicates::IS_AUTHENTICATED));
}

/*
 * Formats the time using the format provided.
 *
 * Uses a 25 byte buffer, large enough to fit an ISO time string with offset.
 */
String formatTime(tm* time, const char* format) {
  char time_string[25];
  strftime(time_string, 25, format, time);
  return String(time_string);
}

String toUTCTimeString(tm* time) {
  return formatTime(time, "%FT%TZ");
}

String toLocalTimeString(tm* time) {
  return formatTime(time, "%FT%T");
}

void NTPStatus::ntpStatus(AsyncWebServerRequest* request) {
  AsyncJsonResponse* response = new AsyncJsonResponse(false, MAX_NTP_STATUS_SIZE);
  JsonObject root = response->getRoot();

  // grab the current instant in unix seconds
  time_t now = time(nullptr);

  // only provide enabled/disabled status for now
  root[F("status")] = sntp_enabled() ? 1 : 0;

  // the current time in UTC
  root[F("utc_time")] = toUTCTimeString(gmtime(&now));

  // local time with offset
  root[F("local_time")] = toLocalTimeString(localtime(&now));

  // the sntp server name
  root[F("server")] = sntp_getservername(0);

  // device uptime in seconds
  root[F("uptime")] = uuid::get_uptime() / 1000;

  response->setLength();
  request->send(response);
}
