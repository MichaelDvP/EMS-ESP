# Changelog

### Added

- show some gpio in heartbeat, allow set gpio to input
- publish response to some commands
- publish on demand via `call system publish <device>`
- boiler command `selburnpow`

### Fixed

### Changed

- move mqtt and strings to local_EN
- move more strings to flash, increase free heap
- AsyncMqttClient allow bigger payload (like v3)
- AsyncWebServer with more texts to flash and buffers dependent on free heap (https://github.com/sascha432/ESPAsyncWebServer.git)
- all mqtt topics lower case
- larger dynamic json for mqtt
- settings for bool-format and dallas-format moved to mqtt (like v3)

### Removed
