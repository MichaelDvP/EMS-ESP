# Changelog

### Added

- show some gpio in heartbeat, allow set gpio to input
- publish response to some commands
- publish on demand via `call system publish <device>`
- boiler command `selburnpow`
- simple value info (without enum, long names, ranges)
- new v3 API (without verbose)
- some solar values and settings
- mqtt subscribes for all values selectable
- show dallassensor reads, fails and quality

### Fixed

### Changed

- move more strings to flash, increase free heap
- AsyncMqttClient allow bigger payload (like v3)
- all mqtt topics lower case
- larger dynamic json for mqtt
- settings for bool-format and dallas-format moved to mqtt (like v3)
- model-flags for boiler
- combine some console commands to save memory (show, set)

### Removed

- v1.9 upgrade detection and conversion
