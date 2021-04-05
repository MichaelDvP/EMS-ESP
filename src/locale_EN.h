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

// common words
MAKE_PSTR_WORD(exit)
MAKE_PSTR_WORD(help)
MAKE_PSTR_WORD(log)
MAKE_PSTR_WORD(logout)
MAKE_PSTR_WORD(enabled)
MAKE_PSTR_WORD(disabled)
MAKE_PSTR_WORD(set)
MAKE_PSTR_WORD(show)
MAKE_PSTR_WORD(on)
MAKE_PSTR_WORD(off)
MAKE_PSTR_WORD(ON)
MAKE_PSTR_WORD(OFF)
MAKE_PSTR_WORD(su)
MAKE_PSTR_WORD(name)
MAKE_PSTR_WORD(auto)
MAKE_PSTR_WORD(scan)
MAKE_PSTR_WORD(password)
MAKE_PSTR_WORD(port)
MAKE_PSTR_WORD(read)
MAKE_PSTR_WORD(version)
MAKE_PSTR_WORD(values)
MAKE_PSTR_WORD(system)
MAKE_PSTR_WORD(fetch)
MAKE_PSTR_WORD(restart)
MAKE_PSTR_WORD(format)
MAKE_PSTR_WORD(raw)
MAKE_PSTR_WORD(watch)
MAKE_PSTR_WORD(send)
MAKE_PSTR_WORD(telegram)
MAKE_PSTR_WORD(bus_id)
MAKE_PSTR_WORD(tx_mode)
MAKE_PSTR_WORD(ems)
MAKE_PSTR_WORD(devices)
MAKE_PSTR_WORD(mqtt)
MAKE_PSTR_WORD(emsesp)
MAKE_PSTR_WORD(connected)
MAKE_PSTR_WORD(disconnected)
MAKE_PSTR_WORD(passwd)
MAKE_PSTR_WORD(hostname)
MAKE_PSTR_WORD(host)
MAKE_PSTR_WORD(wifi)
MAKE_PSTR_WORD(reconnect)
MAKE_PSTR_WORD(ssid)
MAKE_PSTR_WORD(heartbeat)
MAKE_PSTR_WORD(users)
MAKE_PSTR_WORD(master)
MAKE_PSTR_WORD(pin)
MAKE_PSTR_WORD(publish)
MAKE_PSTR_WORD(bar)
MAKE_PSTR_WORD(min)
MAKE_PSTR_WORD(hours)
MAKE_PSTR_WORD(uA)
MAKE_PSTR_WORD(timeout)
MAKE_PSTR_WORD(type)


// for commands
MAKE_PSTR_WORD(call)
MAKE_PSTR_WORD(cmd)
MAKE_PSTR_WORD(id)
MAKE_PSTR_WORD(device)
MAKE_PSTR_WORD(data)
MAKE_PSTR_WORD(command)
MAKE_PSTR_WORD(commands)
MAKE_PSTR_WORD(info)
MAKE_PSTR_WORD(test)
MAKE_PSTR_WORD(settings)

// devices
MAKE_PSTR_WORD(boiler)
MAKE_PSTR_WORD(thermostat)
MAKE_PSTR_WORD(switch)
MAKE_PSTR_WORD(solar)
MAKE_PSTR_WORD(mixer)
MAKE_PSTR_WORD(gateway)
MAKE_PSTR_WORD(controller)
MAKE_PSTR_WORD(connect)
MAKE_PSTR_WORD(heatpump)
MAKE_PSTR_WORD(generic)
MAKE_PSTR_WORD(dallassensor)
MAKE_PSTR_WORD(unknown)
MAKE_PSTR_WORD(other)
MAKE_PSTR_WORD(shower)

// terminal
MAKE_PSTR(1space, " ")
MAKE_PSTR(2spaces, "  ")
MAKE_PSTR(kwh, "kWh")
MAKE_PSTR(wh, "Wh")
MAKE_PSTR(EMSESP, "EMS-ESP")
MAKE_PSTR(master_thermostat_fmt, "Master Thermostat Device ID: %s")
MAKE_PSTR(host_fmt, "Host: %s")
MAKE_PSTR(hostname_fmt, "WiFi Hostname: %s")
MAKE_PSTR(mark_interval_fmt, "Mark interval: %lus")
MAKE_PSTR(wifi_ssid_fmt, "WiFi SSID: %s")
MAKE_PSTR(wifi_password_fmt, "WiFi Password: %S")
MAKE_PSTR(mqtt_heartbeat_fmt, "MQTT Heartbeat: %s")
MAKE_PSTR(cmd_optional, "[cmd]")
MAKE_PSTR(ha_optional, "[ha]")
MAKE_PSTR(deep_optional, "[deep]")
MAKE_PSTR(tx_mode_fmt, "Tx mode: %d")
MAKE_PSTR(bus_id_fmt, "Bus ID: %02X")
MAKE_PSTR(watchid_optional, "[ID]")
MAKE_PSTR(watch_format_optional, "[off | on | raw | unknown]")
MAKE_PSTR(invalid_watch, "Invalid watch type")
MAKE_PSTR(data_mandatory, "\"XX XX ...\"")
MAKE_PSTR(percent, "%")
MAKE_PSTR(lpm, "l/min")
MAKE_PSTR(degrees, "°C")
MAKE_PSTR(asterisks, "********")
MAKE_PSTR(n_mandatory, "<n>")
MAKE_PSTR(id_optional, "[id|hc]")
MAKE_PSTR(data_optional, "[data]")
MAKE_PSTR(offset_optional, "[offset]")
MAKE_PSTR(typeid_mandatory, "<type ID>")
MAKE_PSTR(deviceid_mandatory, "<device ID>")
MAKE_PSTR(device_type_optional, "[device]")
MAKE_PSTR(invalid_log_level, "Invalid log level")
MAKE_PSTR(log_level_fmt, "Log level = %s")
MAKE_PSTR(log_level_optional, "[level]")
MAKE_PSTR(name_mandatory, "<name>")
MAKE_PSTR(name_optional, "[name]")
MAKE_PSTR(new_password_prompt1, "Enter new password: ")
MAKE_PSTR(new_password_prompt2, "Retype new password: ")
MAKE_PSTR(password_prompt, "Password: ")
MAKE_PSTR(unset, "<unset>")
MAKE_PSTR_WORD(temperature)

// logger
MAKE_PSTR(hc_not_found, "Heating Circuit %d not found or activated for device ID 0x%02X")
MAKE_PSTR(invalid, "Invalid value")

// Home Assistant icons
MAKE_PSTR(iconwatertemp, "mdi:coolant-temperature")
MAKE_PSTR(iconpercent, "mdi:sine-wave")
MAKE_PSTR(iconfire, "mdi:fire")
MAKE_PSTR(iconfan, "mdi:fan")
MAKE_PSTR(iconflash, "mdi:flash")
MAKE_PSTR(iconwaterpump, "mdi:water-pump")
MAKE_PSTR(iconexport, "mdi:home-export-outline")
MAKE_PSTR(iconimport, "mdi:home-import-outline")
MAKE_PSTR(iconcruise, "mdi:car-cruise-control")
MAKE_PSTR(iconvalve, "mdi:valve")
MAKE_PSTR(iconpower, "mdi:power-cycle")
MAKE_PSTR(iconthermostat, "mdi:home-thermometer-outline")
MAKE_PSTR(iconpump, "mdi:pump")

// HA keys
MAKE_PSTR(hasensor, "homeassistant/sensor/")
MAKE_PSTR(emsespboiler, "ems-esp-boiler")
MAKE_PSTR_WORD(config)
MAKE_PSTR_WORD(stat_t)
MAKE_PSTR_WORD(unit_of_meas)
MAKE_PSTR_WORD(dev_cla)
MAKE_PSTR_WORD(val_tpl)
MAKE_PSTR_WORD(uniq_id)
MAKE_PSTR_WORD(json_attr_t)
MAKE_PSTR_WORD(dev)
MAKE_PSTR_WORD(ids)
MAKE_PSTR_WORD(ic)
MAKE_PSTR_WORD(sw)
MAKE_PSTR_WORD(mf)
MAKE_PSTR_WORD(mdl)

// MQTT topics and suffix
MAKE_PSTR_WORD(thermostat_data)
MAKE_PSTR_WORD(boiler_data)
MAKE_PSTR_WORD(boiler_data_ww)
MAKE_PSTR_WORD(boiler_data_info)
MAKE_PSTR_WORD(system_data)
MAKE_PSTR_WORD(mixer_data)
MAKE_PSTR_WORD(solar_data)
MAKE_PSTR_WORD(switch_data)
MAKE_PSTR_WORD(shower_data)
MAKE_PSTR_WORD(heatpump_data)
MAKE_PSTR_WORD(dallassensor_data)
MAKE_PSTR_WORD(response)
MAKE_PSTR_WORD(shower_alert)
MAKE_PSTR_WORD(shower_timer)

MAKE_PSTR(mqtt_suffix_ww, "_ww")
MAKE_PSTR(mqtt_suffix_info, "_info")

// values for publish
MAKE_PSTR_WORD(hot)
MAKE_PSTR_WORD(intelligent)
MAKE_PSTR_WORD(flow)
MAKE_PSTR_WORD(buffered_flow)
MAKE_PSTR_WORD(buffer)
MAKE_PSTR_WORD(layered_buffer)
MAKE_PSTR(3wayvalve, "3-way valve")
MAKE_PSTR(chargepump, "charge pump")
MAKE_PSTR_WORD(continuous)
MAKE_PSTR_WORD(time)
MAKE_PSTR_WORD(date)
MAKE_PSTR_WORD(own)
MAKE_PSTR_WORD(low)
MAKE_PSTR_WORD(high)
MAKE_PSTR_WORD(start)
MAKE_PSTR_WORD(heat)
MAKE_PSTR_WORD(hold)
MAKE_PSTR_WORD(cool)
MAKE_PSTR_WORD(end)
MAKE_PSTR_WORD(light)
MAKE_PSTR_WORD(medium)
MAKE_PSTR_WORD(heavy)
MAKE_PSTR_WORD(summer)
MAKE_PSTR_WORD(winter)
MAKE_PSTR_WORD(holiday)
MAKE_PSTR_WORD(reduce)
MAKE_PSTR_WORD(room)
MAKE_PSTR_WORD(outdoor)
MAKE_PSTR_WORD(nofrost)
MAKE_PSTR_WORD(comfort)
MAKE_PSTR_WORD(manual)
MAKE_PSTR_WORD(day)
MAKE_PSTR_WORD(night)
MAKE_PSTR_WORD(eco)
MAKE_PSTR_WORD(offset)
MAKE_PSTR_WORD(design)
MAKE_PSTR_WORD(minflow)
MAKE_PSTR_WORD(maxflow)
MAKE_PSTR_WORD(rc20)
MAKE_PSTR_WORD(rc3x)
MAKE_PSTR_WORD(radiator)
MAKE_PSTR_WORD(convector)
MAKE_PSTR_WORD(floor)
MAKE_PSTR_WORD(simple)
MAKE_PSTR(const, "const.")
MAKE_PSTR_WORD(power)
MAKE_PSTR_WORD(mpc)
MAKE_PSTR_WORD(german)
MAKE_PSTR_WORD(french)
MAKE_PSTR_WORD(dutch)
MAKE_PSTR_WORD(italian)
MAKE_PSTR_WORD(error)

// some extra mqtt commands

MAKE_PSTR_WORD(pause)
MAKE_PSTR_WORD(party)
MAKE_PSTR_WORD(remotetemp)
MAKE_PSTR_WORD(temp)
MAKE_PSTR_WORD(roomtemp)
MAKE_PSTR_WORD(calinttemp)
MAKE_PSTR_WORD(wwtapactivated)
MAKE_PSTR_WORD(reset)
MAKE_PSTR_WORD(switchtime)

// renamed
// MAKE_PSTR_WORD(flowtemp)     // -> setflowtemp
// MAKE_PSTR_WORD(wcirculation) // -> wwcirc
// MAKE_PSTR_WORD(burnperiod)   // -> burnminperiod

// mqtt topics
// to change use MAKE_PSTR(topic, "new topic")

// boiler
MAKE_PSTR_WORD(heatingactive)
MAKE_PSTR_WORD(tapwateractive)
MAKE_PSTR_WORD(servicecode)
MAKE_PSTR_WORD(servicecodenumber)
MAKE_PSTR_WORD(lastcode)
MAKE_PSTR_WORD(wwseltemp)
MAKE_PSTR_WORD(wwsettemp)
MAKE_PSTR_WORD(wwdisinfectiontemp)
MAKE_PSTR_WORD(selflowtemp)
MAKE_PSTR_WORD(selburnpow)
MAKE_PSTR_WORD(curburnpow)
MAKE_PSTR_WORD(heatingpumpmod)
MAKE_PSTR_WORD(heatingpump2mod)
MAKE_PSTR_WORD(wwtype)
MAKE_PSTR_WORD(wwchargetype)
MAKE_PSTR_WORD(wwcircpump)
MAKE_PSTR_WORD(wwcircpumpmode)
MAKE_PSTR_WORD(wwcirc)
MAKE_PSTR_WORD(outdoortemp)
MAKE_PSTR_WORD(wwcurtemp)
MAKE_PSTR_WORD(wwcurtemp2)
MAKE_PSTR_WORD(wwcurflow)
MAKE_PSTR_WORD(curflowtemp)
MAKE_PSTR_WORD(rettemp)
MAKE_PSTR_WORD(switchtemp)
MAKE_PSTR_WORD(syspress)
MAKE_PSTR_WORD(boiltemp)
MAKE_PSTR_WORD(wwstoragetemp1)
MAKE_PSTR_WORD(wwstoragetemp2)
MAKE_PSTR_WORD(exhausttemp)
MAKE_PSTR_WORD(wwactivated)
MAKE_PSTR_WORD(wwonetime)
MAKE_PSTR_WORD(wwdisinfecting)
MAKE_PSTR_WORD(wwcharging)
MAKE_PSTR_WORD(wwrecharging)
MAKE_PSTR_WORD(wwtempok)
MAKE_PSTR_WORD(wwactive)
MAKE_PSTR_WORD(burngas)
MAKE_PSTR_WORD(flamecurr)
MAKE_PSTR_WORD(heatingpump)
MAKE_PSTR_WORD(fanwork)
MAKE_PSTR_WORD(ignwork)
MAKE_PSTR_WORD(wwheat)
MAKE_PSTR_WORD(heatingactivated)
MAKE_PSTR_WORD(heatingtemp)
MAKE_PSTR_WORD(pumpmodmax)
MAKE_PSTR_WORD(pumpmodmin)
MAKE_PSTR_WORD(pumpdelay)
MAKE_PSTR_WORD(burnminperiod)
MAKE_PSTR_WORD(burnminpower)
MAKE_PSTR_WORD(burnmaxpower)
MAKE_PSTR_WORD(boilhyston)
MAKE_PSTR_WORD(boilhystoff)
MAKE_PSTR_WORD(setflowtemp)
MAKE_PSTR_WORD(wwsetpumppower)
MAKE_PSTR_WORD(mixertemp)
MAKE_PSTR_WORD(tankmiddletemp)
MAKE_PSTR_WORD(wwstarts)
MAKE_PSTR_WORD(wwworkm)
MAKE_PSTR_WORD(setburnpow)
MAKE_PSTR_WORD(burnstarts)
MAKE_PSTR_WORD(burnworkmin)
MAKE_PSTR_WORD(heatworkmin)
MAKE_PSTR_WORD(ubauptime)
MAKE_PSTR_WORD(wwcomfort)

// boiler info
MAKE_PSTR_WORD(uptimecontrol)
MAKE_PSTR_WORD(uptimecompheating)
MAKE_PSTR_WORD(uptimecompcooling)
MAKE_PSTR_WORD(uptimecompww)
MAKE_PSTR_WORD(heatingstarts)
MAKE_PSTR_WORD(coolingstarts)
MAKE_PSTR_WORD(wwstarts2)
MAKE_PSTR_WORD(nrgconstotal)
MAKE_PSTR_WORD(auxelecheatnrgconstotal)
MAKE_PSTR_WORD(auxelecheatnrgconsheating)
MAKE_PSTR_WORD(auxelecheatnrgconsdhw)
MAKE_PSTR_WORD(nrgconscomptotal)
MAKE_PSTR_WORD(nrgconscompheating)
MAKE_PSTR_WORD(nrgconscompww)
MAKE_PSTR_WORD(nrgconscompcooling)
MAKE_PSTR_WORD(nrgsupptotal)
MAKE_PSTR_WORD(nrgsuppheating)
MAKE_PSTR_WORD(nrgsuppww)
MAKE_PSTR_WORD(nrgsuppcooling)
MAKE_PSTR_WORD(maintenancemessage)
MAKE_PSTR_WORD(maintenance)
MAKE_PSTR_WORD(maintenancetime)
MAKE_PSTR_WORD(maintenancedate)

// solar
MAKE_PSTR_WORD(collectortemp)
MAKE_PSTR_WORD(tankbottomtemp)
// MAKE_PSTR_WORD(tankmiddletemp)
MAKE_PSTR_WORD(tank2bottomtemp)
MAKE_PSTR_WORD(heatexchangertemp)
MAKE_PSTR_WORD(solarpumpmodulation)
MAKE_PSTR_WORD(cylinderpumpmodulation)
MAKE_PSTR_WORD(collectormaxtemp)
MAKE_PSTR_WORD(tankbottommaxtemp)
MAKE_PSTR_WORD(collectormintemp)
MAKE_PSTR_WORD(pumpworktime)
MAKE_PSTR_WORD(pumpworktimetext)
MAKE_PSTR_WORD(energylasthour)
MAKE_PSTR_WORD(energytoday)
MAKE_PSTR_WORD(energytotal)
MAKE_PSTR_WORD(solarpump)
MAKE_PSTR_WORD(valvestatus)
MAKE_PSTR_WORD(tankheated)
MAKE_PSTR_WORD(collectorshutdown)

// mixer
MAKE_PSTR_WORD(ww_hc)
MAKE_PSTR_WORD(wwtemp)
MAKE_PSTR_WORD(tempstatus)
MAKE_PSTR_WORD(hc)
MAKE_PSTR_WORD(tempswitch)
MAKE_PSTR_WORD(pumpstatus)
MAKE_PSTR_WORD(flowtempvf)
MAKE_PSTR_WORD(flowsettemp)
MAKE_PSTR_WORD(flowtemphc)
MAKE_PSTR_WORD(mixvalve)

// thermostat
MAKE_PSTR_WORD(datetime)
MAKE_PSTR_WORD(errorcode)
MAKE_PSTR_WORD(display)
MAKE_PSTR_WORD(language)
MAKE_PSTR_WORD(clockoffset)
MAKE_PSTR_WORD(dampedoutdoortemp)
MAKE_PSTR_WORD(inttemp1)
MAKE_PSTR_WORD(inttemp2)
MAKE_PSTR_WORD(intoffset)
MAKE_PSTR_WORD(minexttemp)
MAKE_PSTR_WORD(building)
MAKE_PSTR_WORD(floordry)
MAKE_PSTR_WORD(floordrytemp)

MAKE_PSTR_WORD(wwmode)
// MAKE_PSTR_WORD(wwsettemp) // same as in boiler
MAKE_PSTR_WORD(wwsettemplow)
MAKE_PSTR_WORD(wwextra1)
MAKE_PSTR_WORD(wwextra2)
MAKE_PSTR_WORD(wwcircmode)

// thermostat - per heating circuit
MAKE_PSTR_WORD(hatemp)
MAKE_PSTR_WORD(seltemp)
MAKE_PSTR_WORD(currtemp)
MAKE_PSTR_WORD(heattemp)
MAKE_PSTR_WORD(comforttemp)
MAKE_PSTR_WORD(daytemp)
MAKE_PSTR_WORD(ecotemp)
MAKE_PSTR_WORD(nighttemp)
MAKE_PSTR_WORD(manualtemp)
MAKE_PSTR_WORD(holidaytemp)
MAKE_PSTR_WORD(nofrosttemp)
MAKE_PSTR_WORD(heatingtype)
MAKE_PSTR_WORD(targetflowtemp)
MAKE_PSTR_WORD(offsettemp)
MAKE_PSTR_WORD(designtemp)
MAKE_PSTR_WORD(summertemp)
MAKE_PSTR_WORD(summermode)
MAKE_PSTR_WORD(roominfluence)
MAKE_PSTR_WORD(flowtempoffset)
MAKE_PSTR_WORD(minflowtemp)
MAKE_PSTR_WORD(maxflowtemp)
MAKE_PSTR_WORD(reducemode)
MAKE_PSTR_WORD(program)
MAKE_PSTR_WORD(controlmode)
MAKE_PSTR_WORD(control)
MAKE_PSTR_WORD(mode)
MAKE_PSTR_WORD(modetype)

// heat pump
MAKE_PSTR_WORD(airhumidity)
MAKE_PSTR_WORD(dewtemperature)

// other
MAKE_PSTR_WORD(activated)
MAKE_PSTR_WORD(status)

// texts EN

// boiler
MAKE_PSTR(heatingactive_, "heating active")
MAKE_PSTR(tapwateractive_, "warm water/DHW active")
MAKE_PSTR(servicecode_, "service code")
MAKE_PSTR(servicecodenumber_, "service code number")
MAKE_PSTR(lastcode_, "last error")
MAKE_PSTR(wwseltemp_, "warm water selected temperature")
MAKE_PSTR(wwsettemp_, "warm water set temperature")
MAKE_PSTR(wwdisinfectiontemp_, "warm water disinfection temperature")
MAKE_PSTR(selflowtemp_, "selected flow temperature")
MAKE_PSTR(selburnpow_, "burner selected power")
MAKE_PSTR(curburnpow_, "burner current power")
MAKE_PSTR(heatingpumpmod_, "heating pump modulation")
MAKE_PSTR(heatingpump2mod_, "heating pump 2 modulation")
MAKE_PSTR(wwtype_, "warm water type")
MAKE_PSTR(wwcomfort_, "warm water comfort mode")
MAKE_PSTR(wwchargetype_, "warm water charging type")
MAKE_PSTR(wwcircpump_, "warm water circulation pump available")
MAKE_PSTR(wwcircpumpmode_, "warm water circulation pump freq")
MAKE_PSTR(wwcirc_, "warm water circulation active")
MAKE_PSTR(outdoortemp_, "outside temperature")
MAKE_PSTR(wwcurtemp_, "warm water current temperature (intern)")
MAKE_PSTR(wwcurtemp2_, "warm water current temperature (extern)")
MAKE_PSTR(wwcurflow_, "warm water current tap water flow")
MAKE_PSTR(curflowtemp_, "current flow temperature")
MAKE_PSTR(rettemp_, "return temperature")
MAKE_PSTR(switchtemp_, "mixer switch temperature")
MAKE_PSTR(syspress_, "system pressure")
MAKE_PSTR(boiltemp_, "max temperature")
MAKE_PSTR(wwstoragetemp1_, "warm water storage temperature (intern)")
MAKE_PSTR(wwstoragetemp2_, "warm water storage temperature (extern)")
MAKE_PSTR(exhausttemp_, "exhaust temperature")
MAKE_PSTR(wwactivated_, "warm water activated")
MAKE_PSTR(wwonetime_, "warm water one time charging")
MAKE_PSTR(wwdisinfecting_, "warm water disinfecting")
MAKE_PSTR(wwcharging_, "warm water charging")
MAKE_PSTR(wwrecharging_, "warm water recharging")
MAKE_PSTR(wwtempok_, "warm water temperature ok")
MAKE_PSTR(wwactive_, "warm water active")
MAKE_PSTR(burngas_, "gas")
MAKE_PSTR(flamecurr_, "flame current")
MAKE_PSTR(heatingpump_, "heating Pump")
MAKE_PSTR(fanwork_, "fan")
MAKE_PSTR(ignwork_, "ignition")
MAKE_PSTR(wwheat_, "warm water heating")
MAKE_PSTR(heatingactivated_, "heating activated")
MAKE_PSTR(heatingtemp_, "heating temperature setting")
MAKE_PSTR(pumpmodmax_, "circuit pump modulation max power")
MAKE_PSTR(pumpmodmin_, "circuit pump modulation min power")
MAKE_PSTR(pumpdelay_, "circuit pump delay time")
MAKE_PSTR(burnminperiod_, "burner min period")
MAKE_PSTR(burnminpower_, "burner min power")
MAKE_PSTR(burnmaxpower_, "burner max power")
MAKE_PSTR(boilhyston_, "temperature hysteresis on")
MAKE_PSTR(boilhystoff_, "temperature hysteresis off")
MAKE_PSTR(setflowtemp_, "set flow temperature")
MAKE_PSTR(wwsetpumppower_, "warm water pump set power")
MAKE_PSTR(mixertemp_, "mixer temperature")
MAKE_PSTR(tankmiddletemp_, "tank middle temperature (TS3)")
MAKE_PSTR(wwstarts_, "warm water starts")
MAKE_PSTR(wwworkm_, "warm water active time")
MAKE_PSTR(setburnpow_, "burner set power")
MAKE_PSTR(burnstarts_, "burner starts")
MAKE_PSTR(burnworkmin_, "burner active time")
MAKE_PSTR(heatworkmin_, "heating active time")
MAKE_PSTR(ubauptime_, "boiler total uptime")

MAKE_PSTR(uptimecontrol_, "operating time control")
MAKE_PSTR(uptimecompheating_, "operating time compressor heating")
MAKE_PSTR(uptimecompcooling_, "operating time compressor cooling")
MAKE_PSTR(uptimecompww_, "operating time compressor warm water")
MAKE_PSTR(heatingstarts_, "heating starts (control)")
MAKE_PSTR(coolingstarts_, "cooling starts (control)")
MAKE_PSTR(wwstarts2_, "warm water starts (control)")
MAKE_PSTR(nrgconstotal_, "energy consumption total")
MAKE_PSTR(auxelecheatnrgconstotal_, "auxiliary electrical heater energy consumption total")
MAKE_PSTR(auxelecheatnrgconsheating_, "auxiliary electrical heater energy consumption heating")
MAKE_PSTR(auxelecheatnrgconsdhw_, "auxiliary electrical heater energy consumption DHW")
MAKE_PSTR(nrgconscomptotal_, "energy consumption compressor total")
MAKE_PSTR(nrgconscompheating_, "energy consumption compressor heating")
MAKE_PSTR(nrgconscompww_, "energy consumption compressor warm water")
MAKE_PSTR(nrgconscompcooling_, "energy consumption compressor total")
MAKE_PSTR(nrgsupptotal_, "energy supplied total")
MAKE_PSTR(nrgsuppheating_, "energy supplied heating")
MAKE_PSTR(nrgsuppww_, "energy supplied warm water")
MAKE_PSTR(nrgsuppcooling_, "energy supplied cooling")
MAKE_PSTR(maintenancemessage_, "maintenance message")
MAKE_PSTR(maintenance_, "scheduled maintenance")
MAKE_PSTR(maintenancetime_, "next maintenance in")
MAKE_PSTR(maintenancedate_, "next maintenance on")

// solar
MAKE_PSTR(collectortemp_, "collector temperature (TS1)")
MAKE_PSTR(tankbottomtemp_, "tank bottom temperature (TS2)")
// MAKE_PSTR(tankmiddletemp_, "tank middle temperature (TS3)")
MAKE_PSTR(tank2bottomtemp_, "second Tank bottom temperature (TS5)")
MAKE_PSTR(heatexchangertemp_, "heat exchanger temperature (TS6)")
MAKE_PSTR(solarpumpmodulation_, "pump modulation (PS1)")
MAKE_PSTR(cylinderpumpmodulation_, "cylinder pump modulation (PS5)")
MAKE_PSTR(collectormaxtemp_, "maximum collector temperature")
MAKE_PSTR(tankbottommaxtemp_, "maximum Bottom Tank temperature")
MAKE_PSTR(collectormintemp_, "minimum collector temperature")
MAKE_PSTR(pumpworktime_, "pump working time (min)")
MAKE_PSTR(pumpworktimetext_, "pump working time")
MAKE_PSTR(energylasthour_, "energy last hour")
MAKE_PSTR(energytoday_, "energy today")
MAKE_PSTR(energytotal_, "energy total")
MAKE_PSTR(solarpump_, "pump (PS1)")
MAKE_PSTR(valvestatus_, "valve status")
MAKE_PSTR(tankheated_, "tank heated")
MAKE_PSTR(collectorshutdown_, "collector shutdown")

// mixer
MAKE_PSTR(ww_hc_, "  warm water circuit %d:")
MAKE_PSTR(hc_, "  heating circuit %d:")

MAKE_PSTR(wwtemp_, "current warm water temperature")
MAKE_PSTR(tempstatus_, "current temperature status")
MAKE_PSTR(tempswitch_, "temperature switch in assigned hc (MC1)")
MAKE_PSTR(pumpstatus_, "pump status in assigned hc (PC1)")
MAKE_PSTR(flowtempvf_, "flow temperature on header (T0/VF)")
MAKE_PSTR(flowsettemp_, "setpoint flow temperature in assigned hc")
MAKE_PSTR(flowtemphc_, "flow temperature in assigned hc (TC1)")
MAKE_PSTR(mixvalve_, "mixing valve actuator in assigned hc (VC1)")


// thermostat
MAKE_PSTR(datetime_, "time")
MAKE_PSTR(errorcode_, "error code")
MAKE_PSTR(display_, "display")
MAKE_PSTR(language_, "language")
MAKE_PSTR(clockoffset_, "clock offset")
MAKE_PSTR(dampedoutdoortemp_, "damped outdoor temperature")
MAKE_PSTR(inttemp1_, "temperature sensor 1")
MAKE_PSTR(inttemp2_, "temperature sensor 2")
MAKE_PSTR(intoffset_, "offset int. temperature")
MAKE_PSTR(minexttemp_, "min ext. temperature")
MAKE_PSTR(building_, "building")
MAKE_PSTR(floordry_, "floordrying")
MAKE_PSTR(floordrytemp_, "floordrying temperature")

MAKE_PSTR(wwmode_, "warm water mode")
// MAKE_PSTR(wwsettemp_, "warm water set temperature")
MAKE_PSTR(wwsettemplow_, "warm water set temperature low")
MAKE_PSTR(wwextra1_, "warm water circuit 1 extra")
MAKE_PSTR(wwextra2_, "warm water circuit 2 extra")
MAKE_PSTR(wwcircmode_, "warm water circulation mode")

// thermostat - per heating circuit
// MAKE_PSTR_LIST(seltemp_, F("seltemp")_, F("setpoint room temperature"))
MAKE_PSTR(seltemp_, "setpoint room temperature")
MAKE_PSTR(currtemp_, "current room temperature")
MAKE_PSTR(heattemp_, "heat temperature")
MAKE_PSTR(comforttemp_, "comfort temperature")
MAKE_PSTR(daytemp_, "day temperature")
MAKE_PSTR(ecotemp_, "eco temperature")
MAKE_PSTR(nighttemp_, "night temperature")
MAKE_PSTR(manualtemp_, "manual temperature")
MAKE_PSTR(holidaytemp_, "holiday temperature")
MAKE_PSTR(nofrosttemp_, "nofrost temperature")
MAKE_PSTR(heatingtype_, "heating type")
MAKE_PSTR(targetflowtemp_, "target flow temperature")
MAKE_PSTR(offsettemp_, "offset temperature")
MAKE_PSTR(designtemp_, "design temperature")
MAKE_PSTR(summertemp_, "summer temperature")
MAKE_PSTR(summermode_, "summer mode")
MAKE_PSTR(roominfluence_, "room influence")
MAKE_PSTR(flowtempoffset_, "flow temperature offset")
MAKE_PSTR(minflowtemp_, "min. flow temperature")
MAKE_PSTR(maxflowtemp_, "max. flow temperature")
MAKE_PSTR(reducemode_, "reduce mode")
MAKE_PSTR(program_, "timer program")
MAKE_PSTR(controlmode_, "control mode")
MAKE_PSTR(control_, "control device")
MAKE_PSTR(mode_, "mode")
MAKE_PSTR(modetype_, "mode type")

// heat pump
MAKE_PSTR(airhumidity_, "relative air humidity")
MAKE_PSTR(dewtemperature_, "dew point temperature")

// other
MAKE_PSTR(activated_, "switch activated")
MAKE_PSTR(status_, "switch status")
