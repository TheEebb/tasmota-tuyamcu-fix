/*
  xsns_75_prometheus.ino - Web based information for Tasmota

  Copyright (C) 2020  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_PROMETHEUS
/*********************************************************************************************\
 * Prometheus support
\*********************************************************************************************/

#define XSNS_75                    75

void HandleMetrics(void)
{
  if (!HttpCheckPriviledgedAccess()) { return; }

  AddLog_P(LOG_LEVEL_DEBUG, S_LOG_HTTP, PSTR("Prometheus"));

  WSContentBegin(200, CT_PLAIN);


  char parameter[FLOATSZ];

  // Pseudo-metric providing metadata about the running firmware version.
  WSContentSend_P(PSTR("# TYPE tasmota_info gauge\ntasmota_info{version=\"%s\",image=\"%s\",build_timestamp=\"%s\"} 1\n"),
                  my_version, my_image, GetBuildDateAndTime().c_str());
  WSContentSend_P(PSTR("# TYPE tasmota_uptime_seconds gauge\ntasmota_uptime_seconds %d\n"), uptime);
  WSContentSend_P(PSTR("# TYPE tasmota_boot_count counter\ntasmota_boot_count %d\n"), Settings.bootcount);
  WSContentSend_P(PSTR("# TYPE tasmota_flash_writes_total counter\ntasmota_flash_writes_total %d\n"), Settings.save_flag);


  // Pseudo-metric providing metadata about the WiFi station.
  WSContentSend_P(PSTR("# TYPE tasmota_wifi_station_info gauge\ntasmota_wifi_station_info{bssid=\"%s\",ssid=\"%s\"} 1\n"), WiFi.BSSIDstr().c_str(), WiFi.SSID().c_str());

  // Wi-Fi Signal strength
  WSContentSend_P(PSTR("# TYPE tasmota_wifi_station_signal_dbm gauge\ntasmota_wifi_station_signal_dbm{mac_address=\"%s\"} %d\n"), WiFi.BSSIDstr().c_str(), WiFi.RSSI());

  if (!isnan(global_temperature_celsius)) {
    dtostrfd(global_temperature_celsius, Settings.flag2.temperature_resolution, parameter);
    WSContentSend_P(PSTR("# TYPE global_temperature_celsius gauge\nglobal_temperature_celsius %s\n"), parameter);
  }
  if (global_humidity != 0) {
    dtostrfd(global_humidity, Settings.flag2.humidity_resolution, parameter);
    WSContentSend_P(PSTR("# TYPE global_humidity gauge\nglobal_humidity %s\n"), parameter);
  }
  if (global_pressure_hpa != 0) {
    dtostrfd(global_pressure_hpa, Settings.flag2.pressure_resolution, parameter);
    WSContentSend_P(PSTR("# TYPE global_pressure_hpa gauge\nglobal_pressure_hpa %s\n"), parameter);
  }

#ifdef USE_ENERGY_SENSOR
  dtostrfd(Energy.voltage[0], Settings.flag2.voltage_resolution, parameter);
  WSContentSend_P(PSTR("# TYPE energy_voltage_volts gauge\nenergy_voltage_volts %s\n"), parameter);
  dtostrfd(Energy.current[0], Settings.flag2.current_resolution, parameter);
  WSContentSend_P(PSTR("# TYPE energy_current_amperes gauge\nenergy_current_amperes %s\n"), parameter);
  dtostrfd(Energy.active_power[0], Settings.flag2.wattage_resolution, parameter);
  WSContentSend_P(PSTR("# TYPE energy_power_active_watts gauge\nenergy_power_active_watts %s\n"), parameter);
  dtostrfd(Energy.daily, Settings.flag2.energy_resolution, parameter);
  WSContentSend_P(PSTR("# TYPE energy_power_kilowatts_daily counter\nenergy_power_kilowatts_daily %s\n"), parameter);
  dtostrfd(Energy.total, Settings.flag2.energy_resolution, parameter);
  WSContentSend_P(PSTR("# TYPE energy_power_kilowatts_total counter\nenergy_power_kilowatts_total %s\n"), parameter);
#endif

/*
  // Alternative method using the complete sensor JSON data
  // For prometheus it may need to be decoded to # TYPE messages
  mqtt_data[0] = '\0';
  MqttShowSensor();
  char json[strlen(mqtt_data) +1];
  snprintf_P(json, sizeof(json), mqtt_data);

  // Do your Prometheus specific processing here.
  // Look at function DisplayAnalyzeJson() in file xdrv_13_display.ino as an example how to decode the JSON message

  WSContentSend_P(json);
*/

  WSContentEnd();
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns75(uint8_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_WEB_ADD_HANDLER:
      Webserver->on("/metrics", HandleMetrics);
      break;
  }
  return result;
}

#endif  // USE_PROMETHEUS
