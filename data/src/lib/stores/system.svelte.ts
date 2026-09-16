export type SystemInfo = {
  deviceName: string,
  version: string,
  uptime: number,
  free_heap: number,
  wifi_ssid: string,
  wifi_rssi: number,
  eth_enabled: boolean,
  log_level: number,
  chip_model: number,
  nfc_connected: boolean,
  nfc_reader_type?: number,
  relay_paired?: boolean,
  relay_rssi?: number,
  relay_reader_ready?: boolean,
  relay_battery_mv?: number,
  mqtt_connected: boolean,
  mqtt_error_code: number,
  mqtt_error_message?: string,
  backlog_max_size: number
};

export const systemInfo : SystemInfo = $state({
  deviceName: '',
  version: '',
  uptime: 0,
  free_heap: 0,
  wifi_ssid: '',
  wifi_rssi: 0,
  eth_enabled: false,
  log_level: 2,
  chip_model: 0,
  nfc_connected: false,
  nfc_reader_type: 0,
  relay_paired: false,
  relay_rssi: 0,
  relay_reader_ready: false,
  relay_battery_mv: 0,
  mqtt_connected: false,
  mqtt_error_code: 0,
  backlog_max_size: 0
});

/**
 * Update system information from API response
 */
export function updateSystemInfo(newInfo : Partial<SystemInfo>) {
  Object.assign(systemInfo, newInfo);
}


let loadingState = $state(false);

export function setLoadingState(value: boolean) {
  loadingState = value;
}

export function getLoadingState() {
  return loadingState;
}
