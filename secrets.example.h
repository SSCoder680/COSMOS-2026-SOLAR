#pragma once

// Copy these values into secrets.h. Never commit your real credentials.
#define SECRET_SSID "your-wifi-name"
#define SECRET_PASS ""  // Leave empty for an open guest network.
#define SECRET_CH_ID 1234567UL
#define SECRET_WRITE_APIKEY "your-channel-write-api-key"

// Optional exact coordinates. Leave both at 999.0 to estimate location from
// the Wi-Fi network's public IP address.
#define SECRET_LATITUDE 999.0
#define SECRET_LONGITUDE 999.0
