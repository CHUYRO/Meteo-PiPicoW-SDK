# Meteo-PiPicoW-SDK

Atmospheric variables control based on Pi-Pico W powered by battery(4xAA NiMh).

Raw data sent every 186-189 seconds to Thingspeak db. Data sent by MQTT. 

Analysis on raw data done in Thingspeak channel via MATLAB. 

Link to raw data channel: https://thingspeak.com/channels/1713237

Link to processed data channel: https://thingspeak.com/channels/1725298

By default 180 seconds of deep sleep with ≈2mA consumption and 6-9 seconds of activity with 50-60mA consumption.
NTP client and RTC used to know when to wake up, no deep sleep until NTP sets RTC.
In short, each cycle is composed by this actions:

1.-Wakes up and connects to saved WiFi, after WiFi link is up it takes measurements and sends it to DB(6-9 seconds of activity, depends on signal strengh).

2.-Then checks if everything ok. If ok turns off WiFi chip and deep sleeps keeping only RTC online to wake up when set(by default every 180 seconds).

3.-Back to 1.

All welded by me so room for improvement...

Sensors: 

-DHT22: temperature and humidity.

-BMP280: atmospheric pressure and temperature.

-INA219: power control.

-LDR for simple measurement of light conditions.

-On-chip voltage and temperature data.

Battery pack:

-4xAA rechargeable NiMh. 

Extra:

-Over/Underclock control.

-Wifi scan and connection/reconnection logic to set ssids.

-NTP time check/update every 3 days in order to avoid RTC time drift.

TO DO: 

-Some code cleanup needed. Mix of C/C++ needs to be rearranged(dht22 lib).

-Test Li-Ion battery instead of 4xAA NiMh.

-Add solar panel(5-6v 1-2W) for night charging/discharging cycle. Trickle charging NiMh?. 

Images:

-WIP...

