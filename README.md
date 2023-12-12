# Meteo-PiPicoW-SDK(WIP)

Atmospheric variables control based on Pi-Pico W powered by battery(4xAA NiMh).

Raw data sent every 186-189 seconds to Thingspeak db. Data sent by mqtt protocol. 

Numerical analysis on raw data done in Thingspeak channel. 

Link to raw data channel: https://thingspeak.com/channels/1713237

Link to processed data channel: https://thingspeak.com/channels/1725298

By default 180 seconds of deep sleep with ≈2mA consumption and 6-9 seconds of activity with 50-60mA consumption.
NTP client and RTC used to know when to wake up, no deep sleep until NTP sets RTC.
This low power consumption can only be achieved by deep sleeping PicoW after sending data to DB in 186-189 seconds cycles.

In short, each cycle is composed by this actions:

1.-Wakes up and connects to saved WiFi, after WiFi link is up it takes measurements and sends it to DB(6-9 seconds of activity, depends on signal strengh).

2.-Then checks if everything ok. If ok turns off WiFi chip and deep sleeps keeping only RTC online to wake up when we want(by default 180 seconds of deep sleep).

3.-Back to 1.

All welded by me so room for improvement...

Sensors: 

-DHT22: temperature and humidity control.

-BMP280: atmospheric pressure and temperature control.

-INA219: measures voltage and current of battery pack.

-LDR for simple measurement of light conditions.

-On-chip voltage and temperature data.

Battery pack:

-4xAA rechargeable NiMh. 

Extra:

-Can be downclocked all the way down to 50-60MHz.

-Wifi scan and connection/reconnection logic if WiFi link down.

-NTP time check/update every 3 days in order to avoid RTC time drift.

TO DO: 

-Some code cleanup  needed. Mix of C/C++ needs to be rearranged.

-Test Li-Ion battery instead of 4xAA NiMh.

-Add solar panel(5-6v 1-2W) for night charging/discharging cycle. Trickle charging NiMh?. 

Images:

-WIP...

