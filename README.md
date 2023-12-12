# Meteo-PiPicoW-SDK(WIP)

Integrated atmospheric variables control board based on Pi-Pico W powered by battery(4xAA NiMh).

Raw data sent every 186-189 seconds to Thingspeak database. Data sent by mqtt protocol. 

Numerical analysis for raw data done in Thingspeak channel. 

Link to raw data channel: https://thingspeak.com/channels/1713237

Link to processed data channel: https://thingspeak.com/channels/1725298

One of the main features is that nothing is permanently soldered so it is a plug and play PCB. If any sensor or even MCU Pi-Pico W is damaged/wants to be changed, take it out and plug a new one.

By default 180 seconds of deep sleep with 2mA median consumption and 6-9 seconds of activity with 50-60mA median consumption.
NTP client and RTC used to know when to wake up, no deep sleep until NTP sets RTC.
This low power consumption can only be achieved by deep sleeping PicoW after sending data to DB in 186-189 seconds cycles.

Each cycle is composed by this actions:

1.-It wakes up and connects/reconnects to saved WiFi, after WiFi link is up it takes measurements and sends it to DB(6-9 seconds of activity).

2.-Then it turns off WiFi chip after data is sent to DB and deep sleeps keeping only RTC online to wake up when we want(180 seconds of deep sleep).

3.-Back to 1.

All welded by hand.

Main sensors: 

-DHT22: temperature and humidity control.

-BMP280: atmospheric pressure and temperature control.

-INA219: measures voltages and currents of battery pack.

-LDR for simple measurement of light conditions.

-On-chip voltage and temperature data.

Battery:

-4xAA rechargeable NiMh. 

Extra:

-Downclocked to 60MHz.

-Connection/reconnection logic if WiFi link/connection lost.

-NTP time check/update every 3 hours in order to avoid RTC time drift.

TO DO: 

-Code cleanup and reorganization needed. Mix of C/C++ needs to be rearranged.

-Add Li-Ion battery instead of 4xAA NiMh.

-Add solar panel(5-6v 1-2W) for night charging/discharging cycle. Trickle charging?. 

Images:
WIP

