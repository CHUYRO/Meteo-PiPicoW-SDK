#pragma once
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#define defaultMaxNets 40
#pragma endregion

namespace wifiLib{
    long mapcustom(long x, long in_min, long in_max, long out_min, long out_max);    
    class wifiLib{
    public:
        wifiLib(int numMaxNets);
        ~wifiLib();        
        void getWiFiStatus(); 
        void wifiConn(bool reconnect);
        void wifiOff();        
        //void wifiOff(MQTT_CLIENT_T* stateM);             

    private:
        int _numMaxNets = defaultMaxNets;
        inline static cyw43_ev_scan_result_t _network[defaultMaxNets];
        int _iReconexion;
        int _dbpercent; 
        inline static int _netnum;
        int _lastStatus; 
        long int _rssi;
        uint8_t _bssidConex[6]; 
        bool _netFound;   
        static int wifiScan(void *env, const cyw43_ev_scan_result_t *result);    
    };
}