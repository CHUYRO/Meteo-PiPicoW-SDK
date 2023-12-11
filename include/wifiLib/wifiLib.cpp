#include "wifiLib.hpp"

#define DEBUG false  

namespace wifiLib {
    long mapcustom(long x, long in_min, long in_max, long out_min, long out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }    
    // ---------------- Constructor/Destructor ----------------
    #pragma region Constructor/Destructor
    /// @brief Default Constructor.
    /// @param numMaxNets Scanner max nets.
    wifiLib::wifiLib(int numMaxNets){
        _numMaxNets = numMaxNets;
        _iReconexion = 0;
        _dbpercent = 0; 
        _netnum = 0;
        _lastStatus = 33; 
        _rssi = 0;
    }

    /// @brief Default destructor.
    wifiLib::~wifiLib(){}
    #pragma endregion

    //----- Utilities ----
    int wifiLib::wifiScan(void *env, const cyw43_ev_scan_result_t *result){
        if(result){   
            _network[_netnum] = *result;    
            _netnum++;  
        }
        return 0;
    }
    void wifiLib::getWiFiStatus(){        
        cyw43_wifi_get_rssi(&cyw43_state, &_rssi);
        _dbpercent = mapcustom(_rssi,-95,-20,0,100); 
        #if DEBUG      
        printf("\nConectado.\n");  
        printf("SSID: %s\n", WIFI_SSID);
        printf("RSSI: %li -- %i% \n", _rssi,_dbpercent);
        auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
        printf("IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
        printf("I.Reconexion: %i\n",_iReconexion); 
        #endif
    }
    void wifiLib::wifiConn(bool reconnect){                
        if(reconnect==true){
            printf("\n---------------- RECONECTANDO ---------------\n");
            _iReconexion++;   
            cyw43_arch_deinit();
        }      
        if (cyw43_arch_init_with_country(CYW43_COUNTRY('E', 'S', 0)) != 0) {        
            printf("\n\nERROR cyw43------");    
            wifiConn(true);
        }
        if(!cyw43_is_initialized(&cyw43_state)){       
            printf("--NO INICIALIZADO-----------\n");
            wifiConn(true);
        }
        cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_PM1_POWERSAVE_MODE, 20, 1, 1, 1));   
        cyw43_arch_enable_sta_mode();
        netif_set_hostname(netif_default, PICOW_HOSTNAME);
        #if DEBUG
            uint64_t elapTime = 0, elapTimeend = 0, elapTimeendRaw = 0;
            elapTime = time_us_64();
            cyw43_wifi_scan_options_t scan_options = {0};  
            int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, wifiScan);
            if(err == 0){printf("\n-Escaneando redes WiFi");}
            else{printf("-ERROR---Scan redes WiFi: %d\n", err);wifiConn(true);}  
            while(cyw43_wifi_scan_active(&cyw43_state) == true){      
            elapTimeend = time_us_64();   
            if(elapTimeendRaw + 200000 < elapTimeend){
                printf(".");
                elapTimeendRaw=elapTimeend;
            }      
            if(elapTimeend-elapTime > 5000000){ //5 seg timeout
                printf("-ERROR TIMEOUT---Scan redes WiFi err: %d\n", err);
                wifiConn(true);            
            }
            cyw43_arch_poll();
            sleep_ms(1);
            } 
            printf("\n");
            uint8_t bssid[6],BSSIDRaw[_netnum][6],AUTHRaw[_netnum],*SSIDRaw[_netnum]; 
            uint16_t CHANNELRaw[_netnum];
            int16_t RSSIRaw[_netnum];
            int Duplic[_netnum],totNets=0; 
            for(int x = 0; x < _netnum; x++){Duplic[x]=0;AUTHRaw[x]=0;SSIDRaw[x]=0;CHANNELRaw[x]=0;RSSIRaw[x]=0;for(int y = 0; y < 6; y++){BSSIDRaw[x][y]=0;}}      
            for(int i = 0; i < _netnum; i++){                
            for(int j = 0; j < _netnum; j++){
                if(_network[i].bssid[0] == _network[j].bssid[0] && _network[i].bssid[1] == _network[j].bssid[1] && _network[i].bssid[2] == _network[j].bssid[2] && _network[i].bssid[3] == _network[j].bssid[3] && _network[i].bssid[4] == _network[j].bssid[4] && _network[i].bssid[5] == _network[j].bssid[5]){    
                Duplic[i]++;      
                if(_network[i].bssid[0] != BSSIDRaw[j][0] || _network[i].bssid[1] != BSSIDRaw[j][1] || _network[i].bssid[2] != BSSIDRaw[j][2] || _network[i].bssid[3] != BSSIDRaw[j][3] || _network[i].bssid[4] != BSSIDRaw[j][4] || _network[i].bssid[5] != BSSIDRaw[j][5]){       
                    if(Duplic[i]==1 && _network[i].rssi != 0 && _network[i].channel != 0){           
                    AUTHRaw[i] = _network[i].auth_mode;
                    RSSIRaw[i] = _network[i].rssi;
                    CHANNELRaw[i] = _network[i].channel;
                    SSIDRaw[i] = static_cast<uint8_t*>(_network[i].ssid);
                    BSSIDRaw[i][0] = _network[i].bssid[0];  
                    BSSIDRaw[i][1] = _network[i].bssid[1];     
                    BSSIDRaw[i][2] = _network[i].bssid[2]; 
                    BSSIDRaw[i][3] = _network[i].bssid[3];  
                    BSSIDRaw[i][4] = _network[i].bssid[4];
                    BSSIDRaw[i][5] = _network[i].bssid[5]; 
                    totNets++;            
                    if(*SSIDRaw[i] == 0){
                        SSIDRaw[i]=(uint8_t*)"NO_SSID";   
                    }
                    if(strcmp(WIFI_SSID,(const char *)SSIDRaw[i])==0){_netFound=true;}
                    printf("\n-ssid: %s -rssi:%d -ch:%d\n-mac: %lu:%lu:%lu:%lu:%lu:%lu -sec:%u\n",SSIDRaw[i],RSSIRaw[i], CHANNELRaw[i], BSSIDRaw[i][0], BSSIDRaw[i][1], BSSIDRaw[i][2], BSSIDRaw[i][3], BSSIDRaw[i][4], BSSIDRaw[i][5],AUTHRaw[i]);
                    }         
                }                                                 
                }       
            }         
            }  
            elapTimeend = time_us_64();   
            _netnum=0;
        #else
            _netFound=true;  
        #endif
        if(_netFound==true){
            #if DEBUG
            printf("\n-%s encontrada! -- %i redes totales -- ScanTime: %lli ms\n",WIFI_SSID,totNets,(elapTimeend-elapTime)/1000);
            #endif
            if(cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK) != 0) {
            printf("-ERROR--Conexión WiFi Async--E: %i\n",cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK));
            wifiConn(true);
            }else{
            #if DEBUG
                printf("\n-Conectando a %s...\n", WIFI_SSID);
            #endif
            }          
            while (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) != CYW43_LINK_UP){
            cyw43_arch_poll();
            sleep_ms(1);
            if (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) != _lastStatus){
                _lastStatus = cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA);
                switch (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)){      
                case CYW43_LINK_UP:
                { 
                    #if DEBUG 
                    cyw43_wifi_get_rssi(&cyw43_state, &_rssi);                   
                    cyw43_wifi_get_bssid(&cyw43_state, _bssidConex);                
                    if(reconnect==true){printf("\n-Reconectado.\n");}else{printf("\n-Conectado.\n");}  
                    printf("--SSID: %s\n", WIFI_SSID);
                    printf("--BSSID: %lu:%lu:%lu:%lu:%lu:%lu\n", _bssidConex[0],_bssidConex[1],_bssidConex[2],_bssidConex[3],_bssidConex[4],_bssidConex[5]);
                    printf("--RSSI: %li\n", _rssi);  
                    auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
                    printf("--IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
                    #endif
                    break;
                }
                case CYW43_LINK_DOWN:
                    #if DEBUG 
                    printf("\n-CYW43_LINK_DOWN.\n");
                    #endif
                    wifiConn(true);
                    break;    
                case CYW43_LINK_NOIP:
                    #if DEBUG 
                    printf("\n-CYW43_LINK_NOIP.\n");
                    #endif
                    break; 
                case CYW43_LINK_FAIL:
                    #if DEBUG
                    printf("\n-CYW43_LINK_FAIL.\n");
                    #endif
                    wifiConn(true);
                    break;
                case CYW43_LINK_NONET:
                    #if DEBUG 
                    printf("\n-CYW43_LINK_NONET.\n");
                    #endif
                    wifiConn(true);
                    break;    
                case CYW43_LINK_BADAUTH:
                    #if DEBUG 
                    printf("\n-CYW43_LINK_BADAUTH.\n");
                    #endif
                    wifiConn(true);
                    break;       
                default: 
                    break;
                };
            }
            }
        }else{printf("\n-%s NO encontrada!\n", WIFI_SSID);_netFound=false;wifiConn(true);}
    }
    void wifiLib::wifiOff(){
        cyw43_arch_deinit();
        switch (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)){
        case CYW43_LINK_UP:
        { 
            printf("\n--------ERROR no deberia estar conectado.-----------\n"); 
            cyw43_wifi_get_rssi(&cyw43_state, &_rssi);               
            printf("\nConectado.\n");  
            printf("SSID: %s\n", WIFI_SSID);
            printf("RSSI: %li\n", _rssi);
            auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
            printf("IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24); 
            printf("\n----------ERROR no deberia estar conectado.-------\n");                   
            break;
        }
        case CYW43_LINK_DOWN:
            #if DEBUG
                printf("\n-WiFi chip switched OFF!\n");
            #endif                    
            break;    
        case CYW43_LINK_NOIP:
            printf("\nCYW43_LINK_NOIP.\n");
            break; 
        case CYW43_LINK_FAIL:
            printf("\nCYW43_LINK_FAIL.\n");                    
            break;
        case CYW43_LINK_NONET:
            printf("\nCYW43_LINK_NONET.\n");                    
            break;    
        case CYW43_LINK_BADAUTH:
            printf("\nCYW43_LINK_BADAUTH.\n");                   
            break;       
        default:
            break;
        };
    }
}