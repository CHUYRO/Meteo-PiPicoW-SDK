#include <stdio.h>
#include <pico/stdlib.h>
#include "pico/cyw43_arch.h"
#include "pico/sleep.h"

#include "lwipopts.h"
#include "lwip/init.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

#include "hardware/adc.h"
#include "hardware/vreg.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"

#include <dht/include/dht.h>
#include <Adafruit_INA219.h>

#define TESTING 0

//-USB SERIAL WAIT-------------
#include "tusb.h"

//-RTC-------------
unsigned int SegundosRTC = 0, MinutosRTC = 0, HorasRTC = 0, DiaRTC=0, MesRTC=0, DiaWRTC=0, AñoRTC=0;

//-TEMPCOMB--------
float Tcomb = 0;

//-LDR--------------
float ldrVal = 0, ldrValRaw = 0;

//-BMP280-----------
int32_t raw_temperature;
int32_t raw_pressure;
int32_t temperature;
int32_t pressure;
//device has default bus address of 0x76
#define ADDR _u(0x76)
//hardware registers
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)
#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)
#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)
//calibration registers
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)
//number of calibration registers to be read
#define NUM_CALIB_PARAMS 24
struct bmp280_calib_param {
    //temperature params
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    //pressure params
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
};
//retrieve fixed compensation params
struct bmp280_calib_param params;

//-DHT22-------------
static const dht_model_t DHT_MODEL = DHT22;
static const uint DATA_PIN = 16;
float humidity;
float temperature_c;
dht_t dht;

//-INA219------------------------- 
Adafruit_INA219 ina219;
float total_mA = 0, shuntvoltage = 0,busvoltage = 0, current_mA = 0, total_mAH = 0, power_mW = 0, loadvoltage = 0, current_A = 0, power_W = 0, Amp = 0, total_mAM = 0,shuntRaw=0, busvRaw=0,
currentmARaw=0,power_mWRaw=0;

//-READ VSYS----------------
float old_voltage = 0, voltage = 0, adc = 0, tempC = 0;
int voltage_return = 0;
const float conversionFactor = 3.3f / (1 << 12);
#ifndef PICO_POWER_SAMPLE_COUNT
#define PICO_POWER_SAMPLE_COUNT 5
#endif
//Pin used for ADC 0
#define PICO_FIRST_ADC_PIN 26

//-FREQ.-----------------
#define SYS_KHZ (60 * 1000) //Downclocked to 60

//-TIME------------------
uint64_t USec = 0, USecRaw = 0, Millis = 0, MillisRaw = 0, MillisTot = 0, Minutos = 0, MinutosRaw = 0, MinutosTot = 0, Segundos = 0, SegundosRaw = 0, SegundosTot = 0, Horas = 0, HorasRaw = 0, HorasTot = 0, debugControlDia = 0, Dias = 0, DiasTot = 0, Ttotal = 0, elapTime = 0, elapTimeend = 0,ntpHoras = 0, StartingSEC = 0;
int8_t Horantp = 0, Minutontp = 0, Segundontp = 0;
unsigned int TiempoWait = 0, TiempoLoop = 0, diaint = 0, dianum = 0, mes = 0, año = 0, MinutoComienzo = 0, HoraComienzo = 0, SegundoComienzo = 0, DiaComienzo = 0, MesComienzo = 0;
bool movida = false, InitTimeGet = false;
long int movidaDebug = 0, minutos = 0, hora = 0, segundos = 0, ErrorMQTT = 0, ErrorNTP = 0, Timeout = 0;

//-WIFI-----------------
long int rssi = 0;
uint8_t BSSIDRaw[30]; 
int iReconexion = 0, dbpercent = 0, netnum = 0;
cyw43_ev_scan_result_t network[30];
int Duplic[30];

//-NTP------------------
typedef struct NTP_T_ {
    ip_addr_t ntp_server_address;
    bool dns_request_sent;
    struct udp_pcb *ntp_pcb;
    absolute_time_t ntp_test_time;
    alarm_id_t ntp_resend_alarm;
} NTP_T;
#define NTP_SERVER "0.es.pool.ntp.org" //ntp server
#define NTP_MSG_LEN 48
#define NTP_PORT 123 
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
//#define NTP_TEST_TIME (14 * 1000)
//#define NTP_RESEND_TIME (7 * 1000)
datetime_t datetimeNTP = { //not needed cast...try better way...
    .year = (int16_t) 0,
    .month = (int8_t) 0,
    .day = (int8_t) 0,
    .dotw = (int8_t) 0,
    .hour = (int8_t) 0,
    .min = (int8_t) 0,
    .sec = (int8_t) 0
};
NTP_T *state = (NTP_T *)calloc(1, sizeof(NTP_T));

//-SLEEP------------------------------
uint scb_orig = 0, clock0_orig = 0, clock1_orig = 0;
bool mqttdone=false,dhtdone=false,ldrdone=false,bmpdone=false,Tempcombdone=false,inadone=false,serialdone=false,ntpupdated=false,mqttproceso=false,ntpproceso=false;

//MQTT---------------
int ChannelID = 1713237;
#define MQTT_SERVER_HOST "mqtt3.thingspeak.com"
#define MQTT_SERVER_PORT 1883
#define MQTT_TLS 0 // needs to be 1 for AWS IoT. Also set published QoS to 0 or 1
//#define CRYPTO_MOSQUITTO_LOCAL
//#define CRYPTO_AWS_IOT
//#include "crypto_consts.h"

#if MQTT_TLS
#ifdef CRYPTO_CERT
const char *cert = CRYPTO_CERT;
#endif
#ifdef CRYPTO_CA
const char *ca = CRYPTO_CA;
#endif
#ifdef CRYPTO_KEY
const char *key = CRYPTO_KEY;
#endif
#endif
typedef struct MQTT_CLIENT_T_ {
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

//-----------------------------------FUNCIONES-------------------------------

//-INTERNAL TIME OF EXEC.--------------------------
void execTime(){     
    USec = time_us_64() - (TiempoLoop+TiempoWait);
    Millis = USec / 1000;
    Segundos = Millis / 1000;
    Minutos = Segundos / 60;
    Horas = Minutos / 60;
    Dias = Horas / 24;

    MillisTot = Millis - Segundos*1000;
    SegundosTot = Segundos - Minutos*60;
    MinutosTot = Minutos - Horas*60;
    HorasTot = Horas - Dias*24;
    DiasTot = Dias;

    if(MillisTot%1000 == 0 && MillisTot != MillisRaw){
        MillisTot = 0;
        MillisRaw = MillisTot;        
    } 
    if(SegundosTot%60 == 0 && SegundosTot != SegundosRaw){
        SegundosTot = 0;
        SegundosRaw = SegundosTot;        
    }           
    if(MinutosTot%60 == 0 && MinutosTot != MinutosRaw){
        MinutosTot = 0;
        MinutosRaw = MinutosTot;
    }
    if(HorasTot%24 == 0 && HorasTot != HorasRaw){ 
        HorasTot = 0;
        HorasRaw = HorasTot;
    } 
}

//-RTC-----------------------------------------------------------------
//-RTC MOVIDAS VARIAS DE NUESTROS AMIGOS DE RASPBERRY PI ;)
static bool valid_datetimeLOL(datetime_t *t) {
    // Valid ranges taken from RTC doc. Note when setting an RTC alarm
    // these values are allowed to be -1 to say "don't match this value"
    if (!(t->year >= 0 && t->year <= 4095)) return false;
    if (!(t->month >= 1 && t->month <= 12)) return false;
    if (!(t->day >= 1 && t->day <= 31)) return false;
    if (!(t->dotw >= 0 && t->dotw <= 6)) return false;
    if (!(t->hour >= 0 && t->hour <= 23)) return false;
    if (!(t->min >= 0 && t->min <= 59)) return false;
    if (!(t->sec >= 0 && t->sec <= 59)) return false;
    return true;
}
bool rtc_set_datetimeLOL(datetime_t *t) {

    if (!valid_datetimeLOL(t)) {
        return false;
    }

    // Disable RTC
    rtc_hw->ctrl = 0;
    // Wait while it is still active
    while (rtc_running()) {
        tight_loop_contents();
    } 

    // Write to setup registers
    rtc_hw->setup_0 = (((uint32_t)t->year)  << RTC_SETUP_0_YEAR_LSB ) |
                      (((uint32_t)t->month) << RTC_SETUP_0_MONTH_LSB) |
                      (((uint32_t)t->day)   << RTC_SETUP_0_DAY_LSB);
    rtc_hw->setup_1 = (((uint32_t)t->dotw)  << RTC_SETUP_1_DOTW_LSB) |
                      (((uint32_t)t->hour)  << RTC_SETUP_1_HOUR_LSB) |
                      (((uint32_t)t->min)   << RTC_SETUP_1_MIN_LSB)  |
                      (((uint32_t)t->sec)   << RTC_SETUP_1_SEC_LSB);

    // Load setup values into rtc clock domain
    rtc_hw->ctrl = RTC_CTRL_LOAD_BITS;

    // Enable RTC and wait for it to be running
    rtc_hw->ctrl = RTC_CTRL_RTC_ENABLE_BITS;
    while (!rtc_running()) {
        tight_loop_contents();
    }

    busy_wait_us(64);
    // Reload it again on the fly to make it right
    rtc_hw->ctrl = RTC_CTRL_LOAD_BITS | RTC_CTRL_RTC_ENABLE_BITS;
    busy_wait_us(64); 

    return true;
}

void setTimeRTC(){       
    if(InitTimeGet==true){
        datetimeNTP = {
            .year = (int16_t) (año),
            .month = (int8_t) (mes),
            .day = (int8_t) (dianum),
            .dotw = (int8_t) (diaint),
            .hour = (int8_t) (hora),
            .min = (int8_t) (minutos),
            .sec = (int8_t) (segundos)
        };        
        if(rtc_set_datetimeLOL(&datetimeNTP)==true){  
            execTime();          
            //printf("\n-RTC(set):%02i:%02i:%02i T:%02lli:%02lli:%03lli\n",datetimeNTP.hour,datetimeNTP.min,datetimeNTP.sec,MinutosTot,SegundosTot,MillisTot);
            ntpupdated=true; 
            ntpproceso=false;
            Horantp=datetimeNTP.hour;
            Minutontp=datetimeNTP.min;
            Segundontp=datetimeNTP.sec;                             
        }
        else{
            //printf("Horas BAD!\n");
        }                           
    }         
}

void RTC(){ 
    rtc_get_datetime(&datetimeNTP);
    SegundosRTC = datetimeNTP.sec; 
    MinutosRTC = datetimeNTP.min;
    HorasRTC = datetimeNTP.hour;  
    DiaRTC = datetimeNTP.day;
    MesRTC = datetimeNTP.month; 
    DiaWRTC = datetimeNTP.dotw;
    AñoRTC = datetimeNTP.year;
}

//-MQTT---------------------------------------------------------
static MQTT_CLIENT_T* mqtt_client_init(void) {
    MQTT_CLIENT_T *stateM = (MQTT_CLIENT_T *)calloc(1, sizeof(MQTT_CLIENT_T));    
    if (!stateM) {
        printf("-failed to allocate state\n");
        return NULL;
    }
    stateM->received = 0;
    return stateM;
}

void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    MQTT_CLIENT_T *stateM = (MQTT_CLIENT_T*)callback_arg;
    printf("-DNS query finished with resolved addr of %s.\n", ip4addr_ntoa(ipaddr));
    stateM->remote_addr = *ipaddr;
}

void run_dns_lookup(MQTT_CLIENT_T *state) {
    printf("\n-Running mqtt DNS query for %s.\n", MQTT_SERVER_HOST);

    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(MQTT_SERVER_HOST, &(state->remote_addr), dns_found, state);
    cyw43_arch_lwip_end();

    if (err == ERR_ARG) {
        printf("-Failed to start mqtt DNS query\n");
        return;
    }

    if (err == ERR_OK) {
        printf("-no dns mqtt lookup needed");
        return;        
    }

    while (state->remote_addr.addr == 0 && Timeout <= 15000) {
        cyw43_arch_poll();
        Timeout++;  
        busy_wait_ms(1);        
    }
    Timeout = 0;
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status != 0) {} 
    else {
        //printf("\n-MQTT connected.\n"); 
        mqttproceso = false;                      
    }
    printf("-mqtt_connection_cb: err %d.\n", status);
}

void mqtt_pub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_T *stateM = (MQTT_CLIENT_T *)arg;
    printf("-mqtt_pub_request_cb: err %d\n", err);
    if(err==ERR_OK){
        stateM->counter++;  
        printf("-MQTT Ok:%ld Error:%ld\n", stateM->counter,ErrorMQTT); 
        mqttdone=true;
    }      
    stateM->received++;
    mqttproceso = false;
    StartingSEC=Segundos;
}

err_t mqtt_test_publish(MQTT_CLIENT_T *state){
    char buffer[128];
    err_t err;
    u8_t qos = 0; /* 0, 1 or 2, ThingSpeak is 0.*/
    u8_t retain = 0;    
    sprintf(buffer, "field1=%.2f&field2=%.2f&field3=%.3f&field4=%.2f&field5=%.2f&field6=%.2f&field7=%f&field8=%.2f", total_mAH,humidity,pressure/ 100.f,ldrVal,Tcomb,loadvoltage,voltage,Amp);
    String topicString = "channels/"+String(ChannelID)+"/publish"; 
    mqttproceso = true;
    cyw43_arch_lwip_begin();
    err = mqtt_publish(state->mqtt_client, topicString.c_str(), buffer, strlen(buffer), qos, retain, mqtt_pub_request_cb, state);
    cyw43_arch_lwip_end();
    printf("-mqtt_test_publish: err %d\n", err);

    return err; 
}

err_t mqtt_test_connect(MQTT_CLIENT_T *state) {
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = MQTT_CLIENT_ID; 
    ci.client_user = MQTT_CLIENT_ID;
    ci.client_pass = MQTT_CLIENT_PSW;
    ci.keep_alive = 0;
    ci.will_topic = NULL;
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = 0;

    #if MQTT_TLS

    struct altcp_tls_config *tls_config;
  
    #if defined(CRYPTO_CA) && defined(CRYPTO_KEY) && defined(CRYPTO_CERT)
    printf("Setting up TLS with 2wayauth.\n");
    tls_config = altcp_tls_create_config_client_2wayauth(
        (const u8_t *)ca, 1 + strlen((const char *)ca),
        (const u8_t *)key, 1 + strlen((const char *)key),
        (const u8_t *)"", 0,
        (const u8_t *)cert, 1 + strlen((const char *)cert)
    );
    // set this here as its a niche case at the moment.
    // see mqtt-sni.patch for changes to support this.
    ci.server_name = MQTT_SERVER_HOST;
    #elif defined(CRYPTO_CERT)
    printf("Setting up TLS with cert.\n");
    tls_config = altcp_tls_create_config_client((const u8_t *) cert, 1 + strlen((const char *) cert));
    #endif

    if (tls_config == NULL) {
        printf("Failed to initialize config\n");
        return -1;
    }

    ci.tls_config = tls_config;
    #endif

    const struct mqtt_connect_client_info_t *client_info = &ci;
    mqttproceso = true;    
    cyw43_arch_lwip_begin();
    err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr), MQTT_SERVER_PORT, mqtt_connection_cb, state, client_info);
    cyw43_arch_lwip_end();
    printf("-mqtt_test_connect: err %d\n", err);

    return err;
}

void MQTT(MQTT_CLIENT_T* stateM){
    if(mqttdone == false && mqttproceso == false){ 
        if(mqtt_client_is_connected(stateM->mqtt_client)) {                
            cyw43_arch_lwip_begin(); 
            if(mqtt_test_publish(stateM) == ERR_OK){}else{printf("-Full ringbuffer...Wait\n");} 
            cyw43_arch_lwip_end();                
        }else{
            int errReconnect = mqtt_test_connect(stateM);     
            if(errReconnect != ERR_OK && errReconnect != ERR_ISCONN){               
                printf("-Connection BAD!\n");
                ErrorMQTT++;
            }else if(errReconnect == ERR_ISCONN){  
                printf("-Connection already ON!\n");         
            }                  
        }         
    }
}

//-SLEEP-------------------------------------
static void sleep_callback(void) {
    //printf("RTC woke us up\n"); 
    return;   
}

static void rtc_sleep_custom(uint minute_to_sleep_to, uint second_to_sleep_to) {
    
    uint secondTO = second_to_sleep_to + (uint)(datetimeNTP.sec);
    uint minuteTO = minute_to_sleep_to + (uint)(datetimeNTP.min);
    uint horaTO = datetimeNTP.hour;
    uint diaTO = datetimeNTP.day;
    uint diaWTO = datetimeNTP.dotw;
    uint mesTO = datetimeNTP.month;
    uint añoTO = datetimeNTP.year;

    while(secondTO>=60 || minuteTO>=60){
        if(secondTO>=60){
            secondTO-=60;
            minuteTO++;
        }
        if(minuteTO>=60){
            minuteTO-=60;
            horaTO++;
            if(horaTO==24){
                horaTO=0;
                diaTO++;
                diaWTO++;
                if(diaWTO>6){
                    diaWTO=0;
                }
                if(añoTO%4==0){//Bisiesto Feb 29dias
                    if(mesTO==1||mesTO==3||mesTO==5||mesTO==7||mesTO==8||mesTO==10||mesTO==12){
                        if(diaTO>31){
                            diaTO=1;
                            mesTO++;
                            //if(mes>12)añoTO++ do algo...
                        }
                    }
                    else if(mesTO==4||mesTO==6||mesTO==9||mesTO==11){
                        if(diaTO>30){
                            diaTO=1;
                            mesTO++;
                        }
                    }
                    else if(mesTO==2){
                        if(diaTO>29){
                            diaTO=1;
                            mesTO++;
                        }
                    }
                }
                else{//NO bisiesto Feb 28dias
                    if(mesTO==1||mesTO==3||mesTO==5||mesTO==7||mesTO==8||mesTO==10||mesTO==12){
                        if(diaTO>31){
                            diaTO=1;
                            mesTO++;
                            //if(mesTO>12)
                        }
                    }
                    else if(mesTO==4||mesTO==6||mesTO==9||mesTO==11){
                        if(diaTO>30){
                            diaTO=1;
                            mesTO++;
                        }
                    }
                    else if(mesTO==2){
                        if(diaTO>28){
                            diaTO=1;
                            mesTO++;
                        }
                    }
                }
            }
        }
    }    

    datetime_t t_alarm = {
            .year  = (int16_t)(añoTO),
            .month = (int8_t)(mesTO),
            .day   = (int8_t)(diaTO),
            .dotw  = (int8_t)(diaWTO), // 0 is Sunday, so 5 is Friday
            .hour  = (int8_t)(horaTO),
            .min   = (int8_t)(minuteTO),
            .sec   = (int8_t)(secondTO)
    };    
    sleep_goto_sleep_until(&t_alarm, &sleep_callback);
}

void recover_from_sleep(){

    //Re-enable ring Oscillator control
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

    //reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = clock0_orig;
    clocks_hw->sleep_en1 = clock1_orig;

    //reset clocks
    clocks_init();
    //try to reinit usb
    //stdio_init_all();

    return;
}
//Measure freqs.
void measure_freqs() {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
    //-SLEEP----------------    
    //save clock values for awake
    scb_orig = scb_hw->scr;
    clock0_orig = clocks_hw->sleep_en0;
    clock1_orig = clocks_hw->sleep_en1;

    printf("\n-FREQS.");
    printf("\npll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
    printf("clk_rtc  = %dkHz\n", f_clk_rtc); 
    // Can't measure clk_ref / xosc as it is the ref
}

//-LDR---------------------------------------
void LDRLoop() {
    if(ldrdone==false){
        adc_init();        
        adc_gpio_init(27);        
        adc_select_input(1);
        //float LUX = adc_read()*(3.3/1023);
        ldrValRaw = 0;
        for (int i = 0; i < 10; i++){
            ldrValRaw += 140 - adc_read();   
            busy_wait_ms(10);         
        }
        ldrVal = ldrValRaw / 10;
        if (ldrVal < 5) {
            ldrVal = 0.001;
        }
        ldrdone=true;
    }
}
    
//-BMP280------------------------------------
#ifdef i2c_default
void bmp280_init() {
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];

    // 500ms sampling time, x16 filter
    //const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;
    // XDms sampling time, x1 filter
    const uint8_t reg_config_val = ((0x07 << 5) | (0x01 << 2)) & 0xFC;

    // send register number followed by its corresponding value
    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;
    i2c_write_blocking(i2c1, ADDR, buf, 2, false);

    // osrs_t x1, osrs_p x4, normal mode operation
    //const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
    // osrs_t x1, osrs_p x1, forced mode operation
    const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x01 << 2) | (0x01);
    // osrs_t x1, osrs_p x1, normal mode operation
    //const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x01 << 2) | (0x03);
    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    i2c_write_blocking(i2c1, ADDR, buf, 2, false);
}

void bmp280_read_raw(int32_t* temp, int32_t* pressure) {
    // BMP280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
    // note: normal mode does not require further ctrl_meas and config register writes
    bmp280_init();

    uint8_t buf[6];
    uint8_t reg = REG_PRESSURE_MSB;
    i2c_write_blocking(i2c1, ADDR, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(i2c1, ADDR, buf, 6, false);  // false - finished with bus

    // store the 20 bit read in a 32 bit signed integer for conversion
    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
}

void bmp280_reset() {
    // reset the device with the power-on-reset procedure
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(i2c1, ADDR, buf, 2, false);
}

// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
int32_t bmp280_convert(int32_t temp, struct bmp280_calib_param* params) {
    // use the 32-bit fixed point compensation implementation given in the
    // datasheet
    
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
    return var1 + var2;
}

int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
    int32_t t_fine = bmp280_convert(temp, params);
    return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp, struct bmp280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = bmp280_convert(temp, params);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}

void bmp280_get_calib_params(struct bmp280_calib_param* params) {
    // raw temp and pressure values need to be calibrated according to
    // parameters generated during the manufacturing of the sensor
    // there are 3 temperature params, and 9 pressure params, each with a LSB
    // and MSB register, so we read from 24 registers

    uint8_t buf[NUM_CALIB_PARAMS] = { 0 };
    uint8_t reg = REG_DIG_T1_LSB;
    i2c_write_blocking(i2c1, ADDR, &reg, 1, true);  // true to keep master control of bus
    // read in one go as register addresses auto-increment
    i2c_read_blocking(i2c1, ADDR, buf, NUM_CALIB_PARAMS, false);  // false, we're done reading

    // store these in a struct for later use
    params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];
}
#endif

void bmp280Setup(){
    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(18, GPIO_FUNC_I2C);
    gpio_set_function(19, GPIO_FUNC_I2C);
    gpio_pull_up(18);
    gpio_pull_up(19);

    // configure BMP280
    bmp280_init();
    bmp280_get_calib_params(&params);
    sleep_ms(250); // sleep so that data polling and register update don't collide
    //printf("\n-BMP280 SETUP\n");
}

void bmp280() {
    if(bmpdone==false){
        bmp280_read_raw(&raw_temperature, &raw_pressure);
        temperature = bmp280_convert_temp(raw_temperature, &params)-1.2; // average offset caused by chip heating
        pressure = bmp280_convert_pressure(raw_pressure, raw_temperature, &params);
        bmpdone=true;
    }
}

//-DHT22------------------------------------
void dht22() {
    if(dhtdone==false){
        dht_init(&dht, DHT_MODEL, pio0, DATA_PIN, true);
        //sleep_ms(20);
        dht_start_measurement(&dht);
        dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);
        if (result == DHT_RESULT_OK) {            
            //printf("\n%.1f C , %.1f hum\n", temperature_c, humidity);
        } else if (result == DHT_RESULT_TIMEOUT) {
            printf("DHT sensor not responding. Please check your wiring.");
        } else {
            assert(result == DHT_RESULT_BAD_CHECKSUM);
            printf("Bad checksum");
        } 
        dhtdone=true;
        dht_deinit(&dht);       
    }
}

//-INA219------------------------------------------
void ina219Setup(){
    // Try to initialize the INA219
    if (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
    }
    else{
        Serial.println("\n-INA219 SETUP");    
    }
    //Default Calib.
    //ina219.setCalibration_32V_2A();
    // To use a slightly lower 32V, 1A range (higher precision on amps): 
    //ina219.setCalibration_32V_1A(); 
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    ina219.setCalibration_16V_400mA();
}

void LoopINA219() {
    if(inadone==false){
        ina219.powerSave(false);
        sleep_ms(50);
        for(int i = 0; i <= 20; i++){
            shuntRaw += ina219.getShuntVoltage_mV();
            busvRaw += ina219.getBusVoltage_V();
            currentmARaw += ina219.getCurrent_mA();
            power_mWRaw += ina219.getPower_mW();            
        }
        ina219.powerSave(true);
        shuntvoltage=shuntRaw/21;
        busvoltage=busvRaw/21;
        current_mA=currentmARaw/21;
        power_mW=power_mWRaw/21;
        shuntRaw=0,busvRaw=0,currentmARaw=0,power_mWRaw=0;
        Amp = current_mA;
        if (Amp < 0){
            Amp = 0;
        }
        current_A = Amp / 1000;        
        if (power_mW < 0){
            power_mW = 0;
        }
        power_W = power_mW / 1000;
        loadvoltage = busvoltage + (shuntvoltage / 1000);
        total_mA += Amp;
        total_mAH = total_mA/188; 
/*         if(Horas > 0){
            total_mAM = total_mAH / Horas;
        }
        else{
            total_mAM = total_mAH;
        } */
        inadone=true;
    }
}

//-VSYS VOLTAGE-----------------------------
int power_voltage(float *voltage_result) {
    cyw43_thread_enter();
    // setup adc
    adc_init();
    adc_gpio_init(29);
    adc_select_input(29 - PICO_FIRST_ADC_PIN);
    // read vsys
    uint32_t vsys = 0;
    for(int i = 0; i < PICO_POWER_SAMPLE_COUNT; i++) {
        vsys += adc_read();
    }
    vsys /= PICO_POWER_SAMPLE_COUNT;
    cyw43_thread_exit();
    // Generate voltage
    const float conversion_factor = 3.3f / (1 << 12);
    *voltage_result = vsys * 3 * conversion_factor;
    return PICO_OK;
}

//-GENERIC----------------------------------
long mapcustom(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//-NTP--------------------------------------
static void ntp_result(NTP_T* state, int status, time_t *result) {    
    if (status == 0 && result) {
        struct tm *utc = gmtime(result);        
        if(segundos != utc->tm_sec || minutos != utc->tm_min){
            segundos = utc->tm_sec;            
            minutos = utc->tm_min;
            dianum = utc->tm_mday;
            mes = utc->tm_mon + 1;
            año = 1900 + utc->tm_year;

            if (debugControlDia < 1) {
                diaint = utc->tm_wday;
                debugControlDia = 1;
            }
            if (mes == 13) { //???
                mes = 1;
            }
            if (mes >= 11 || mes < 4) {
                if (utc->tm_hour + 1 == 24) {
                    hora = 0;
                }
                else if (utc->tm_hour + 1 == 25) {
                    hora = 1;
                }
                else {
                    hora = utc->tm_hour + 1;
                }
            }
            else if (mes >= 4 && mes < 11) {
                if (utc->tm_hour + 2 == 24) {
                    hora = 0;
                }
                else if (utc->tm_hour + 2 == 25) {
                    hora = 1;
                }
                else {
                    hora = utc->tm_hour + 2;
                }
            }
            if(hora == 0 || hora == 1){
                if (movidaDebug == 0) {
                    movida = true;
                    movidaDebug = 1;
                }    
                if (movida == true) {
                    diaint += 1;
                    movida = false;
                } 
                dianum += 1; //Hay que tener en cuenta el mes para saber si dianum > 28 o 29 mes++.
                if(año%4==0){//Bisiesto Feb 29dias
                    if(mes==1||mes==3||mes==5||mes==7||mes==8||mes==10||mes==12){
                        if(dianum>31){
                            dianum=1;
                            mes++;
                            //if(mes>12)
                        }
                    }
                    else if(mes==4||mes==6||mes==9||mes==11){
                        if(dianum>30){
                            dianum=1;
                            mes++;
                        }
                    }
                    else if(mes==2){
                        if(dianum>29){
                            dianum=1;
                            mes++;
                        }
                    }
                }
                else{//NO bisiesto Feb 28dias
                    if(mes==1||mes==3||mes==5||mes==7||mes==8||mes==10||mes==12){
                        if(dianum>31){
                            dianum=1;
                            mes++;
                            //if(mes>12)
                        }
                    }
                    else if(mes==4||mes==6||mes==9||mes==11){
                        if(dianum>30){
                            dianum=1;
                            mes++;
                        }
                    }
                    else if(mes==2){
                        if(dianum>28){
                            dianum=1;
                            mes++;
                        }
                    }
                }
                if (diaint > 6) {
                    diaint = 0;
                }
            }               
            //printf("\n-NTP: %02i/%02i/%04i %02i:%02i:%02i\n", dianum, mes, utc->tm_year + 1900, hora, minutos, segundos);                          
            if(InitTimeGet != true && año != 1900){
                SegundoComienzo=segundos;
                MinutoComienzo=minutos;
                HoraComienzo=hora;
                DiaComienzo=dianum;
                MesComienzo=mes;
                InitTimeGet = true;
                printf("\n-INIT: %02i/%02i %02i:%02i:%02i\n\n",DiaComienzo,MesComienzo,HoraComienzo,MinutoComienzo,SegundoComienzo);
            }  
            setTimeRTC();
        }                 
    }
    else{        
        ntpupdated=false;
        ntpproceso=false;
    } 
    state->dns_request_sent = false;
}

static void ntp_request(){
    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    uint8_t *req = (uint8_t *) p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1b;
    udp_sendto(state->ntp_pcb, p, &state->ntp_server_address, NTP_PORT);
    pbuf_free(p);
    cyw43_arch_lwip_end();
}

static void ntp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg){
 NTP_T *state = (NTP_T*)arg;
    if (ipaddr) {
        state->ntp_server_address = *ipaddr;
        //printf("\n-NTP DNS address: %s\n", ipaddr_ntoa(ipaddr));
        ntp_request();
    } else {
        //printf("\n-NTP DNS request failed...Not found\n");
        ntp_result(state, -1, NULL);
    }
    ntpproceso=false;
}

static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    NTP_T *state = (NTP_T*)arg;
    uint8_t mode = pbuf_get_at(p, 0) & 0x7;
    uint8_t stratum = pbuf_get_at(p, 1);

    // Check the result
    if (ip_addr_cmp(addr, &state->ntp_server_address) && port == NTP_PORT && p->tot_len == NTP_MSG_LEN &&
        mode == 0x4 && stratum != 0) {
        uint8_t seconds_buf[4] = {0};
        pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
        uint32_t seconds_since_1900 = seconds_buf[0] << 24 | seconds_buf[1] << 16 | seconds_buf[2] << 8 | seconds_buf[3];
        uint32_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;
        time_t epoch = seconds_since_1970;
        ntp_result(state, 0, &epoch);
    } else {
        //printf("\ninvalid NTP response\n");
        ntp_result(state, -1, NULL);
    }
    pbuf_free(p);
}

static void NTPSetup(){
    if (!state) {
        printf("failed to allocate state\n");        
    }
    state->ntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!state->ntp_pcb) {
        printf("failed to create pcb\n");
        free(state);        
    }
    else{
        udp_recv(state->ntp_pcb, ntp_recv, state);
    } 
    rtc_init(); 
    printf("\n-NTP SETUP\n");  
}

void NTPLoop(){    
    if(Horas > ntpHoras+(uint64_t)(1)){
        ntpupdated=false;
        ntpHoras=Horas;        
    } 
    if(ntpupdated==false&&ntpproceso==false){ 
        ntpproceso=true;
        cyw43_arch_lwip_begin();
        int err = dns_gethostbyname(NTP_SERVER, &state->ntp_server_address, ntp_dns_found, state);
        cyw43_arch_lwip_end();
        //printf("\n-NTP DNS request sent\n");
        state->dns_request_sent = true;
        if (err == ERR_OK) {
            ntp_request(); // Cached result
        } else if (err == ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
            //printf("\n-NTP DNS request in progress\n");
            ntp_result(state, -1, NULL);
            sleep_ms(1000);
        }  
        else if(err != ERR_INPROGRESS && err != ERR_OK){
            //printf("\n-NTP DNS request ERROR\n");
            ntp_result(state, -1, NULL);
            ErrorNTP++;
            sleep_ms(5000);
        }       
    }
    RTC();
}
//-WIFI----------------------------------------------------

/* static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if(result){        
        memcpy(&network[netnum], result, sizeof(*result));
        netnum++;       
    }
    return 0;
} */

static void Wifi_Reconnect(){
    printf("\n\n---------------- RECONECTANDO ---------------\n");
    iReconexion++;    
    cyw43_arch_deinit();    
    if (cyw43_arch_init_with_country(CYW43_COUNTRY('E', 'S', 0)) != 0) {        
        printf("\n\nERROR cyw43------");
        Wifi_Reconnect();
    }
    else{
        //printf("\n-Inicializando cyw43: ");
    }
    if(!cyw43_is_initialized(&cyw43_state)){        
        printf("NO INICIALIZADO\n");
        Wifi_Reconnect();
    }
    else{
        //printf("inicializado\n");
        //extern cyw43_t cyw43_state;
    }
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_PM1_POWERSAVE_MODE, 20, 1, 1, 1));   
    cyw43_arch_enable_sta_mode();
    netif_set_hostname(netif_default, "PicoW_Meteo");

    /* printf("\n-----------Realizando scan de redes WiFi----------\n");
    cyw43_wifi_scan_options_t scan_options = {0};
    int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
    elapTime = time_us_64();
    while(cyw43_wifi_scan_active(&cyw43_state) == true){
        if (err != 0) {
            printf("\n-ERROR---scan redes WiFi: %d\n", err); 
            Wifi_Reconnect();  
        }        
        elapTimeend = time_us_64();         
        if(elapTimeend-elapTime > 10000000){ // 10 seg
            printf("\n-ERROR TIMEOUT---scan redes WiFi: %d\n", err);
            Wifi_Reconnect();            
        }
    }
    printf("\n-----------Encontradas %i redes -- elapTime: %lli elapTimeEnd: %lli Delta mS: %lli ----------\n", netnum,elapTime,elapTimeend,(elapTimeend-elapTime)/1000);     
    for(int i = 0; i < netnum; i++){                
        for(int j = 0; j < netnum; j++){
            if(network[i].bssid[0] == network[j].bssid[0]){                
                Duplic[i]++;                                                 
            }     
        }        
        if(network[i].bssid[0] != BSSIDRaw[i-1] && network[i].bssid[0] != BSSIDRaw[i-2] && network[i].bssid[0] != BSSIDRaw[i-3]&& network[i].bssid[0] != BSSIDRaw[i-4] && network[i].bssid[0] != BSSIDRaw[i-5]){
            printf("\nssid: %-19s rssi: %4d chan: %3d\nmac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\nDuplic: %i i: %i\n",
            network[i].ssid, network[i].rssi, network[i].channel,
            network[i].bssid[0], network[i].bssid[1], network[i].bssid[2], network[i].bssid[3], network[i].bssid[4], network[i].bssid[5],
            network[i].auth_mode,Duplic[i], i);            
        }      
        BSSIDRaw[i] = network[i].bssid[0];                                  
    }
    netnum = 0;     */

    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK) != 0) {
        printf("ERROR--Conexión WiFi Async--E: %i\n",cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK));
        sleep_ms(500);
        Wifi_Reconnect();
    } 
    else{
        //printf("\nConectando a %s...\n", WIFI_SSID); 
    }          
    while (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) != CYW43_LINK_UP){
        cyw43_arch_poll();
        sleep_ms(500);
        switch (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)){
            case CYW43_LINK_UP:
            {
                cyw43_wifi_get_rssi(&cyw43_state, &rssi);                   
                //cyw43_wifi_get_bssid(&cyw43_state, &bssid);                
                //printf("\nConectado.\n");  
                //printf("SSID: %s\n", WIFI_SSID);
                //printf("BSSID: %hhu\n", bssid);
                //printf("RSSI: %li\n", rssi);           

                //extern cyw43_t cyw43_state;
                //auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
                //printf("IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
                
                break;
            }
            case CYW43_LINK_DOWN:
                printf("\nCYW43_LINK_DOWN.\n");
                Wifi_Reconnect();
                break;    
            case CYW43_LINK_NOIP:
                printf("\nCYW43_LINK_NOIP.\n");
                break; 
            case CYW43_LINK_FAIL:
                printf("\nCYW43_LINK_FAIL.\n");
                Wifi_Reconnect();
                break;
            case CYW43_LINK_NONET:
                printf("\nCYW43_LINK_NONET.\n");
                Wifi_Reconnect();
                break;    
            case CYW43_LINK_BADAUTH:
                printf("\nCYW43_LINK_BADAUTH.\n");
                Wifi_Reconnect();
                break;       
            default:
                //printf("\nProcesando.\n");  
                break;
        };
    }
    ntpupdated=false;
    ntpproceso=false;        
    mqttdone=false;
    mqttproceso=false;
}

static void Wifi_Off(MQTT_CLIENT_T* stateM){
    mqtt_disconnect(stateM->mqtt_client);
    cyw43_arch_deinit();
    switch (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)){
        case CYW43_LINK_UP:
        {
            cyw43_wifi_get_rssi(&cyw43_state, &rssi);                   
            //cyw43_wifi_get_bssid(&cyw43_state, &bssid);                
            //printf("\nConectado.\n");  
            //printf("SSID: %s\n", WIFI_SSID);
            //printf("BSSID: %hhu\n", bssid);
            //printf("RSSI: %li\n", rssi);
            //extern cyw43_t cyw43_state;
            //auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
            //printf("IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);                    
            break;
        }
        case CYW43_LINK_DOWN:
            printf("\n-WiFi chip switched Off!\n\n");                   
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
            //printf("\nProcesando.\n");  
            break;
    };
}

/* static void checkWiFiStatus(){
    if(wifidone==false){                   
        cyw43_wifi_get_rssi(&cyw43_state, &rssi);
        dbpercent = mapcustom(rssi,-95,-20,0,100); 
        //extern cyw43_t cyw43_state;
        //auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;        
        printf("\nConectado.\n");  
        printf("SSID: %s\n", WIFI_SSID);
        printf("RSSI: %li -- %i% \n", rssi,dbpercent);
        printf("IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
        printf("I.Reconexion: %i\n",iReconexion);          
        wifidone=true;
    }       
} */

static void Wifi_Reconnect_Sleep(){ 
    //printf("\n---------------- RECONECTANDO ------------\n");    
    if (cyw43_arch_init_with_country(CYW43_COUNTRY('E', 'S', 0)) != 0) {        
        printf("\n\nERROR cyw43------");
        Wifi_Reconnect();
    }
    else{
        //printf("\n-Inicializando cyw43: ");
    }
    if(!cyw43_is_initialized(&cyw43_state)){        
        printf("--NO INICIALIZADO-----------\n");
        Wifi_Reconnect();
    }
    else{
        //printf("inicializado\n");
        //extern cyw43_t cyw43_state;
    }
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_PM1_POWERSAVE_MODE, 20, 1, 1, 1));
    //cyw43_wifi_pm(&cyw43_state ,CYW43_AGGRESSIVE_PM);
	//cyw43_wifi_pm(&cyw43_state ,CYW43_PERFORMANCE_PM);   
    cyw43_arch_enable_sta_mode();
    netif_set_hostname(netif_default, "PicoW_Meteo");

    //TEST--------------------------------------------------------------------
/*     printf("\n-----------Realizando scan de redes WiFi----------\n");
    cyw43_wifi_scan_options_t scan_options = {0};
    sleep_ms(500);
    int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
    elapTime = time_us_64();
    while(cyw43_wifi_scan_active(&cyw43_state) == true){
        if (err != 0) {
            printf("\n-ERROR---scan redes WiFi: %d\n", err); 
            Wifi_Reconnect();  
        }        
        elapTimeend = time_us_64();         
        if(elapTimeend-elapTime > 10000000){ // 10 seg
            printf("\n-ERROR TIMEOUT---scan redes WiFi: %d\n", err);
            Wifi_Reconnect();            
        }
    }
    printf("\n-----------Encontradas %i redes -- elapTime: %lli elapTimeEnd: %lli Delta mS: %lli ----------\n", netnum,elapTime,elapTimeend,(elapTimeend-elapTime)/1000);       
    for(int i = 0; i < netnum; i++){                
        for(int j = 0; j < netnum; j++){
            if(network[i].bssid[0] == network[j].bssid[0]){                
                Duplic[i]++;                                                 
            }     
        }        
        if(network[i].bssid[0] != BSSIDRaw[i-1] && network[i].bssid[0] != BSSIDRaw[i-2] && network[i].bssid[0] != BSSIDRaw[i-3]&& network[i].bssid[0] != BSSIDRaw[i-4] && network[i].bssid[0] != BSSIDRaw[i-5]){
            printf("\nssid: %-19s rssi: %4d chan: %3d\nmac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\nDuplic: %i i: %i\n",
            network[i].ssid, network[i].rssi, network[i].channel,
            network[i].bssid[0], network[i].bssid[1], network[i].bssid[2], network[i].bssid[3], network[i].bssid[4], network[i].bssid[5],
            network[i].auth_mode,Duplic[i], i);            
        }      
        BSSIDRaw[i] = network[i].bssid[0];                                  
    }
    netnum = 0; */
    //TEST--------------------------------------------------------------------

    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK) != 0) {
        printf("ERROR--Conexión WiFi Async--E: %i\n",cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK));
        sleep_ms(500);
        Wifi_Reconnect();
    } 
    else{
        //printf("\nConectando a %s...\n", WIFI_SSID); 
    }          
    while (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) != CYW43_LINK_UP){
        cyw43_arch_poll();
        sleep_ms(500);
        switch (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)){
            case CYW43_LINK_UP:
            {
                cyw43_wifi_get_rssi(&cyw43_state, &rssi); 
                    mqttdone=false;
                    mqttproceso = false;          
                    dhtdone=false;  
                    bmpdone=false;
                    ldrdone=false;     
                    Tempcombdone=false;  
                    inadone=false;     
                    serialdone=false;                   
                //cyw43_wifi_get_bssid(&cyw43_state, &bssid);                
                //printf("\nConectado.\n");  
                //printf("SSID: %s\n", WIFI_SSID);
                //printf("BSSID: %hhu\n", bssid);
                //printf("RSSI: %li\n", rssi);           

                //extern cyw43_t cyw43_state;
                //auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
                //printf("IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
                
                break;
            }
            case CYW43_LINK_DOWN:
                printf("\nCYW43_LINK_DOWN.\n");
                Wifi_Reconnect();
                break;    
            case CYW43_LINK_NOIP:
                //printf("\nCYW43_LINK_NOIP.\n");
                break; 
            case CYW43_LINK_FAIL:
                printf("\nCYW43_LINK_FAIL.\n");
                Wifi_Reconnect();
                break;
            case CYW43_LINK_NONET:
                printf("\nCYW43_LINK_NONET.\n");
                Wifi_Reconnect();
                break;    
            case CYW43_LINK_BADAUTH:
                printf("\nCYW43_LINK_BADAUTH.\n");
                Wifi_Reconnect();
                break;       
            default:
                //printf("\nProcesando.\n");  
                break;
        };
    }    
}

static void Wifi_Setup(){
    if (cyw43_arch_init_with_country(CYW43_COUNTRY('E', 'S', 0)) != 0) {        
        printf("\n\nERROR cyw43------");
        Wifi_Reconnect();
    }
    else{
        printf("\n-Inicializando cyw43: ");
    }
    if(!cyw43_is_initialized(&cyw43_state)){        
        printf("NO INICIALIZADO\n");
        Wifi_Reconnect();
    }
    else{
        printf("inicializado\n");
        //extern cyw43_t cyw43_state;
    }
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_PM1_POWERSAVE_MODE, 20, 1, 1, 1));   
    cyw43_arch_enable_sta_mode();
    netif_set_hostname(netif_default, "PicoW_Meteo");

    /* printf("\n-----------Realizando scan de redes WiFi----------\n");
    cyw43_wifi_scan_options_t scan_options = {0};
    sleep_ms(500);
    int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
    elapTime = time_us_64();
    while(cyw43_wifi_scan_active(&cyw43_state) == true){
        if (err != 0) {
            printf("\n-ERROR---scan redes WiFi: %d\n", err); 
            Wifi_Reconnect();  
        }        
        elapTimeend = time_us_64();         
        if(elapTimeend-elapTime > 10000000){ // 10 seg
            printf("\n-ERROR TIMEOUT---scan redes WiFi: %d\n", err);
            Wifi_Reconnect();            
        }
    }
    printf("\n-----------Encontradas %i redes -- elapTime: %lli elapTimeEnd: %lli Delta mS: %lli ----------\n", netnum,elapTime,elapTimeend,(elapTimeend-elapTime)/1000);       
    for(int i = 0; i < netnum; i++){                
        for(int j = 0; j < netnum; j++){
            if(network[i].bssid[0] == network[j].bssid[0]){                
                Duplic[i]++;                                                 
            }     
        }        
        if(network[i].bssid[0] != BSSIDRaw[i-1] && network[i].bssid[0] != BSSIDRaw[i-2] && network[i].bssid[0] != BSSIDRaw[i-3]&& network[i].bssid[0] != BSSIDRaw[i-4] && network[i].bssid[0] != BSSIDRaw[i-5]){
            printf("\nssid: %-19s rssi: %4d chan: %3d\nmac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\nDuplic: %i i: %i\n",
            network[i].ssid, network[i].rssi, network[i].channel,
            network[i].bssid[0], network[i].bssid[1], network[i].bssid[2], network[i].bssid[3], network[i].bssid[4], network[i].bssid[5],
            network[i].auth_mode,Duplic[i], i);            
        }      
        BSSIDRaw[i] = network[i].bssid[0];                                  
    }
    netnum = 0;  */  

    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK) != 0) {
        printf("ERROR--Conexión WiFi Async--E: %i\n",cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK));
        sleep_ms(500);
        Wifi_Reconnect();
    } 
    else{
        printf("\nConectando a: %s ...\n", WIFI_SSID); 
    }          
    while (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) != CYW43_LINK_UP){
        cyw43_arch_poll();
        sleep_ms(500);
        switch (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)){
            case CYW43_LINK_UP:
            {
                cyw43_wifi_get_rssi(&cyw43_state, &rssi);                   
                //cyw43_wifi_get_bssid(&cyw43_state, &bssid);                
                printf("\nConectado.\n");  
                printf("SSID: %s\n", WIFI_SSID);
                //printf("BSSID: %hhu\n", bssid);
                printf("RSSI: %li\n", rssi);         

                extern cyw43_t cyw43_state;
                auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
                printf("IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
                
                break;
            }
            case CYW43_LINK_DOWN:
                printf("\nCYW43_LINK_DOWN.\n");
                Wifi_Reconnect();
                break;    
            case CYW43_LINK_NOIP:
                printf("\nCYW43_LINK_NOIP.\n");
                break; 
            case CYW43_LINK_FAIL:
                printf("\nCYW43_LINK_FAIL.\n");
                Wifi_Reconnect();
                break;
            case CYW43_LINK_NONET:
                printf("\nCYW43_LINK_NONET.\n");
                Wifi_Reconnect();
                break;    
            case CYW43_LINK_BADAUTH:
                printf("\nCYW43_LINK_BADAUTH.\n");
                Wifi_Reconnect();
                break;       
            default:
                printf("\nProcesando.\n");  
                break;
        };
    }
}

//-SETUPINFO-----------------------------------
static void SetupInfo(){
    //ADC READS
/*     adc_init();
    adc_gpio_init(27);
    adc_select_input(1);
    adc_set_clkdiv(0);
    uint32_t start = time_us_64();
    int sum = adc_read();
    for (int i = 0; i < 10000; i++) {
        sum += adc_read();
    }
    uint32_t end = time_us_64();
    printf("\n--ADC reads per second: %0.0f", 10000.0 / ((end - start) * 0.000001));  */
    //CORE FREQ
    /* printf("\n--Clock info -SYS:%lu kHz // -PERI:%lu kHz\n", frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS), frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI)); */

    printf("\n-PICO_SDK_VERSION_STRING: %s\n", PICO_SDK_VERSION_STRING);
    printf("-rp2040_chip_version: %d\n", rp2040_chip_version());
    printf("-rp2040_rom_version: %d\n", rp2040_rom_version());
}

//-SERIALINFO-----------------------------------
static void serialInfo(bool old_voltage){
    if(serialdone==false){         
        //-TEMP
        adc_set_temp_sensor_enabled(true);
        // Get voltage
        voltage = 0;
        voltage_return = power_voltage(&voltage);
        voltage = floorf(voltage * 100) / 100;
        // Get power val if it's changed
        if (old_voltage != voltage) {
            if (voltage_return == PICO_OK) {
                //const float min_battery_volts = 3.0f;
                //const float max_battery_volts = 4.2f;
                //int percent_val = ((voltage - min_battery_volts) / (max_battery_volts - min_battery_volts)) * 100;
            }
            // Also get the temperature
            adc_select_input(4); 
            adc = 0;
            for (size_t i = 0; i < PICO_POWER_SAMPLE_COUNT; i++)
            {
                adc += (float)adc_read() * conversionFactor;
            }            
            tempC = 27.0f - ((adc/PICO_POWER_SAMPLE_COUNT) - 0.706f) / 0.001721f;
            old_voltage = voltage;            
        } 
        //-TEMP
        adc_set_temp_sensor_enabled(false);  
        serialdone=true;
    }
}

//-MEDICIONES ELECT.-------------------------------
void MedicionesElect(){ 
    LoopINA219();    
}

//-Mediciones Generales-------------------------
void MedGen() {
    dht22();
    bmp280();
    LDRLoop(); 
    if(Tempcombdone==false){
        Tcomb = ((temperature/100.f) + temperature_c)/2;
        Tempcombdone=true;
    } 
}

void mqttTimeout(MQTT_CLIENT_T *stateM){
    if(StartingSEC+(uint64_t)(200) <= Segundos){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(3000);
        printf("\n---TIMEOUT MQTT +%lli SECONDS---",Segundos-StartingSEC);
        printf("\n-STATUS---> \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-Tempcombdone:%d \n-inadone:%d \n-------",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,Tempcombdone,inadone);
        printf("\n---TRYING---");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(1000);
        Wifi_Off(stateM);  
        Wifi_Reconnect_Sleep();
        ntpupdated=false;
        ntpproceso=false;
        state->dns_request_sent = false;
        dhtdone=false;  
        bmpdone=false;
        ldrdone=false;     
        Tempcombdone=false;  
        inadone=false;     
        serialdone=false; 
        mqttdone=false;
        mqttproceso=false; 
        execTime();
        NTPLoop();
        serialInfo(old_voltage);
        MedicionesElect();
        MedGen();                           
        MQTT(stateM);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(1000);
        printf("\n-STATUS AFTER TRY---> \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-Tempcombdone:%d \n-inadone:%d \n-------\n",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,Tempcombdone,inadone);
    }
}

void Sleep(MQTT_CLIENT_T *stateM){
    if(ntpupdated==true && InitTimeGet==true && mqttdone==true && mqttproceso==false && dhtdone==true && bmpdone==true && ldrdone==true && Tempcombdone==true && inadone==true){    
        Wifi_Off(stateM);                
        //-SLEEPING--
        if(TESTING){
            sleep_ms(10000);
        }else{
            sleep_run_from_xosc();                                
            rtc_sleep_custom(0,180); //180sec sleep
            recover_from_sleep();
        } 
        //-SLEEP DONE--  
        Wifi_Reconnect_Sleep();
              
    } 
}

//-------------------------------------------CORE0--------------------------------------
int main() {    
    //-SYSTEM SETUP----------
/*     vreg_set_voltage(VREG_VOLTAGE_0_90);
    sleep_ms(10); 
    set_sys_clock_khz(SYS_KHZ, true); 
    sleep_ms(10);
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        SYS_KHZ * 1000,                                   // Input frequency
        SYS_KHZ * 1000                                    // Output (must be same as no divider)
    );  */
    sleep_ms(10);      
    stdio_init_all();    
    if(TESTING){
        while(!tud_cdc_connected()){sleep_ms(1);}//Wait to console ready
    }    
    sleep_ms(10);
    TiempoWait = time_us_64();
    printf("\n----------------- SERIAL CONECTADO ------------------\n");
    //-SETUP START------
    SetupInfo();
    Wifi_Setup();
    ina219Setup();     
    NTPSetup();    
    //-MQTT--------------
    MQTT_CLIENT_T *stateM = mqtt_client_init();     
    run_dns_lookup(stateM);
    stateM->mqtt_client = mqtt_client_new();
    stateM->counter = 0;
    if (stateM->mqtt_client == NULL) {
        printf("-Failed to create new mqtt client\n");        
    } 
    if (mqtt_test_connect(stateM) != ERR_OK) {
        printf("-Connection BAD!\n");
    } 
    mqttdone=false;
    //-BMP280---------------
    bmp280Setup();
    //-FREQS.--------
    measure_freqs();
    //-LED OFF-----
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);  
    //-END SETUP--------------  
    //-SETUPTIME---
    TiempoLoop = time_us_64()-TiempoWait;
    printf("\n-SETUP OK!  -T.Wait: %i -T.Setup: %i -T.Total: %i\n",TiempoWait/1000,TiempoLoop/1000,(TiempoWait+TiempoLoop)/1000);        
    printf("\n------------ INICIANDO -----------\n\n");
    //-LOOP-------------------
    while (true) {
        cyw43_arch_poll();
        while(cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) == CYW43_LINK_UP){
            cyw43_arch_poll();
            execTime();
            NTPLoop();
            serialInfo(old_voltage);
            MedicionesElect();
            MedGen();                         
            MQTT(stateM);
            Sleep(stateM);          
            mqttTimeout(stateM);            
        }
        printf("\n------- LOOP WIFI ERROR ----------\n");
        Wifi_Reconnect();
        state->dns_request_sent = false;
    }
    cyw43_arch_deinit();     
    return 0;
} 