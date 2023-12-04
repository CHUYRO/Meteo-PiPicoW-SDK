#include <stdio.h>
#include <math.h>

#include <pico/stdlib.h>
#include "pico/cyw43_arch.h"
#include "pico/sleep.h"

#include "lwipopts.h"
#include "lwip/init.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

#include "hardware/adc.h"
#include "hardware/vreg.h"
#include "hardware/rtc.h"
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"
#include "hardware/spi.h"

#include <ina219.h>
#include <dht/include/dht.h>
#include "bmp/BMP280.hpp"
#include "tusb.h"

#define DEBUG false 
#if DEBUG
  #define DEBUGBMP280 false
  #define DEBUGDHT22 false
  #define DEBUGMQTT false
#else
  #define DEBUGBMP280 false
  #define DEBUGDHT22 false
  #define DEBUGMQTT false
#endif

#define SLEEPTIME 180

#define UNDERCLOCK false
#if UNDERCLOCK 
  #define SYS_KHZ (60 * 1000)   
#else
  #define SYS_KHZ (125 * 1000)  
#endif

//---------- GLOBAL ----------

//-RTC-------------
unsigned int SegundosRTC = 0, MinutosRTC = 0, HorasRTC = 0, DiaRTC = 0, MesRTC = 0, DiaWRTC = 0, AñoRTC = 0;

//-LDR--------------
float ldrVal = -1, ldrValRaw = 0;

//-BMP280-----------
#define MOSI 19
#define SCK 18
#define CS 17
#define MISO 16
#define chip_id 0xD0
uint8_t datachipid = chip_id;
uint8_t chipID = 0;
double temperature = 0, pressure = 0;

//-DHT22-------------
static const dht_model_t DHT_MODEL = DHT22;
static const uint DATA_PIN = 15;
float humidity;
float temperature_c = 0, Tcomb = 0;
dht_t dht;

//-INA219-------------------------
float total_mA = 0, shuntvoltage = 0, busvoltage = 0, current_mA = 0, total_mAH = 0, power_mW = 0, loadvoltage = 0, current_A = 0, power_W = 0, total_mAM = 0;
float SHUNT_OHMS = 0.1;
float MAX_EXPECTED_AMPS = 3.2;    
INA219 i(SHUNT_OHMS, MAX_EXPECTED_AMPS);

//-VSYS----------------
float old_voltage = 0, voltage = 0, adc = 0, tempC = 0;
int voltage_return = 0;
const float conversionFactor = 3.3f / (1 << 12);
#ifndef PICO_POWER_SAMPLE_COUNT
#define PICO_POWER_SAMPLE_COUNT 5
#endif
#define PICO_FIRST_ADC_PIN 26 //Pin used for ADC 0

//-TIME------------------
uint64_t USec = 0, USecRaw = 0, Millis = 0, MillisRaw = 0, MillisTot = 0, Minutos = 0, MinutosRaw = 0, MinutosTot = 0, Segundos = 0, SegundosRaw = 0, SegundosTot = 0, Horas = 0, HorasRaw = 0, HorasTot = 0, debugControlDia = 0, Dias = 0, DiasTot = 0, ntpHoras = 0, StartingSEC = 0, SegundoRawDHT22 = 0, SegundosRawBMP = 0;
unsigned int TiempoWait = 0, TiempoLoop = 0, diaint = 0, dianum = 0, mes = 0, año = 0, MinutoComienzo = 0, HoraComienzo = 0, SegundoComienzo = 0, DiaComienzo = 0, MesComienzo = 0;
bool movida = false, InitTimeGet = false;
long int movidaDebug = 0, minutos = 0, hora = 0, segundos = 0, ErrorMQTT = 0, ErrorNTP = 0, Timeout = 0;

//-WIFI-----------------
const int numMaxNets=40;
cyw43_ev_scan_result_t network[numMaxNets];
long int rssi = 0;
int iReconexion = 0, dbpercent = 0, netnum = 0, lastStatus = 33;    
uint8_t bssidConex[6]; 
bool netFound = false;  

//-NTP------------------
typedef struct NTP_T_ {
  ip_addr_t ntp_server_address;
  bool dns_request_sent;
  struct udp_pcb *ntp_pcb;
  absolute_time_t ntp_test_time;
  alarm_id_t ntp_resend_alarm;
} NTP_T;
#define NTP_SERVER "0.es.pool.ntp.org" 
#define NTP_MSG_LEN 48
#define NTP_PORT 123 
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
datetime_t datetimeNTP = {   //not needed cast...try better way...
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
unsigned int scb_orig = 0, clock0_orig = 0, clock1_orig = 0;
bool mqttdone = false, dhtdone = false, ldrdone = false, bmpdone = false, inadone = false, serialdone = false, ntpupdated = false, mqttproceso = false, ntpproceso = false;

//-MQTT---------------
const char *ChannelID = "1713237";
#define MQTT_SERVER_HOST "mqtt3.thingspeak.com"
#define MQTT_SERVER_PORT 1883
typedef struct MQTT_CLIENT_T_ {
  ip_addr_t remote_addr;
  mqtt_client_t *mqtt_client;
  u32_t received;
  u32_t counter;
  u32_t reconnect;
} MQTT_CLIENT_T;

//----------------------------------- FUNC. START -------------------------------
//-INTERNAL TIME OF EXEC.--------------------------
void __no_inline_not_in_flash_func(execTime)(){     
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
//-TESTING RTC THINGS--
static bool __no_inline_not_in_flash_func(valid_datetimeLOL)(datetime_t *t) {
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
bool __no_inline_not_in_flash_func(rtc_set_datetimeLOL)(datetime_t *t) {
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

void __no_inline_not_in_flash_func(setTimeRTC)(){       
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
      #if DEBUG           
        printf("\n-RTC(set):%02i:%02i:%02i T:%02lli:%02lli:%03lli\n",datetimeNTP.hour,datetimeNTP.min,datetimeNTP.sec,MinutosTot,SegundosTot,MillisTot);
      #endif 
      ntpupdated=true; 
      ntpproceso=false;                           
    }
    else{
      printf("Horas BAD!\n");
    }                           
  }         
}

void __no_inline_not_in_flash_func(RTC)(){ 
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

void run_dns_lookup(MQTT_CLIENT_T *stateM) {
  printf("\n-Running mqtt DNS query for %s.\n", MQTT_SERVER_HOST);

  cyw43_arch_lwip_begin();
  err_t err = dns_gethostbyname(MQTT_SERVER_HOST, &(stateM->remote_addr), dns_found, stateM);
  cyw43_arch_lwip_end();

  if (err == ERR_ARG) {
    printf("-Failed to start mqtt DNS query\n");
    return;
  }

  if (err == ERR_OK) {
    printf("-no dns mqtt lookup needed");
    return;        
  }

  while (stateM->remote_addr.addr == 0 && Timeout <= 15000) {
    cyw43_arch_poll();
    Timeout++;  
    busy_wait_ms(1);        
  }
  Timeout = 0;
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
  if (status != 0) {ErrorMQTT++;} 
  #if DEBUGMQTT
    printf("-mqtt_connection_cb: err %d\n", status);
  #endif  
  mqttproceso = false;
}

void mqtt_pub_request_cb(void *arg, err_t err) {
  MQTT_CLIENT_T *stateM = (MQTT_CLIENT_T *)arg;
  #if DEBUGMQTT 
    printf("-mqtt_pub_request_cb: err %d\n", err);
  #endif
  if(err==ERR_OK){
    stateM->counter++; 
    #if DEBUGMQTT  
      printf("-MQTT Ok:%ld Error:%ld\n", stateM->counter,ErrorMQTT); 
    #endif
    mqttdone=true;
    StartingSEC=Segundos;
  }      
  stateM->received++;
  mqttproceso = false;
}

err_t mqtt_test_publish(MQTT_CLIENT_T *stateM){
  char buffer[128];
  err_t err;
  u8_t qos = 0;   //0, 1 or 2. ThingSpeak is 0.
  u8_t retain = 0;    
  sprintf(buffer, "field1=%.2f&field2=%.2f&field3=%.3f&field4=%.2f&field5=%.2f&field6=%.2f&field7=%f&field8=%.2f", total_mAH,humidity,pressure,ldrVal,Tcomb,loadvoltage,voltage,current_mA);
  const char *channels = "channels/";
  const char *chanNum = ChannelID;
  const char *publish = "/publish";
  char topicString[25];   
  strcpy(topicString, channels);
  strcat(topicString, chanNum);
  strcat(topicString, publish);
  mqttproceso = true;
  cyw43_arch_lwip_begin();
  err = mqtt_publish(stateM->mqtt_client, topicString, buffer, strlen(buffer), qos, retain, mqtt_pub_request_cb, stateM);
  cyw43_arch_lwip_end();
  #if DEBUGMQTT 
    printf("-mqtt_test_publish: err %d\n", err);
  #endif

  return err; 
}

err_t mqtt_test_connect(MQTT_CLIENT_T *stateM) {
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

  const struct mqtt_connect_client_info_t *client_info = &ci;
  mqttproceso = true;    
  cyw43_arch_lwip_begin();
  err = mqtt_client_connect(stateM->mqtt_client, &(stateM->remote_addr), MQTT_SERVER_PORT, mqtt_connection_cb, stateM, client_info);
  cyw43_arch_lwip_end();
  #if DEBUGMQTT 
    printf("\n-mqtt_test_connect: err %d\n", err);
  #endif

  return err;
}

void MQTT(MQTT_CLIENT_T* stateM){
  if(mqttdone == false && mqttproceso == false && dhtdone == true && bmpdone == true && ldrdone == true && inadone == true){ 
    if(mqtt_client_is_connected(stateM->mqtt_client)) {                
      cyw43_arch_lwip_begin(); 
      if(mqtt_test_publish(stateM) == ERR_OK){}else{printf("-Full ringbuffer or Publish error.\n");} 
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
  //try to reinit usb - NOT working... after sleep usb comm. fail
  //stdio_init_all();

  return;
}

//-Measure freqs.--------------------------------------
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
  printf("\n--pll_sys  = %dkHz\n", f_pll_sys);
  printf("--pll_usb  = %dkHz\n", f_pll_usb);
  printf("--rosc     = %dkHz\n", f_rosc);
  printf("--clk_sys  = %dkHz\n", f_clk_sys);
  printf("--clk_peri = %dkHz\n", f_clk_peri);
  printf("--clk_usb  = %dkHz\n", f_clk_usb);
  printf("--clk_adc  = %dkHz\n", f_clk_adc);
  printf("--clk_rtc  = %dkHz\n", f_clk_rtc); 
  // Can't measure clk_ref / xosc as it is the ref
}

//-LDR---------------------------------------
void __no_inline_not_in_flash_func(LDRLoop)() {
  if(ldrdone==false){
    adc_init();        
    adc_gpio_init(27);        
    adc_select_input(1);
    ldrValRaw = 0;
    for (int i = 0; i < 10; i++){
      ldrValRaw += 140 - adc_read();         
    }
    ldrVal = ldrValRaw / 10;
    if (ldrVal < 5) {
      ldrVal = 0.01;
    }
    ldrdone=true;
    //printf("\n-LDR: %f\n",ldrVal);
  }
}

//-DHT22------------------------------------
void __no_inline_not_in_flash_func(dht22)() {
  if(dhtdone==false && SegundoRawDHT22+(uint64_t)2 < Segundos){
    SegundoRawDHT22=Segundos;
    dht_init(&dht, DHT_MODEL, pio0, DATA_PIN, true);
    dht_start_measurement(&dht);
    dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);
    if (result == DHT_RESULT_OK) {
      #if DEBUGDHT22 
        printf("\n---- DHT22 DEBUG ----\n");
        printf("-Temperature: %.1f [C] \n-Hum: %.1f [%%]\n", temperature_c, humidity);
        printf("-------- END -------\n");
      #endif  
      dhtdone=true;
    }else if (result == DHT_RESULT_TIMEOUT) {
      printf("\n-DHT sensor not responding. Please check your wiring.\n");
    }else {
      assert(result == DHT_RESULT_BAD_CHECKSUM);
      printf("\n-Bad checksum");
    } 
    dht_deinit(&dht);       
  }
}

//-INA219------------------------------------------
void ina219Setup(){
  i.configure(RANGE_16V, GAIN_8_320MV, ADC_32SAMP, ADC_32SAMP);
}

void __no_inline_not_in_flash_func(LoopINA219)() {
  if(inadone==false){   
    i.wake();
    busvoltage = i.voltage();
    shuntvoltage = i.shunt_voltage();
    current_mA = i.current(); 
    power_mW = i.power();
    loadvoltage = i.supply_voltage();
    i.sleep(); 
    if (current_mA < 0){
      current_mA *=-1;
    } 
    current_A = current_mA / 1000;        
    if (power_mW < 0){
      power_mW *=-1;
    }
    power_W = power_mW / 1000;       
    total_mA += current_mA;
    total_mAH = total_mA/SLEEPTIME+8; 
    if(Horas > 0){
      total_mAM = total_mAH / Horas;
    }
    else{
      total_mAM = total_mAH;
    } 
    inadone=true;
  }
}
//-BMP280------------------------------------
void bmp280setup(BMP280::BMP280 &bmp280) {
  spi_write_blocking(spi0, &datachipid, 1);
  spi_read_blocking(spi0, 0, &chipID, 1);
  sleep_ms(10);
  if(!bmp280.setPowerMode(BMP280::PowerMode::Forced, true)){printf("\n-Error PowerMode\n");}else{printf("\n-Power mode: %i\n", bmp280.readPowerMode());}
  sleep_ms(10);
  //(Temperature oversampling x1, pressure oversampling x1 0x27->0b00100111)
  if(!bmp280.setRegister(0xF4, 0b00100111, true)){printf("\n-Error Register\n");} 
  sleep_ms(10);
  if(!bmp280.setOversampling(BMP280::Temperature, (uint8_t)1, true)){printf("\n-Error OVS TEMP\n");}{printf("-Temperature oversampling: %i\n", bmp280.readOversampling(BMP280::Type::Temperature));}
  sleep_ms(10);
  if(!bmp280.setOversampling(BMP280::Pressure, (uint8_t)1, true)){printf("\n-Error OVS PRESS\n");}else{printf("-Pressure oversampling: %i\n", bmp280.readOversampling(BMP280::Type::Pressure));}
  sleep_ms(10);
} 
void __no_inline_not_in_flash_func(bmp280loop)(BMP280::BMP280 &bmp280) {
  if(bmpdone==false && SegundosRawBMP + (uint64_t)1 < Segundos){
    if(bmp280.readForChipID()!=0){
      bmpdone=true;
    }
    if(!bmp280.setPowerMode(BMP280::PowerMode::Forced, true)){printf("\n-Error PowerMode\n");bmpdone=false;}
    if(bmp280.readOversampling(BMP280::Type::Temperature) != 1){printf("\n-Error OVS TEMP\n");bmpdone=false;bmp280.setRegister(0xF4, 0b00100111, false);}
    if(bmp280.readOversampling(BMP280::Type::Pressure) != 1){printf("\n-Error OVS PRESS\n");bmpdone=false;bmp280.setRegister(0xF4, 0b00100111, false);}
    SegundosRawBMP = Segundos;
    temperature = bmp280.readTemperature();
    pressure = bmp280.readPressure(1);
    #if DEBUGBMP280 
      printf("\n------ BMP280 DEBUG ------\n");
      printf("-ChipID: %#x\n", bmp280.readForChipID());
      printf("-Power mode: %i\n", bmp280.readPowerMode());
      printf("-Temperature oversampling: %i\n", bmp280.readOversampling(BMP280::Type::Temperature));
      printf("-Temperature: %.2f[C]\n", temperature);
      printf("-Pressure oversampling: %i\n", bmp280.readOversampling(BMP280::Type::Pressure));
      printf("-Pressure: %.2f[hPa]\n", pressure);
      printf("---------- END ----------\n");
    #endif    
  }
} 

void __no_inline_not_in_flash_func(MedGen)(BMP280::BMP280 &bmp280){
  LDRLoop();      
  bmp280loop(bmp280);
  dht22();
  Tcomb = (temperature + temperature_c)/2;
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
      execTime();
      #if DEBUG              
        printf("\n-NTP: %02i/%02i/%04i %02i:%02i:%02i T:%02lli:%02lli:%03lli\n",dianum, mes, utc->tm_year + 1900, hora, minutos, segundos, MinutosTot, SegundosTot, MillisTot);  
      #endif                        
      if(InitTimeGet != true && año != 1900){
        SegundoComienzo=segundos;
        MinutoComienzo=minutos;
        HoraComienzo=hora;
        DiaComienzo=dianum;
        MesComienzo=mes;
        InitTimeGet = true;
        printf("\n-FECHA INIT.: %02i/%02i %02i:%02i:%02i\n",DiaComienzo,MesComienzo,HoraComienzo,MinutoComienzo,SegundoComienzo);
      }  
      setTimeRTC();
    }                 
  }
  else if(status != 0 && !result && ntpproceso == false){        
    ntpupdated=false;
    ErrorNTP++;
  } 
  state->dns_request_sent = false;
}

static void ntp_request(){
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
    ErrorNTP++;     
  }
  else{
    udp_recv(state->ntp_pcb, ntp_recv, state);
  } 
  rtc_init(); 
  printf("\n-NTP SETUP\n");  
}

void __no_inline_not_in_flash_func(NTPLoop)(){    
  if(Horas >= ntpHoras+(uint64_t)(3)){
    ntpupdated = false;
    ntpproceso = false;
    ntpHoras = Horas;        
  } 
  if(ntpupdated == false && ntpproceso == false){ 
    ntpproceso = true;
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(NTP_SERVER, &state->ntp_server_address, ntp_dns_found, state);
    cyw43_arch_lwip_end();
    //printf("\n-NTP DNS request sent\n");
    state->dns_request_sent = true;
    if(err == ERR_OK) {
      //printf("\n-NTP DNS Cached result: err %i\n",err);
      ntp_request(); // Cached result
    }else if (err == ERR_INPROGRESS || err == ERR_OK || err == ERR_ALREADY || err == ERR_ISCONN) { //expect a callback
      //printf("\n-NTP DNS request expect callback/wait: err %i\n",err);
    }else if(err != ERR_INPROGRESS && err != ERR_OK && err != ERR_ALREADY && err != ERR_ISCONN){
      //printf("\n-NTP DNS request ERROR!: err %i\n",err);
      ntp_result(state, -1, NULL);
    }
  }
  RTC();
}
//-WIFI----------------------------------------------------

static int __no_inline_not_in_flash_func(scan_result)(void *env, const cyw43_ev_scan_result_t *result) {
  if(result){     
    network[netnum] = *result;    
    netnum++;        
  }
  return 0;
}  

static void __no_inline_not_in_flash_func(wifi_Conn)(bool Reconnect){
  if(Reconnect==true){
    printf("\n---------------- RECONECTANDO ---------------\n");
    iReconexion++;   
    cyw43_arch_deinit();
  }      
  if (cyw43_arch_init_with_country(CYW43_COUNTRY('E', 'S', 0)) != 0) {        
    printf("\n\nERROR cyw43------");    
    wifi_Conn(true);
  }
  if(!cyw43_is_initialized(&cyw43_state)){       
    printf("--NO INICIALIZADO-----------\n");
    wifi_Conn(true);
  }
  cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_PM1_POWERSAVE_MODE, 20, 1, 1, 1));   
  cyw43_arch_enable_sta_mode();
  netif_set_hostname(netif_default, "PicoW_Meteo");
  #if DEBUG
    uint64_t elapTime = 0, elapTimeend = 0, elapTimeendRaw = 0;
    elapTime = time_us_64();
    cyw43_wifi_scan_options_t scan_options = {0};  
    int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
    if(err == 0){printf("\n-Escaneando redes WiFi");}
    else{printf("-ERROR---Scan redes WiFi: %d\n", err);wifi_Conn(true);}  
    while(cyw43_wifi_scan_active(&cyw43_state) == true){      
      elapTimeend = time_us_64();   
      if(elapTimeendRaw + 200000 < elapTimeend){
        printf(".");
        elapTimeendRaw=elapTimeend;
      }      
      if(elapTimeend-elapTime > 5000000){ //5 seg timeout
        printf("-ERROR TIMEOUT---Scan redes WiFi err: %d\n", err);
        wifi_Conn(true);            
      }
      cyw43_arch_poll();
      sleep_ms(1);
    } 
    printf("\n");
    uint8_t bssid[6],BSSIDRaw[netnum][6],AUTHRaw[netnum],*SSIDRaw[netnum]; 
    uint16_t CHANNELRaw[netnum];
    int16_t RSSIRaw[netnum];
    int Duplic[netnum],totNets=0; 
    for(int x = 0; x < netnum; x++){Duplic[x]=0;AUTHRaw[x]=0;SSIDRaw[x]=0;CHANNELRaw[x]=0;RSSIRaw[x]=0;for(int y = 0; y < 6; y++){BSSIDRaw[x][y]=0;}}      
    for(int i = 0; i < netnum; i++){                
      for(int j = 0; j < netnum; j++){
        if(network[i].bssid[0] == network[j].bssid[0] && network[i].bssid[1] == network[j].bssid[1] && network[i].bssid[2] == network[j].bssid[2] && network[i].bssid[3] == network[j].bssid[3] && network[i].bssid[4] == network[j].bssid[4] && network[i].bssid[5] == network[j].bssid[5]){    
          Duplic[i]++;      
          if(network[i].bssid[0] != BSSIDRaw[j][0] || network[i].bssid[1] != BSSIDRaw[j][1] || network[i].bssid[2] != BSSIDRaw[j][2] || network[i].bssid[3] != BSSIDRaw[j][3] || network[i].bssid[4] != BSSIDRaw[j][4] || network[i].bssid[5] != BSSIDRaw[j][5]){       
            if(Duplic[i]==1 && network[i].rssi != 0 && network[i].channel != 0){           
              AUTHRaw[i] = network[i].auth_mode;
              RSSIRaw[i] = network[i].rssi;
              CHANNELRaw[i] = network[i].channel;
              SSIDRaw[i] = static_cast<uint8_t*>(network[i].ssid);
              BSSIDRaw[i][0] = network[i].bssid[0];  
              BSSIDRaw[i][1] = network[i].bssid[1];     
              BSSIDRaw[i][2] = network[i].bssid[2]; 
              BSSIDRaw[i][3] = network[i].bssid[3];  
              BSSIDRaw[i][4] = network[i].bssid[4];
              BSSIDRaw[i][5] = network[i].bssid[5]; 
              totNets++;            
              if(*SSIDRaw[i] == 0){
                SSIDRaw[i]=(uint8_t*)"NO_SSID";   
              }
              if(strcmp(WIFI_SSID,(const char *)SSIDRaw[i])==0){netFound=true;}
              printf("\n-ssid: %s -rssi:%d -ch:%d\n-mac: %lu:%lu:%lu:%lu:%lu:%lu -sec:%u\n",SSIDRaw[i],RSSIRaw[i], CHANNELRaw[i], BSSIDRaw[i][0], BSSIDRaw[i][1], BSSIDRaw[i][2], BSSIDRaw[i][3], BSSIDRaw[i][4], BSSIDRaw[i][5],AUTHRaw[i]);
            }         
          }                                                 
        }       
      }         
    }  
    elapTimeend = time_us_64();   
    netnum=0;
  #else
    netFound=true;  
  #endif
  if(netFound==true){
    #if DEBUG
      printf("\n-%s encontrada! -- %i redes totales -- ScanTime: %lli ms\n",WIFI_SSID,totNets,(elapTimeend-elapTime)/1000);
    #endif
    if(cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK) != 0) {
      printf("-ERROR--Conexión WiFi Async--E: %i\n",cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK));
      wifi_Conn(true);
    }else{
      #if DEBUG
        printf("\n-Conectando a %s...\n", WIFI_SSID);
      #endif
    }          
    while (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) != CYW43_LINK_UP){
      cyw43_arch_poll();
      sleep_ms(1);
      if (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) != lastStatus){
        lastStatus = cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA);
        switch (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)){      
          case CYW43_LINK_UP:
          { 
            #if DEBUG 
              cyw43_wifi_get_rssi(&cyw43_state, &rssi);                   
              cyw43_wifi_get_bssid(&cyw43_state, bssidConex);                
              if(Reconnect==true){printf("\n-Reconectado.\n");}else{printf("\n-Conectado.\n");}  
              printf("--SSID: %s\n", WIFI_SSID);
              printf("--BSSID: %lu:%lu:%lu:%lu:%lu:%lu\n", bssidConex[0],bssidConex[1],bssidConex[2],bssidConex[3],bssidConex[4],bssidConex[5]);
              printf("--RSSI: %li\n", rssi);  
              auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
              printf("--IP: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
            #endif               
            mqttdone=false;
            mqttproceso=false;
            ntpproceso = false;
            state->dns_request_sent = false;
            dhtdone=false;  
            bmpdone=false;
            ldrdone=false;   
            inadone=false;     
            serialdone=false; 
            break;
          }
          case CYW43_LINK_DOWN:
            #if DEBUG 
              printf("\n-CYW43_LINK_DOWN.\n");
            #endif
            wifi_Conn(true);
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
            wifi_Conn(true);
            break;
          case CYW43_LINK_NONET:
            #if DEBUG 
              printf("\n-CYW43_LINK_NONET.\n");
            #endif
            wifi_Conn(true);
            break;    
          case CYW43_LINK_BADAUTH:
            #if DEBUG 
              printf("\n-CYW43_LINK_BADAUTH.\n");
            #endif
            wifi_Conn(true);
            break;       
          default: 
            break;
        };
      }
    }
  }else{printf("\n-%s NO encontrada!\n", WIFI_SSID);netFound=false;wifi_Conn(true);}  
}

static void __no_inline_not_in_flash_func(Wifi_Off)(MQTT_CLIENT_T* stateM){
  mqtt_disconnect(stateM->mqtt_client);
  cyw43_arch_deinit();
  switch (cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)){
  case CYW43_LINK_UP:
  { 
    printf("\n--------ERROR no deberia estar conectado.-----------\n"); 
    cyw43_wifi_get_rssi(&cyw43_state, &rssi);               
    printf("\nConectado.\n");  
    printf("SSID: %s\n", WIFI_SSID);
    printf("RSSI: %li\n", rssi);
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
//-SETUPINFO-----------------------------------
static void SetupInfo(){
  //ADC READS
  adc_init();
  adc_gpio_init(27);
  adc_select_input(1);
  adc_set_clkdiv(0);
  uint32_t start = time_us_64();
  int sum = adc_read();
  for (int i = 0; i < 10000; i++) {
  sum += adc_read();
  }
  uint32_t end = time_us_64();
  printf("\n--ADC reads per second: %0.0f\n", 10000.0 / ((end - start) * 0.000001));  

  printf("\n-PICO_SDK_VERSION_STRING: %s\n", PICO_SDK_VERSION_STRING);
  printf("-rp2040_chip_version: %d\n", rp2040_chip_version());
  printf("-rp2040_rom_version: %d\n", rp2040_rom_version());
  measure_freqs();
}

//-SERIALINFO-----------------------------------
int __no_inline_not_in_flash_func(power_voltage)(float *voltage_result) {
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

static void __no_inline_not_in_flash_func(serialInfo)(bool old_voltage){
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
      for (size_t i = 0; i < PICO_POWER_SAMPLE_COUNT; i++){
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

//-TIMEOUT CONTROL-------------------
void __no_inline_not_in_flash_func(TimeoutControl)(MQTT_CLIENT_T *stateM){
  execTime();
  if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == false && mqttdone == false && dhtdone == true && bmpdone == true && ldrdone == true && inadone == true && ntpupdated == true && ntpproceso == false){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("\n--- TIMEOUT MQTT +%lli SECONDS, sysSeg: %lli lastSendSeg: %lli ---",Segundos-StartingSEC,Segundos,StartingSEC);
    printf("\n-NTP OK. Sensors status OK. MQTT BAD: \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-inadone:%d \n-------\n",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,inadone);
    dhtdone=false;  
    bmpdone=false;
    ldrdone=false;  
    inadone=false;     
    serialdone=false; 
    mqttdone=false;
    mqttproceso=false;
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(3000);
  }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == false && mqttdone == true && (dhtdone != true || bmpdone != true || ldrdone != true || inadone != true) && ntpupdated == true && ntpproceso == false){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("\n--- TIMEOUT SENSORS +%lli SECONDS, sysSeg: %lli mqttlastSendSeg: %lli ---",Segundos-StartingSEC,Segundos,StartingSEC);
    printf("\n-NTP OK. MQTT OK. Sensors status BAD: \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-inadone:%d \n-------\n",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,inadone);   
    sleep_ms(300); 
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(60000);
  }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == false && mqttdone == false && (dhtdone != true || bmpdone != true || ldrdone != true || inadone != true) && ntpupdated == true && ntpproceso == false){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("\n--- TIMEOUT MQTT+SENSORS +%lli SECONDS, sysSeg: %lli mqttlastSendSeg: %lli ---",Segundos-StartingSEC,Segundos,StartingSEC);
    printf("\n-NTP OK. MQTT status BAD. Sensors status BAD: \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-inadone:%d \n-------\n",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,inadone); 
    dhtdone=false;  
    bmpdone=false;
    ldrdone=false;  
    inadone=false;     
    serialdone=false; 
    mqttdone=false;
    mqttproceso=false;   
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(60000);
  }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == false && mqttdone == true && dhtdone == true && bmpdone == true && ldrdone == true && inadone == true && ntpupdated == false && ntpproceso == false){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("\n--- TIMEOUT NTP +%lli SECONDS, sysSeg: %lli mqttlastSendSeg: %lli ---",Segundos-StartingSEC,Segundos,StartingSEC);
    printf("\n-MQTT OK. Sensors OK. NTP status BAD: \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-inadone:%d \n-------\n",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,inadone); 
    ntpproceso=false;
    ntpupdated=false;
    state->dns_request_sent = false;
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(60000);
  }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == false && mqttdone == false && (dhtdone != true || bmpdone != true || ldrdone != true || inadone != true) && ntpupdated == false && ntpproceso == false){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("\n--- TIMEOUT MQTT+SENSORS+NTP +%lli SECONDS, sysSeg: %lli mqttlastSendSeg: %lli ---",Segundos-StartingSEC,Segundos,StartingSEC);
    printf("\n-NTP status BAD. MQTT status BAD. Sensors status BAD: \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-inadone:%d \n-------\n",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,inadone); 
    ntpupdated=false;
    ntpproceso=false;
    state->dns_request_sent = false;
    dhtdone=false;  
    bmpdone=false;
    ldrdone=false;  
    inadone=false;     
    serialdone=false; 
    mqttdone=false;
    mqttproceso=false;  
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(60000);
  }else if(StartingSEC+(uint64_t)(SLEEPTIME+15) <= Segundos && ntpproceso == true){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("\n--- TIMEOUT NTPproceso(only) +%lli SECONDS, sysSeg: %lli mqttlastSendSeg: %lli ---",Segundos-StartingSEC,Segundos,StartingSEC);
    printf("\n-NTPproceso status BAD: \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-inadone:%d \n-------\n",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,inadone);
    ntpproceso=false;
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    StartingSEC=Segundos;
  }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == true){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("\n--- TIMEOUT MQTTproceso(ONLY) +%lli SECONDS, sysSeg: %lli lastSendSeg: %lli ---",Segundos-StartingSEC,Segundos,StartingSEC);
    printf("\n-MQTTproceso status BAD: \n-ntpupdated:%d ntpproceso:%d InitTimeGet:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-inadone:%d \n-------\n",ntpupdated,ntpproceso,InitTimeGet,mqttdone,mqttproceso,dhtdone,bmpdone,ldrdone,inadone);
    dhtdone=false;  
    bmpdone=false;
    ldrdone=false;  
    inadone=false;     
    serialdone=false; 
    mqttdone=false;
    mqttproceso=false;    
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(300);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    StartingSEC=Segundos;
  }
}

//-SLEEP-------------------------------------
void __no_inline_not_in_flash_func(Sleep)(MQTT_CLIENT_T *stateM){
  if(ntpupdated==true && InitTimeGet==true && mqttdone==true && mqttproceso==false && dhtdone==true && bmpdone==true && ldrdone==true && inadone==true){    
    Wifi_Off(stateM);                
    //-SLEEPING--
    #if DEBUG 
      printf("-Sleeping...\n");
      sleep_ms(SLEEPTIME*1000);
      printf("-Awaking...\n");
    #else
      sleep_run_from_xosc();                                
      rtc_sleep_custom(0,SLEEPTIME); //180sec sleep
      recover_from_sleep();
    #endif
    //-SLEEP DONE--  
    #if DEBUG  
      printf("-WiFi chip switched ON!\n");  
    #endif 
    wifi_Conn(false);        
  } 
}
//-------------- END FUNC. --------------------
int __no_inline_not_in_flash_func(main)() {    
  //- SYSTEM SETUP -----------
  //vreg_set_voltage(VREG_VOLTAGE_0_90);
  sleep_ms(10); 
  set_sys_clock_khz(SYS_KHZ, true); 
  sleep_ms(10);
  clock_configure(
  clk_peri,
  0,                                                // No glitchless mux
  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
  SYS_KHZ * 1000,                                   // Input frequency
  SYS_KHZ * 1000                                    // Output (must be same as no divider)
  );  
  sleep_ms(10);    
  stdio_init_all(); 
  sleep_ms(10);
  #if DEBUG 
    while(!tud_cdc_connected()){sleep_ms(1);}//Wait to console ready
    sleep_ms(50);
  #endif   
  TiempoWait = time_us_64();
  //- SETUP START ------------
  printf("\n------------ SETUP -------------\n");
  SetupInfo();
  wifi_Conn(false);
  ina219Setup();     
  NTPSetup();    
  //- MQTT --------------
  MQTT_CLIENT_T *stateM = mqtt_client_init();     
  run_dns_lookup(stateM);
  stateM->mqtt_client = mqtt_client_new();
  stateM->counter = 0;
  if (stateM->mqtt_client == NULL) {
    printf("-Failed to create new mqtt client\n");        
  }   
  //- BMP280 ------------   
  printf("\n-BMP280 SETUP"); 
  //Baudrate 0.5Mhz - couldn't find in datasheet
  spi_init(spi0, 500000);
  //SPI acceptable 00 and 11 configuration
  spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, (spi_order_t)0);
  //Mapping GPIO
  gpio_set_function(MOSI, GPIO_FUNC_SPI);
  gpio_set_function(MISO, GPIO_FUNC_SPI);
  gpio_set_function(SCK, GPIO_FUNC_SPI);  
  BMP280::BMP280 bmp280 = BMP280::BMP280(spi0, CS);
  bmp280setup(bmp280);   
  //- ONBOARD LED OFF -----------
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  //- SETUPTIME ---------
  TiempoLoop = time_us_64()-TiempoWait;
  printf("\n-SETUP OK! -T.Wait: %ims -T.Setup: %ims -T.Total: %ims\n",TiempoWait/1000,TiempoLoop/1000,(TiempoWait+TiempoLoop)/1000);    
  //- END SETUP --------------     
  printf("\n------------ LOOP -------------\n");   
  //- START LOOP -------------
  while (true) {
    execTime();
    cyw43_arch_poll();    
    while(cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) == CYW43_LINK_UP){    
      cyw43_arch_poll(); 
      execTime();     
      NTPLoop();
      serialInfo(old_voltage);
      LoopINA219();      
      MedGen(bmp280);                     
      MQTT(stateM);
      TimeoutControl(stateM);
      Sleep(stateM);      
    }
    printf("\n------- LOOP WIFI ERROR -------\n");
    wifi_Conn(true);    
  }
  cyw43_arch_deinit();     
  return 0;
} 