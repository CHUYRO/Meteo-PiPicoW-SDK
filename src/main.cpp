#include <stdio.h>
#include <math.h>

#include <pico/stdlib.h>
#include "pico/cyw43_arch.h"
#include "pico/sleep.h"

#include "lwip/apps/mqtt.h"

#include "hardware/adc.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"

#include <ina219.hpp>
#include <dht22.h>
#include <bmp280.hpp>
#include <wifiLib.hpp>
#include <telnetdLib.hpp>  

#define DEBUG false // debug logic

#if DEBUG
  #define DEBUGBMP280 false
  #define DEBUGDHT22 false
  #define DEBUGMQTT false
  #define DEBUGSERIALWAIT false
  #define DEBUGINA219 false
  #define DEBUGTELNET false
  #define SLEEPTIME 10  
#else
  #define DEBUGBMP280 false
  #define DEBUGDHT22 false
  #define DEBUGMQTT false
  #define DEBUGINA219 false
  #define DEBUGSERIALWAIT false
  #define DEBUGTELNET false
  #define SLEEPTIME 10
#endif

#define UNDERCLOCK false // over/underclock logic

#if UNDERCLOCK 
  #define SYS_KHZ (60 * 1000)   
#else
  #define SYS_KHZ (125 * 1000)  
#endif

//---------- GLOBAL ---------- 

//- TELNET SERIAL ----
telnetdLib::telnetdLib telnetd = telnetdLib::telnetdLib();

//- LDR --------------
float ldrVal = -1, ldrValRaw = 0;

//- BMP280 -----------
#define MOSI 19
#define SCK 18
#define CS 17
#define MISO 16
#define chip_id 0xD0
uint8_t datachipid = chip_id;
uint8_t chipID = 0;
double temperature = 0, pressure = 0;
BMP280::BMP280 bmp280 = BMP280::BMP280(spi0, CS);

//- DHT22 -------------
static const dht_model_t DHT_MODEL = DHT22;
static const uint DATA_PIN = 15;
float humidity;
float temperature_c = 0, Tcomb = 0;
dht_t dht;

//- INA219 -------------------------
float shuntvoltage = 0, busvoltage = 0, current_mA = 0, total_mAH = 0, power_mW = 0, loadvoltage = 0, current_A = 0, power_W = 0, total_mAM = 0, total_mA = 0, total_mW = 0, total_mWH = 0, total_mWM = 0;
float SHUNT_OHMS = 0.1;
float MAX_EXPECTED_AMPS = 3.2;  
INA219 i(SHUNT_OHMS, MAX_EXPECTED_AMPS); 
absolute_time_t INA219sendTimer = nil_time;

//- VSYS ----------------
float voltage = 0, adc = 0, tempC = 0;
int voltage_return = 0;
const float conversionFactor = 3.3f / (1 << 12);
#ifndef PICO_POWER_SAMPLE_COUNT
#define PICO_POWER_SAMPLE_COUNT 5
#endif
#define PICO_FIRST_ADC_PIN 26 //Pin used for ADC 0

//- TIME ------------------
uint64_t USec = 0, USecRaw = 0, Millis = 0, MillisRaw = 0, MillisTot = 0, Minutos = 0, MinutosRaw = 0, MinutosTot = 0, Segundos = 0, SegundosRaw = 0, SegundosTot = 0, Horas = 0, HorasRaw = 0, HorasTot = 0, debugControlDia = 0, Dias = 0, DiasTot = 0, StartingSEC = 0, SegundoRawDHT22 = 0, SegundosRawBMP = 0;
unsigned int TiempoWait = 0, TiempoLoop = 0;
long int ErrorMQTT = 0, Timeout = 0;
absolute_time_t timeoutControl_timer = nil_time;

//- WIFI -----------------
wifiLib::wifiLib Wifi = wifiLib::wifiLib(40);

//- SLEEP ------------------------------
unsigned int scb_orig = 0, clock0_orig = 0, clock1_orig = 0;
bool mqttdone = false, dhtdone = false, ldrdone = false, bmpdone = false, inadone = false, serialdone = false, mqttproceso = false;
uint64_t exactSleepTime = 0, exactSleepTimeFirst = 0, exactOnTimeFirst = 0, exactOnTime = 0, exactOnTimeCalc = 0;

//- MQTT ---------------
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
//- INTERNAL TIME OF EXEC. --------------------------
void __no_inline_not_in_flash_func(execTime)(){ 
  USec = time_us_64() - (TiempoLoop + TiempoWait);
  Millis = USec / 1000;
  Segundos = Millis / 1000;
  Minutos = Segundos / 60;
  Horas = Minutos / 60;
  Dias = Horas / 24;

  MillisTot = Millis - Segundos * 1000;
  SegundosTot = Segundos - Minutos * 60;
  MinutosTot = Minutos - Horas * 60;
  HorasTot = Horas - Dias * 24;
  DiasTot = Dias;

  if(MillisTot % 1000 == 0 && MillisTot != MillisRaw){
    MillisTot = 0;
    MillisRaw = MillisTot;        
  } 
  if(SegundosTot % 60 == 0 && SegundosTot != SegundosRaw){
    SegundosTot = 0;
    SegundosRaw = SegundosTot;        
  }           
  if(MinutosTot % 60 == 0 && MinutosTot != MinutosRaw){
    MinutosTot = 0;
    MinutosRaw = MinutosTot;
  }
  if(HorasTot % 24 == 0 && HorasTot != HorasRaw){ 
    HorasTot = 0;
    HorasRaw = HorasTot;
  } 
}
//- Measure freqs. --------------------------------------
void measure_freqs() {
  uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
  uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
  uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
  uint f_xosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_XOSC_CLKSRC);
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
  printf("--xosc     = %dkHz\n", f_xosc);
  printf("--clk_sys  = %dkHz\n", f_clk_sys);
  printf("--clk_peri = %dkHz\n", f_clk_peri);
  printf("--clk_usb  = %dkHz\n", f_clk_usb);
  printf("--clk_adc  = %dkHz\n", f_clk_adc);
  printf("--clk_rtc  = %dkHz\n", f_clk_rtc); 
  // Can't measure clk_ref / xosc as it is the ref
}
//- MQTT ---------------------------------------------------------
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
    #if PICO_CYW43_ARCH_POLL
    cyw43_arch_poll();
    #endif 
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
  sprintf(buffer, "field1=%.2f&field2=%.2f&field3=%.3f&field4=%.2f&field5=%.2f&field6=%.2f&field7=%f&field8=%.2f", total_mAM,humidity,pressure,ldrVal,Tcomb,loadvoltage,voltage,current_mA);
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
    if(mqtt_client_is_connected(stateM->mqtt_client)){                
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
//- SLEEP -------------------------------------
static void sleep_callback(void) {
  //printf("RTC woke us up\n"); 
  return;   
}
static void rtc_sleep_custom(uint minute_to_sleep_to, uint second_to_sleep_to) {  
  Wifi.NTPLoop();
  uint secondTO = second_to_sleep_to + Wifi.SegundosRTC;
  uint minuteTO = minute_to_sleep_to + Wifi.MinutosRTC;
  uint horaTO = Wifi.HorasRTC;
  uint diaTO = Wifi.DiaRTC;
  uint diaWTO = Wifi.DiaWRTC;
  uint mesTO = Wifi.MesRTC;
  uint añoTO = Wifi.AñoRTC;

  minuteTO += secondTO / 60;
  secondTO %= 60;

  horaTO += minuteTO / 60;
  minuteTO %= 60;

  diaTO += horaTO / 24;
  diaWTO = (diaWTO + horaTO / 24) % 7;
  horaTO %= 24;

  if (añoTO % 4 == 0) { // Leap year check
    if ((mesTO == 1 || mesTO == 3 || mesTO == 5 || mesTO == 7 || mesTO == 8 || mesTO == 10 || mesTO == 12) && diaTO > 31) {
      diaTO = 1;
      mesTO++;
    } else if ((mesTO == 4 || mesTO == 6 || mesTO == 9 || mesTO == 11) && diaTO > 30) {
      diaTO = 1;
      mesTO++;
    } else if (mesTO == 2 && diaTO > 29) {
      diaTO = 1;
      mesTO++;
    }
  } else {
    if ((mesTO == 1 || mesTO == 3 || mesTO == 5 || mesTO == 7 || mesTO == 8 || mesTO == 10 || mesTO == 12) && diaTO > 31) {
      diaTO = 1;
      mesTO++;
    } else if ((mesTO == 4 || mesTO == 6 || mesTO == 9 || mesTO == 11) && diaTO > 30) {
      diaTO = 1;
      mesTO++;
    } else if (mesTO == 2 && diaTO > 28) {
      diaTO = 1;
      mesTO++;
    }
  }   

  datetime_t t_alarm = {
    .year  = (int16_t)añoTO,
    .month = (int8_t)mesTO,
    .day   = (int8_t)diaTO,
    .dotw  = (int8_t)diaWTO, // 0 is Sunday, so 5 is Friday
    .hour  = (int8_t)horaTO,
    .min   = (int8_t)minuteTO,
    .sec   = (int8_t)secondTO
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

  //try to reinit usb - NOT working... after sleep usb is down.
  //sleep_ms(10);
/*   stdio_init_all();
  stdio_usb_init();
  tusb_init(); */   

  return;
}
void Sleep(MQTT_CLIENT_T* stateM){
  if(Wifi.InitTimeGet && Wifi.ntpupdated && !Wifi.ntpproceso && mqttdone && !mqttproceso && dhtdone && bmpdone && ldrdone && inadone){  
    mqtt_disconnect(stateM->mqtt_client);       
    //execTime();      
    //Wifi.NTPLoop(); 
    //printf("\n-Sleeping... RTC: %02i/%02i %02i:%02i:%02i T: %02lli:%02lli:%03lli SON: %02lli ms\n",Wifi.DiaRTC, Wifi.MesRTC, Wifi.HorasRTC, Wifi.MinutosRTC, Wifi.SegundosRTC, MinutosTot, SegundosTot, MillisTot, exactOnTime);
    //measure_freqs();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); 
    #if !DEBUGTELNET
    Wifi.wifiOff(); 
    #endif                  
    exactSleepTimeFirst = time_us_64();          
    //-SLEEPING--
    #if DEBUG || DEBUGTELNET
      sleep_ms(SLEEPTIME*1000);     
    #elif !DEBUG && !DEBUGTELNET   
      //sleep_run_from_xosc();   not awaking 
      sleep_run_from_rosc();                             
      rtc_sleep_custom(0,SLEEPTIME);
      recover_from_sleep();     
    #endif     
    exactSleepTime = (time_us_64() - exactSleepTimeFirst)/1000;
    exactOnTimeFirst = time_us_64();    
    //execTime();
    //Wifi.NTPLoop();    
    //printf("\n----------------------------------------------------\n");
    //printf("\n-Awaking... RTC: %02i/%02i %02i:%02i:%02i T: %02lli:%02lli:%03lli SOFF: %02lli ms STOT: %02lli ms\n",Wifi.DiaRTC, Wifi.MesRTC, Wifi.HorasRTC, Wifi.MinutosRTC, Wifi.SegundosRTC, MinutosTot, SegundosTot, MillisTot, exactSleepTime, exactSleepTime+exactOnTime);
    //measure_freqs();
    //-SLEEP DONE-- 
    #if !DEBUGTELNET     
    Wifi.wifiConn(false,false);
    #endif
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);   
    mqttdone=false;
    mqttproceso=false;
    dhtdone=false;  
    bmpdone=false;
    ldrdone=false;   
    inadone=false;     
    serialdone=false;     
  } 
}
//- LDR ---------------------------------------
void LDRLoop() {
  if(ldrdone==false){
    adc_init();        
    adc_gpio_init(27);        
    adc_select_input(1);
    ldrValRaw = 0;
    for (int i = 0; i < 3; i++){
      ldrValRaw += 140 - adc_read();         
    }
    ldrVal = ldrValRaw / 3;
    if (ldrVal < 5) {
      ldrVal = 0.01;
    }
    ldrdone=true;
    //printf("\n-LDR: %f\n",ldrVal);
  }
}
//- DHT22 ------------------------------------
void dht22() {
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
//- INA219 ------------------------------------------
void ina219Setup(){
  i.configure(RANGE_16V, GAIN_8_320MV, ADC_8SAMP, ADC_8SAMP);
}
void LoopINA219() {
  if(inadone==false){     
    float busvoltageSum=0,shuntvoltageSum=0,current_mASum=0,power_mWSum=0,loadvoltageSum=0;
    int numLoop = 3;
    i.wake();
    for(int t = 0; t < numLoop; t++){
      busvoltageSum += i.voltage();
      shuntvoltageSum += i.shunt_voltage();
      current_mASum += i.current(); 
      power_mWSum += i.power();
      loadvoltageSum += i.supply_voltage();
    }    
    i.sleep(); 
    busvoltage = busvoltageSum/numLoop;
    shuntvoltage = shuntvoltageSum/numLoop;
    current_mA = current_mASum/numLoop; 
    power_mW = power_mWSum/numLoop;
    loadvoltage = loadvoltageSum/numLoop; 
    if (current_mA < 0){
      current_mA *=-1;
    } 
    current_A = current_mA / 1000;        
    if (power_mW < 0){
      power_mW *=-1;
    }
    power_W = power_mW / 1000; 
    exactOnTime = (time_us_64() - exactOnTimeFirst)/1000;
    double Ont = exactOnTime;
    double Oft = exactSleepTime;
    if(Ont > 0 && Oft > 0){
      total_mAH += ((current_mA * (Ont / 3600000.0)) + (3.0 * (Oft / 3600000.0))); //measured 3 mA during sleep
      total_mWH += ((power_mW * (Ont / 3600000.0)) + ((3.0*loadvoltage) * (Oft / 3600000.0))); //assume 3mA x V 
      execTime();
      double SegIna = Segundos;
      if(SegIna > 0.0){total_mAM = (total_mAH / (SegIna/3600.0))/18;total_mWM = (total_mWH / (SegIna/3600.0))/18;}
    } //Divided by 18.
    #if DEBUGINA219   
    if (absolute_time_diff_us(get_absolute_time(), INA219sendTimer) < 0) {  
      printf("\n------ INA219 DEBUG ------\n");
      printf("-busvoltage: %.3f V\n", busvoltage);
      printf("-shuntvoltage: %.2f mV\n", shuntvoltage);
      printf("-loadvoltage: %.3f V\n", loadvoltage);
      printf("-current_mA: %.2f mA\n", current_mA);
      printf("-power_mW: %.2f mW\n", power_mW);
      printf("-cON: %.4f cOFF: %.4f total_mAh: %.2f mAh/h: %.2f\n", (current_mA * (Ont / 3600000.0)),(4.0 * (Oft / 3600000.0)), total_mAH, total_mAM);
      printf("-total_mWh: %.2f mWh/h: %.2f\n", total_mWH, total_mWM);
      printf("-Segundos: %02lli t_Btwn_Meas/OffTime: %02lli t_Meas/onTime: %.1f\n",Segundos ,exactSleepTime,Ont);
      printf("---------- END ----------\n");        
      INA219sendTimer = make_timeout_time_us(1000000); //1000ms 
    }
    #endif
    inadone=true;  
    exactSleepTimeFirst = time_us_64()/1000;  
  }
}
//- BMP280 ------------------------------------
void bmp280setup() {
  #if DEBUGBMP280
    printf("\n-BMP280 SETUP"); 
  #endif  
  spi_init(spi0, 500000);                     //Baudrate 0.5Mhz - couldn't find in datasheet
  spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, (spi_order_t)0); //SPI acceptable 00 and 11 configuration
  gpio_set_function(MOSI, GPIO_FUNC_SPI);     //Mapping GPIO
  gpio_set_function(MISO, GPIO_FUNC_SPI);
  gpio_set_function(SCK, GPIO_FUNC_SPI);
  gpio_init(CS);
  gpio_set_dir(CS, true);
  gpio_put(CS, true);
  bmp280.getTrimmingParameters();
  spi_write_blocking(spi0, &datachipid, 1);
  spi_read_blocking(spi0, 0, &chipID, 1);
  sleep_ms(10);
  if(!bmp280.setPowerMode(BMP280::PowerMode::Forced, true)){printf("\n-Error PowerMode\n");}
  else{
    #if DEBUGBMP280
      printf("\n-Power mode: %i\n", bmp280.readPowerMode());
    #endif
  }
  sleep_ms(10);
  //(Temperature oversampling x1, pressure oversampling x1 0x27->0b00100111)
  if(!bmp280.setRegister(0xF4, 0b00100111, true)){printf("\n-Error Register\n");} 
  sleep_ms(10);
  if(!bmp280.setOversampling(BMP280::Temperature, (uint8_t)1, true)){printf("\n-Error OVS TEMP\n");}
  else{
    #if DEBUGBMP280
      printf("-Temperature oversampling: %i\n", bmp280.readOversampling(BMP280::Type::Temperature));
    #endif
  }
  sleep_ms(10);
  if(!bmp280.setOversampling(BMP280::Pressure, (uint8_t)1, true)){printf("\n-Error OVS PRESS\n");}
  else{
    #if DEBUGBMP280
      printf("-Pressure oversampling: %i\n", bmp280.readOversampling(BMP280::Type::Pressure));
    #endif
  }
  sleep_ms(10);
} 
void bmp280loop() {
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
//- DATA COLLECTION FUNC. ------------------
void MedGen(){
  LDRLoop();      
  bmp280loop();
  dht22();
  Tcomb = (temperature + temperature_c)/2;
}
//- GENERIC ----------------------------------
long mapcustom(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//- SETUP INFO -----------------------------------
static void SetupInfo(){
  if (watchdog_enable_caused_reboot()) {
    printf("\n-Rebooted by Watchdog Timeout!\n");
  } else {
    printf("\n-Clean boot\n");
  }
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
//- SERIAL INFO -----------------------------------
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
static void serialInfo(){
  if(serialdone==false){         
    //-TEMP
    adc_set_temp_sensor_enabled(true);
    // Get pico voltage
    voltage = 0;
    voltage_return = power_voltage(&voltage);
    voltage = floorf(voltage * 100) / 100;
    // Also get the temperature
    adc_select_input(4); 
    adc = 0;
    for (size_t i = 0; i < PICO_POWER_SAMPLE_COUNT; i++){
      adc += (float)adc_read() * conversionFactor;
    }            
    tempC = 27.0f - ((adc/PICO_POWER_SAMPLE_COUNT) - 0.706f) / 0.001721f; 
    //-TEMP
    adc_set_temp_sensor_enabled(false);  
    serialdone=true;
  }
}
//- TIMEOUT CONTROL -------------------  
void handleTimeout(const char *timeoutType) {
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  sleep_ms(300);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  sleep_ms(300);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  sleep_ms(300);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  printf("\n--- TIMEOUT %s: +%lli SECONDS, sysSeg: %lli lastSendSeg: %lli ---", timeoutType, Segundos-StartingSEC, Segundos, StartingSEC);
  printf("\n-MQTT %s. NTP %s. Sensors %s: \n", mqttdone ? "OK" : "BAD", Wifi.ntpupdated ? "OK" : "BAD", dhtdone && bmpdone && ldrdone && inadone ? "OK" : "BAD");
  printf("-initDate:%d ntpproceso:%d ntpupdated:%d \n-mqttdone:%d mqttproceso:%d \n-dhtdone:%d \n-bmpdone:%d \n-ldrdone:%d \n-inadone:%d \n----------\n", Wifi.InitTimeGet,Wifi.ntpproceso,Wifi.ntpupdated, mqttdone, mqttproceso, dhtdone, bmpdone, ldrdone, inadone);
  if(strcmp(timeoutType,"MQTT")==0 || strcmp(timeoutType,"MQTT+SENSORS")==0 || strcmp(timeoutType,"MQTTproceso")==0 || strcmp(timeoutType,"NTPproceso")==0){
    dhtdone=false;  
    bmpdone=false;
    ldrdone=false;  
    inadone=false;     
    serialdone=false; 
    mqttdone=false;
    mqttproceso=false; 
    Wifi.ntpproceso=false;
    Wifi.ntpupdated=false; 
  } 
}
void TimeoutControl(MQTT_CLIENT_T *stateM){
  if (absolute_time_diff_us(get_absolute_time(), timeoutControl_timer) < 0) {
    if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == false && mqttdone == false && dhtdone == true && bmpdone == true && ldrdone == true && inadone == true){
      handleTimeout("MQTT");
      sleep_ms(3000);
    }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == false && mqttdone == true && (dhtdone != true || bmpdone != true || ldrdone != true || inadone != true)){
      handleTimeout("SENSORS"); 
      sleep_ms(60000);  
    }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == false && mqttdone == false && (dhtdone != true || bmpdone != true || ldrdone != true || inadone != true)){
      handleTimeout("MQTT+SENSORS"); 
    }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && mqttproceso == true){
      handleTimeout("MQTTproceso");   
      StartingSEC=Segundos;
    }else if(StartingSEC+(uint64_t)(SLEEPTIME+50) <= Segundos && (Wifi.ntpproceso || !Wifi.ntpupdated)){
      handleTimeout("NTPproceso");   
      StartingSEC=Segundos;
    }
    timeoutControl_timer = make_timeout_time_us(500000);   //500ms 
  }   
}
//- WIFI RECONNECT LOOP ----------
void Reconnect_Loop(){
  printf("\n------- LOOP WIFI ERROR -------\n");
  Wifi.wifiConn(false,true); 
  mqttdone=false;
  mqttproceso=false;
  dhtdone=false;  
  bmpdone=false;
  ldrdone=false;   
  inadone=false;     
  serialdone=false;   
  sleep_ms(1000);
}
//-------------- END FUNC. --------------------

int main() {    
  //- SYSTEM SETUP -----------
  vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
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
  #if DEBUGSERIALWAIT
    while(!stdio_usb_connected()){sleep_ms(1);}       //Wait til active stdio CDC connection ready
    sleep_ms(50);
    if(stdio_usb_connected()){ 
      printf("\n--------- Active stdio CDC connection detected ---------\n");
    }
  #endif 
  TiempoWait = time_us_64();
  //- SETUP START ------------
  printf("\n------------ SETUP -------------\n");
  SetupInfo();
  Wifi.wifiConn(false,true); 
  ina219Setup(); 
  bmp280setup();  
  #if DEBUGTELNET
  telnetd.telnetdSetup();
  #endif 
  //- MQTT --------------
  MQTT_CLIENT_T *stateM = mqtt_client_init();     
  run_dns_lookup(stateM);
  stateM->mqtt_client = mqtt_client_new();
  stateM->counter = 0;
  if(stateM->mqtt_client == NULL){printf("-Failed to create new mqtt client\n");}else{printf("-MQTT SETUP\n");}
  //- ONBOARD LED OFF -----------
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  //- SETUPTIME ---------
  TiempoLoop = time_us_64()-TiempoWait;
  printf("\n-SETUP OK! -T.Wait: %ims -T.Setup: %ims -T.Total: %ims\n",TiempoWait/1000,TiempoLoop/1000,(TiempoWait+TiempoLoop)/1000);    
  //- END SETUP --------------    
  printf("\n------------ LOOP -------------\n"); 
  while(true){   
    while(cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA) == CYW43_LINK_UP){ 
      execTime(); 
      Wifi.NTPLoop();
      serialInfo();
      LoopINA219();      
      MedGen();                     
      MQTT(stateM);
      TimeoutControl(stateM);  
      Sleep(stateM);      
    }
    Reconnect_Loop();
  }
  cyw43_arch_deinit();     
  return 0;
} 