#include "uart.h"
#include "driver/uart.h"
#include <string.h>
#include <time.h>
#include <esp_log.h>
#include "driver/gpio.h"
#include "mqtt.h"
#include "pins.h"

#define PACKET_SENT_INTERVAL_MS  1000
#define PACKET_INFO_INTERVAL_MS  2000
#define PACKET_TYPE_DEFAULT  99

#define START_OF_PACKET 0xfc

#define POWER_OFF 0
#define POWER_ON 1
#define POWER_NONE 16 // Our own "Do-Not-Update" flag

#define MODE_NONE 0 // Our own "Do-Not-Update" flag
#define MODE_HEAT 1
#define MODE_DRY 2
#define MODE_COOL 3
#define MODE_FAN 7
#define MODE_AUTO 8

#define TEMP_NONE 0 // Our own "Do-Not-Update" flag


int actual_roomTemp = TEMP_NONE;
unsigned char queried_power=POWER_NONE;
unsigned char queried_fan=FAN_NONE;
unsigned char queried_mode=MODE_NONE;
unsigned char queried_temp=TEMP_NONE;
unsigned char queried_raw_temp=0;
unsigned char queried_operating=0;
unsigned char alarm_state = ALARM_STATE_UNKNOWN;
long last_ir =0L;
long calendar_override_end =0L;
long need_ir_by =0L;
char last_report[LAST_REPORT_SIZE]={0};

char *alarmStateString() {
	switch (alarm_state) {
		case ALARM_STATE_UNKNOWN:
			return ("Unknown");
		case ALARM_STATE_ARMED:
			return ("Armed");
		case ALARM_STATE_DISARMED:
			return ("Disarmed");
		default:
			return ("Invalid");
	}
}
// https://github.com/SwiCago/HeatPump/blob/master/src/HeatPump.cpp
//
//
/*
HeatPump>  send -c fc 42 01 30 10 20
Sending:
fc 42 01 30 10 20 5d                            ?B?0? ]
HeatPump>  send -c fc 42 01 30 10 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
Sending:
fc 42 01 30 10 20 00 00 00 00 00 00 00 00 00 00 ?B?0? ??????????
00 00 00 00 00 5d                               ?????]
HeatPump>  I (1639840) RX_TASK: Read 22 bytes:
fc 62 01 30 10 20 04 08 0c 10 14 18 3c 40 44 48 ?b?0? ??????<@DH
4c 50 54 58 00 99                               LPTX??

Read Room Temp
fc 42 01 30 10 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 7a


I (1639850) UART: crunchPacket returned -4

*/

/* Header is 5 bytes like:
 * 0xFC (SOF)
 * Opcode
 * 0x01
 * 0x30
 * Data Length

 *
 * Then we have "Data Length" bytes of data payload
 
 * Then we have our checksum
 
 */

#define TAG "UART"

#define CONNECT_LEN 8
unsigned char CONNECT[CONNECT_LEN] = {0xfc, 0x5a, 0x01, 0x30, 0x02, 0xca, 0x01, 0xa8};
#define HEADER_LEN  8
const byte HEADER[HEADER_LEN]  = {0xfc, 0x41, 0x01, 0x30, 0x10, 0x01, 0x00, 0x00};
#define INFOHEADER_LEN  5
const byte INFOHEADER[INFOHEADER_LEN]  = {0xfc, 0x42, 0x01, 0x30, 0x10};

#define BUFLEN 256

#define INFOMODE_LEN 6
const byte INFOMODE[INFOMODE_LEN] = {
  0x02, // request a settings packet - RQST_PKT_SETTINGS
  0x03, // request the current room temp - RQST_PKT_ROOM_TEMP
  0x04, // unknown
  0x05, // request the timers - RQST_PKT_TIMERS
  0x06, // request status - RQST_PKT_STATUS
  0x09  // request standby mode (maybe?) RQST_PKT_STANDBY
};
const int RCVD_PKT_FAIL            = 0;
const int RCVD_PKT_CONNECT_SUCCESS = 1;
const int RCVD_PKT_SETTINGS        = 2;
const int RCVD_PKT_ROOM_TEMP       = 3;
const int RCVD_PKT_UPDATE_SUCCESS  = 4;
const int RCVD_PKT_STATUS          = 5;
const int RCVD_PKT_TIMER           = 6;
const int RCVD_PKT_FUNCTIONS       = 7;

bool tempMode=0;

// Byte 6
#define CONTROL_POWER 0x01
#define CONTROL_MODE 0x02
#define CONTROL_TEMP 0x04
#define CONTROL_FAN 0x08
#define CONTROL_VANE 0x10

// Byte 7
#define CONTROL_WIDEVANE 0x01

#define MAX_FUNCTION_CODE_COUNT 30

const byte CONTROL_PACKET_1[5] = {0x01,    0x02,  0x04,  0x08, 0x10};
                               //{"POWER","MODE","TEMP","FAN","VANE"};
const byte CONTROL_PACKET_2[1] = {0x01};
                               //{"WIDEVANE"};
const byte POWER[2]            = {0x00, 0x01};
const char* POWER_MAP[2]       = {"OFF", "ON"};
const byte MODE[5]             = {0x01,   0x02,  0x03, 0x07, 0x08};
const char* MODE_MAP[5]        = {"HEAT", "DRY", "COOL", "FAN", "AUTO"};
const byte TEMP[16]            = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
const int TEMP_MAP[16]         = {31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16};
const byte FAN[6]              = {0x00,  0x01,   0x02, 0x03, 0x05, 0x06};
const char* FAN_MAP[6]         = {"AUTO", "QUIET", "1", "2", "3", "4"};
const byte VANE[7]             = {0x00,  0x01, 0x02, 0x03, 0x04, 0x05, 0x07};
const char* VANE_MAP[7]        = {"AUTO", "1", "2", "3", "4", "5", "SWING"};
const byte WIDEVANE[7]         = {0x01, 0x02, 0x03, 0x04, 0x05, 0x08, 0x0c};
const char* WIDEVANE_MAP[7]    = {"<<", "<",  "|",  ">",  ">>", "<>", "SWING"};
const byte ROOM_TEMP[32]       = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
                                  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
const int ROOM_TEMP_MAP[32]    = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                                  26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41};
const byte TIMER_MODE[4]       = {0x00,  0x01,  0x02, 0x03};
const char* TIMER_MODE_MAP[4]  = {"NONE", "OFF", "ON", "BOTH"};
unsigned char raw[MAX_FUNCTION_CODE_COUNT];
bool functionValid1=0;
bool functionValid2=0;
#define TIMER_INCREMENT_MINUTES 10

const byte FUNCTIONS_SET_PART1 = 0x1F;
const byte FUNCTIONS_GET_PART1 = 0x20;
const byte FUNCTIONS_SET_PART2 = 0x21;
const byte FUNCTIONS_GET_PART2 = 0x22;

unsigned char normalizedWantedTemp(short temp) {
	unsigned char rawValue;
    	float c = ((temp-32)*5)/9;
	  if (!tempMode) {
		// Do by Index (Map)
		int t = 31-c;
		if (t < 0) t=0;
		if (t > 15) t=15;
		rawValue = t;
	  } else {
		// Temp quazi-direct Celcius
		rawValue = (c*2)+128;
	  }
	  return (rawValue);
}

char *charMapLookup(const byte *keys, const char **values, int key, int len) {
	int i;
	for (i=0;i<len;i++) {
		if (key == keys[i])
			return (char *) values[i];
	}
	return ("??");
}

int intMapLookup(const int *keys, const int *values, int key, int len) {
	int i;
	for (i=0;i<len;i++) {
		if (key == keys[i])
			return values[i];
	}
	return -1;
}
// these settings will be initialised in connect()
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
//
void createPacket(unsigned char *packet, int power, int mode, int fan, int temp) {

  memset(packet, 0, PACKET_LEN);
  
  for (int i = 0; i < HEADER_LEN; i++) {
    packet[i] = HEADER[i];
  }  

  if (power != POWER_NONE) {
	  packet[8] = power;
	  packet[6] += CONTROL_POWER;
  }

  if (mode != MODE_NONE) {
	  packet[9] = mode;
	  packet[6] += CONTROL_MODE;
  }

  if (temp != TEMP_NONE) {
    	float c = ((temp-32)*5)/9;
	  if (!tempMode) {
		// Do by Index (Map)
		int t = 31-c;
		if (t < 0) t=0;
		if (t > 15) t=15;
		packet[10] = t;
	  } else {
		// Temp quazi-direct Celcius
		packet[19] = (c*2)+128;
	  }
  packet[6] += CONTROL_TEMP;
  }


  packet[11] = fan; // ???? depends on TmpMode??
  packet[6] += CONTROL_FAN;
  /*

  packet[12] = vane; 
  packet[6] += CONTROL_VANE;

  packet[18] = widevane; 
  packet[7] += CONTROL_VANE;
  */
  packet[21] = checkSum(packet,PACKET_LEN-1);
  //Transmit!
}

void packets() {
}
 
const uart_port_t uart_num = UART_NUM_2;
void bkg_uart_init() {
  uart_config_t uart_config = {
      .baud_rate = 2400,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_EVEN,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      //.rx_flow_ctrl_thresh = 122,
  };
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, -1, -1));
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(uart_num, BUFLEN, \
                                          BUFLEN, 10, &uart_queue, 0));
}

void bkg_uart_xmit(void *data, size_t len) {
  // Write data to UART.
  uart_write_bytes(uart_num, (const char*)data, len);
  // Wait for packet to be sent
  ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 250)); // wait timeout is 100 RTOS ticks (TickType_t)
}

int bkg_uart_recv(void *data,size_t len) {
  // Read data from UART.
  int length = len;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
  length = uart_read_bytes(uart_num, data, length, 100);
  return (length);
}

void bkg_uart_set_baud(int rate) {
  ESP_ERROR_CHECK(uart_set_baudrate(uart_num,rate));
}

/* Given state of the system:
 * What should the current state of the system be?
 * Is the system actually in that state?
 * Change if needed
 */

void should_set(void) {
	bool change=false;
	short setMode = MODE_NONE;
	short want_temp=0;
	short setPower = POWER_NONE;
	if (managed.mode == MANAGED_UNMANAGED)
		return;

	if ((managed.mode == MANAGED_HEAT)  && ((queried_mode != MODE_HEAT) || (queried_power != POWER_ON))) {
		setMode = MODE_HEAT;
		setPower = POWER_ON;
		ESP_LOGI(TAG,"Managed mode needs HEAT and ON");
		change=true;
	}
	else if ((managed.mode == MANAGED_COOL)  && ((queried_mode != MODE_COOL) || (queried_power != POWER_ON))) {
		setMode = MODE_COOL;
		setPower = POWER_ON;
		ESP_LOGI(TAG,"Managed mode needs COOL and ON");
		change=true;
	}
	else if ((managed.mode == MANAGED_OFF)  && (queried_power != POWER_OFF)) {
		ESP_LOGI(TAG,"Managed mode needs OFF");
		setPower = POWER_OFF;
		change=true;
	}

	if (queried_fan != FAN_AUTO) {
		change=true;
	}

	/* Temperate is based on many factors */

	if (alarm_state == ALARM_STATE_ARMED) {
		want_temp = managed.setpoint_unoccupied;
	} else {
		/* If unknown or disaremd */
		if (override_end != 0) {
			// We are in override
			want_temp = managed.override_setpoint;
		} else {
			want_temp = managed.setpoint;
		}
	}

	unsigned char want_raw = normalizedWantedTemp(want_temp);
	if (queried_raw_temp != want_raw){
		ESP_LOGI(TAG,"Managed mode Alarm %s override %s needs temp %d,not temp %d (raw is %d, wants %d)",
			alarmStateString(),
			override_end != 0 ? "SET" : "off",
			want_temp,queried_temp,queried_raw_temp,want_raw);
		change=true;
	} 

	if (change) {
		ESP_LOGI(TAG,"Applying managed settings");
		unsigned char packet[PACKET_LEN];
		createPacket(packet, setPower, setMode, FAN_AUTO, want_temp);
		bkg_uart_xmit(packet,PACKET_LEN);
	}

}


void buildQuery(unsigned char *packet,unsigned char code) {
	 const unsigned char header[] = {0xfc, 0x42, 0x01, 0x30, 0x10};
	 memset(packet,0,PACKET_LEN);
	 memcpy(packet, header, sizeof(header));
	 packet[5] = code;
	 packet[21] = checkSum(packet,21);

}
/* Decode the packet */
int crunchPacket(unsigned char *bytes,int len) {
  byte *header;
  byte *data;
  byte checksum = 0;
  int temp;
  float t,f;
  unsigned char txData[PACKET_LEN];

  header = bytes;
  if ((header[0] & 0xfc) != HEADER[0] || header[2] != HEADER[2] || header[3] != HEADER[3]) 
    return -1;

header[0] &= ~0x02;
  header[1] &= ~0x80;
  data =&bytes[INFOHEADER_LEN];
  //short dataLength = header[4];

  checksum = checkSum(bytes,len-1);
  if (checksum != bytes[len-1])  {
    ESP_LOGE(TAG,"Checksum Error (Expected 0x%x Got 0x%x)",checksum,bytes[len-1]);
      return -2;
  }

  ESP_LOGI(TAG,"Type 0x%x Subtype 0x%x",header[1],data[0]);
  if(header[1] == 0x62) { 
    switch (data[0]) {
      case 0x02: // setting info
	  {
	  int mode = data[4];
	  int power = data[3];
	  int fan = data[6];
	  int vane = data[7];
	  int widevane = data[10];

	  queried_power=power;
	  queried_mode=mode;
	  queried_fan=fan;
	  f=0;

	  // This is the one we won't use (???)
		  temp = data[5];
		  float f2=0;
		  if (temp != 0) {
			  tempMode=false;
			  temp = 31-temp; // There is a limited range of settings here: 0=31c(88f) 15=15fc(59f)
			  f2 = ((temp*9)/5)+32;
			  ESP_LOGI(TAG,"TEMP format 1 raw %d C=%d F=%f",data[5],temp,f2);
			  queried_temp=f2;
		  }
	  // data11 is temp in celcius
	  temp = data[11];
	  if (temp != 0) {
		  temp -= 128;
		  t = temp/2;
		  f = ((t*9)/5)+32;
		  ESP_LOGI(TAG,"TEMP format 0 raw %d C=%f F=%f",data[11],t,f);
		  tempMode = true;
		  queried_temp =  f;
		queried_raw_temp=data[11];
	  } else { 
		  queried_temp = f2;
		  tempMode = false;
		queried_raw_temp=data[5];
	  }
	  


	  snprintf(last_report,LAST_REPORT_SIZE-1,"Power %s %d - Mode %s %d - Temp %f - Temp2 %f - tempMode %s - Fan %d - Vane %d - WideVane 0x%x",
		  charMapLookup(POWER,POWER_MAP,power,2),power,
		  charMapLookup(MODE,MODE_MAP,mode,5),mode,
		 f,f2,tempMode ? "true" : "false", fan,vane,widevane);

	  ESP_LOGI(TAG,"%s",last_report);

	mqtt_report(
		  charMapLookup(POWER,POWER_MAP,power,2),
		  queried_temp,actual_roomTemp,
		  charMapLookup(MODE,MODE_MAP,mode,5),
		  charMapLookup(FAN,FAN_MAP,fan,6));



	  ESP_LOGI(TAG,"MANAGED MODE IS %d",managed.mode);
		should_set();
	  }
          break;
      case 0x03: // Room Temp Reading
          ESP_LOGI(TAG,"Room Temp Reading");
	  if (data[6] != 0) {
		temp = data[6];
		temp -= 128;
		t = temp/2;
		f = ((t*9)/5)+32;
		ESP_LOGI(TAG,"Room temp is %f",f);
		actual_roomTemp = f;
	  } 
	  
	 buildQuery(txData,0x02);
	 bkg_uart_xmit((unsigned char *) txData,22);
	  // Query Settings
	 //const unsigned char txData3[] = {0xfc, 0x42, 0x01, 0x30, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00, 0x00, 0x00, 0x7b};

          break;
      case 0x05: // Timer
          ESP_LOGI(TAG,"Timer");
          break;
      case 0x06: // Status
          ESP_LOGI(TAG,"Status");
	 queried_operating = data[4];
	 buildQuery(txData,0x03);
	 bkg_uart_xmit((unsigned char *) txData,22);
              //receivedStatus.compressorFrequency = data[3];
          break;
      case 0x09: // Standby mode?
          ESP_LOGI(TAG,"Standby");
          break;
      case 0x20:
          ESP_LOGI(TAG,"Function Info1");
	  memcpy(raw,data,15);
	  functionValid1=1;
	  // Now get second half - i.e. Function2 data
	 buildQuery(txData,0x22);
	 //const unsigned char txData[] = {0xfc, 0x42, 0x01, 0x30, 0x10, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00, 0x00, 0x00, 0x5b};
	 bkg_uart_xmit((unsigned char *) txData,22);
	 break;
      case 0x22: // Received packet funtions?
          ESP_LOGI(TAG,"Function Info2");
	  memcpy(&raw[15],data,15);
	  functionValid2=1;
	  // Query Room Temperature
	 buildQuery(txData,0x06);
	 //const unsigned char txData2[] = {0xfc, 0x42, 0x01, 0x30, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00, 0x00, 0x00, 0x7a};
	 bkg_uart_xmit((unsigned char *) txData,22);
          break;
      default:
        return -4;
    }
    return 0;
  }
  else if(header[1] == 0x61) { 
    // Last update successful
    ESP_LOGI(TAG,"Update Successful");
	// Query Settings
	// Readback Settings
	// const unsigned char txData4[] = {0xfc, 0x42, 0x01, 0x30, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00, 0x00, 0x00, 0x7b};
	 buildQuery(txData,0x02);
	 bkg_uart_xmit((unsigned char *) txData,22);
    return (1);
  }
  else if(header[1] == 0x7a) { 
    // Connect Successful
    ESP_LOGI(TAG,"Connect Successful");
    // Query Function 1
	 //unsigned char txData[] = {0xfc, 0x42, 0x01, 0x30, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00, 0x00, 0x00, 0x5d};
	 buildQuery(txData,0x20);
	 bkg_uart_xmit((unsigned char *) txData,22);
    return (2);
  } else {
    ESP_LOGI(TAG,"Unknown Type");
    return -3;
  }

  
}

unsigned char  checkSum(unsigned char *bytes, int len) {
  unsigned char sum = 0;
  for (int i = 0; i < len; i++) {
    sum += bytes[i];
  }
  return (0xfc - sum) & 0xff;
}


long getSecs() {
	time_t now;
time(&now);
return (now);
}

#define MINIBUF 24
void uart_monitor_thread(void *parameters) {
  ESP_LOGI(TAG,"Monitor thread running");
  static const char *RX_TASK_TAG = "RX_TASK";
  uint8_t* data = (uint8_t*) malloc(MINIBUF+1);
  uint8_t* packet = (uint8_t*) malloc(PACKET_LEN+1);
  int packetsize=0;
  short awaitSOF=1;
  int i;
  long nextTime=0;
  //ESP_LOGI(RX_TASK_TAG, "send -c fc 5a 01 30 02 ca 01");
  //ESP_LOGI(RX_TASK_TAG, "send -c fc 42 01 30 10 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00");
  packets();
  // Send initial "Connect" packet
  unsigned char txData[] = {0xfc, 0x5a, 0x01, 0x30, 0x02, 0xca, 0x1, 0xa8};

	bkg_uart_xmit((unsigned char *) txData,8);
  while (1) {
      unsigned long now = getSecs();

      if ((override_end != 0) && (override_end <= now)) {
	      ESP_LOGW(TAG,"Override end");
	      override_end = 0;
				setLed(LED_STATE_OFF);
      }

      if ((calendar_override_end != 0) && (calendar_override_end <= now)) {
	      ESP_LOGW(TAG,"Calendar Override end");
	      calendar_override_end = 0;
				setLed(LED_STATE_OFF);
      }

      // If we are in override mode, and requested IR verification, and it's been too long since
      // we got it - cancel the override
      
      if ((override_end) &&  (need_ir_by) && (now > need_ir_by)) {
	      ESP_LOGW(TAG,"Override cancled to to lack of IR");
	      override_end = 0;
	      need_ir_by=0L;
				setLed(LED_STATE_OFF);
      }
      

      if (nextTime < now) {
			 ESP_LOGW(TAG,"Transmiting Connect");
			 bkg_uart_xmit((unsigned char *) txData,8);
			 nextTime = now+120;
      }
      const int rxBytes = uart_read_bytes(uart_num, data, MINIBUF, 5000 / portTICK_PERIOD_MS);
      if (rxBytes > 0) {
          data[rxBytes] = 0;
          ESP_LOGI(RX_TASK_TAG, "Read %d bytes: ", rxBytes);
	  dumpmem(NOSOCKET,0,data,rxBytes);
          ESP_LOGI(RX_TASK_TAG, "Processing packet (size=%d",packetsize);
	  dumpmem(NOSOCKET,0,packet,packetsize);
          for (i=0;i<rxBytes;i++) {
            if (awaitSOF) {
              if ((data[i] & START_OF_PACKET) == START_OF_PACKET) {
                packetsize=1;
                awaitSOF=0;
                packet[0] = data[i];
              }
            } else {
              // We are mid-packet
              packet[packetsize++] = data[i];

              if (packetsize >= INFOHEADER_LEN) {
                // Do we have enough header bytes to even determine what is here?
                if ((packet[0] & START_OF_PACKET) != HEADER[0] || packet[2] != HEADER[2] || packet[3] != HEADER[3])  {
                  // This is a botched header
                  awaitSOF=1;
                  packetsize=0;
		  ESP_LOGE(TAG,"Botched Packet");
                }

                if (packetsize == (INFOHEADER_LEN + 1 + packet[4])) {
                  // We should have a complete packet
                  
                  gpio_set_level(GPIO_LED,0);
                  vTaskDelay(250 / portTICK_PERIOD_MS);
                  gpio_set_level(GPIO_LED,1);
                  
                  int ret = crunchPacket(packet,packetsize);
                  // Reset for next packet
		  if (ret < 0)
			  ESP_LOGE(TAG,"crunchPacket returned ERROR %d",ret);
		  else
			  ESP_LOGI(TAG,"crunchPacket returned %d",ret);
                  awaitSOF=1;
                  packetsize=0;
                }
              }
            }
          }
      } else {
				ESP_LOGE(TAG,"NoData");
        // If we've gone 5 seconds with no data - assume we're getting a new transmission
	if (awaitSOF != 1)
		ESP_LOGE(TAG,"Clearing Packet");
        awaitSOF=1;
        packetsize=0;
      }



  }
  free(data);
  free(packet);
}

// IR motion detected
void do_ir() {
	ESP_LOGI(TAG,"IR Tripped");
	unsigned long now = getSecs();
	last_ir = now;
	if (managed.ir_interval) {
		need_ir_by = now + (managed.ir_interval * 60);
	}
	setLed(LED_FLAG_IR);
}

/* Override via Calendar API */
void do_calendar_override(unsigned long secs) {
	unsigned long now = getSecs();
	if (managed.mode == MANAGED_UNMANAGED) {
		ESP_LOGI(TAG,"Override is disabled");
    setLed(LED_FLAG_BUTTON);
		return;
	}
	override_end = now + (secs * 60);
	need_ir_by = 0;
	if (managed.mode == MANAGED_HEAT)
		setLed(LED_STATE_OVERRIDE_HEAT);
	else if (managed.mode == MANAGED_COOL)
		setLed(LED_STATE_OVERRIDE_COOL);
  else
    setLed(LED_FLAG_BUTTON);
	ESP_LOGW(TAG,"Calendar Override active");
}

/* Someone pressed the "override" button (physically, or virtually) */
void do_override() {
	unsigned long now = getSecs();

  if (calendar_override_end != 0) {
		ESP_LOGI(TAG,"Calendar Override Active");
    return;
  }

	if ((managed.override_time == 0) || (managed.mode == MANAGED_UNMANAGED)){
		ESP_LOGI(TAG,"Override is disabled");
    setLed(LED_FLAG_BUTTON);
		return;
	}


	override_end = now + (managed.override_time * 60);
	need_ir_by = 0;
	if (managed.ir_interval) {
		need_ir_by = now + (managed.ir_interval * 60);
	}
	if (managed.mode == MANAGED_HEAT)
		setLed(LED_STATE_OVERRIDE_HEAT);
	else if (managed.mode == MANAGED_COOL)
		setLed(LED_STATE_OVERRIDE_COOL);
  else
    setLed(LED_FLAG_BUTTON);
	ESP_LOGW(TAG,"Override active");
}
