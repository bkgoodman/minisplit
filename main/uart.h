#include <stddef.h>

typedef unsigned char byte;
void bkg_uart_init();

int bkg_uart_recv(void *data,size_t length);
void bkg_uart_xmit(void *data, size_t len);
void bkg_uart_set_baud(int rate);
unsigned char  checkSum(unsigned char *bytes, int len);
int crunchPacket(unsigned char *bytes,int len);
void uart_monitor_thread(void *parameters);
void dumpmem(void *context,unsigned int addr, unsigned char *data,int size);
#define NOSOCKET ((void *) 0xFFFFFFFF)

#define PACKET_LEN  22

#define POWER_OFF 0
#define POWER_ON 1
#define POWER_NONE 16 // Our own "Do-Not-Update" flag

#define MODE_NONE 0 // Our own "Do-Not-Update" flag
#define MODE_HEAT 1
#define MODE_DRY 2
#define MODE_COOL 3
#define MODE_FAN 7
#define MODE_AUTO 8

#define FAN_NONE 16 // Our own "Do-Not-Update" flag
#define FAN_AUTO 0
#define FAN_QUIET 1
#define FAN_1 2
#define FAN_2 3
#define FAN_3 4
#define FAN_4 5

#define TEMP_NONE 0 // Our own "Do-Not-Update" flag

#define MANAGED_UNMANAGED 0
#define MANAGED_HEAT 1
#define MANAGED_COOL 2
#define MANAGED_OFF 3


typedef struct managed_setings_s {
	unsigned char mode;
	unsigned char setpoint;
	unsigned char setpoint_unoccupied;
	unsigned short	override_time;
	unsigned short	override_setpoint;
	unsigned short	ir_interval;
} managed_settings_t;

extern long	override_end;
long last_ir;
extern managed_settings_t managed;
extern const byte POWER[2];
extern const char *POWER_MAP[2];
extern const byte MODE[5];
extern const char *MODE_MAP[5];

void should_set(void);
#define ALARM_STATE_UNKNOWN 0
#define ALARM_STATE_ARMED 1
#define ALARM_STATE_DISARMED 2
extern unsigned char alarm_state;
char *alarmStateString();

void do_override(void);
void do_ir(void);

void createPacket(unsigned char *packet, int power, int mode, int fan,int temp);
void doQuery();

char *charMapLookup(const byte *keys, const char **values, int key, int len);
int intMapLookup(const int *keys, const int *values, int key, int len);
extern int actual_roomTemp;
extern unsigned char queried_power;
extern unsigned char queried_mode;
extern unsigned char queried_temp;
extern unsigned char queried_raw_temp;
extern unsigned char queried_operating;

#define LED_STATE_NO_CHANGE 0
#define LED_STATE_OFF 1
#define LED_STATE_UNOCCUPIED 2
#define LED_STATE_OVERRIDE_HEAT 3
#define LED_STATE_OVERRIDE_COOL 4
#define LED_STATE_MASK 0x0f

#define LED_FLAG_MASK 0xf0
#define LED_FLAG_IR 0x40
#define LED_FLAG_NEAREND 0x80

void setLed(short led);
