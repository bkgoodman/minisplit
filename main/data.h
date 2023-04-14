#include <esp_http_server.h>
/* Defines */

#define RINGS 8
#define MODE_AUTOADVANCE (10)
#define SEQSIZE 100

// Power Stage Change - do not report flags
#define REPORTFLAG_NOMQTT 1
#define REPORTFLAG_NONVS 2
#define REPORTFLAG_DESIRED 4
#define STORAGE_NAMESPACE "PlasmaLamp"

#define GPIO_LED GPIO_NUM_2
#define GPIO_STARTGAME GPIO_NUM_23
#define GPIO_ENDGAME GPIO_NUM_19

/* Data Types */

typedef struct ring_s {
  unsigned short start;
  unsigned short end;
  unsigned short size;
  double seq;
  unsigned short pos;
  unsigned short mode;
  double speed;
  unsigned short angle;
  unsigned short width;
  unsigned short len;
	unsigned char red;
	unsigned char green;
	unsigned char blue;
	unsigned long huespeed;
	unsigned long hue;			// We use top byte only
} ring_t;

typedef struct preset_s {
  ring_t rings[RINGS];
  unsigned short sparkle;
  unsigned short numflicker;
  unsigned char flicker_r;
  unsigned char flicker_g;
  unsigned char flicker_b;
} preset_t;

/* GLobals */

extern short ringsize[RINGS];
extern ring_t  rings[RINGS];
extern unsigned int globalDelay;
/* Slot is the CURRENT things running.
 MODE is the desired SETTING. These often equate - but a 
 MODE of 10 means we want to auto-advance SLOTs! */
extern int mode ;
extern bool powerState;
extern volatile short workphase;
extern unsigned short sparkle;
extern unsigned char flicker_r;
extern unsigned char flicker_g;
extern unsigned char flicker_b;
extern unsigned short numflicker;
extern short preset_running;
extern unsigned short fade_in;
extern unsigned short fade_out;
extern time_t next_time;

/* Functions */

bool load_powerState();
void change_displayMode(short newMode,unsigned short reportFlags);
short load_displayMode();
void save_displayMode();
void tcp_server_task(void *pvParameters);
void console(void *parameters);
httpd_handle_t start_webserver(void);
void plasma_powerOn(unsigned short reportFlags);
void plasma_powerOff(unsigned short reportFlags);

void mqtt_app_start(void);
void plasma_powerOn(unsigned short reportFlags);
void plasma_powerOff(unsigned short reportFlags);
void change_displayMode(short mode, unsigned short reportFlags);
void save_powerState();
void mqtt_report_powerState(bool powerStat, unsigned short requestFlags);
void mqtt_report_displayMode(short displayMode, unsigned short requestFlags);
void hue_to_rgb(int hue, unsigned char *ro, unsigned char *go, unsigned char *bo);

void connect_handler(void* arg, esp_event_base_t event_base, 
                            int32_t event_id, void* event_data);

void disconnect_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data);
void test_neopixel(void *parameters);
int load_preset(int slot);
void save_preset(int slot);
void telnet_command_handler(int sock, char *buffer);

