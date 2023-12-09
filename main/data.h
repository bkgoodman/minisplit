#include <esp_http_server.h>
/* Defines */


// Power Stage Change - do not report flags
#define REPORTFLAG_NOMQTT 1
#define REPORTFLAG_NONVS 2
#define REPORTFLAG_DESIRED 4
#define STORAGE_NAMESPACE "PlasmaLamp"

/* GLobals */

/* Slot is the CURRENT things running.
 MODE is the desired SETTING. These often equate - but a 
 MODE of 10 means we want to auto-advance SLOTs! */

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

