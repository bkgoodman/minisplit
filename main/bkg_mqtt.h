void mqtt_app_start(void);
void plasma_powerOn(unsigned short reportFlags);
void plasma_powerOff(unsigned short reportFlags);
void change_displayMode(short mode, unsigned short reportFlags);
void save_powerState();
void mqtt_report_powerState(bool powerStat, unsigned short requestFlags);
void mqtt_report_displayMode(short displayMode, unsigned short requestFlags);

// Power Stage Change - do not report flags
#define REPORTFLAG_NOMQTT 1
#define REPORTFLAG_NONVS 2
#define REPORTFLAG_DESIRED 4