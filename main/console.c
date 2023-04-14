#include 	<stdio.h>
#include	<string.h>
#include "driver/uart.h"
#include	"esp_event.h"	//	for usleep
#include "esp_log.h"
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_console.h"

#include "uart.h"
#include "lwip/sockets.h"
#include "data.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "wifi_sta.h"
#include "ota.h"


static const char *TAG = "Console";
#include <stdarg.h>

#define TXBUFSIZE (100)

typedef struct telnet_commands_s {
  const char *cmd;
  int (*func)(void *,int , char **); 
} telnet_commands_t;

static void printf_socket(void *context, const char *format, va_list args) {
  char txbuf[TXBUFSIZE+1];
  int sock = (int) context;
  int len = vsnprintf(txbuf,TXBUFSIZE,format,args);

  int to_write = len;
            while (to_write > 0) {
                int written = send(sock, txbuf + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
}


void printf_console(void *context, const char *format, ...)
{
    va_list args;
    va_start(args, format);

    if (context==NOSOCKET)
      vprintf(format, args);
    else
      printf_socket(context,format, args);

    va_end(args);
}


void dumpmem(void *context,unsigned int addr, unsigned char *data,int size) {
      int i;
      for (int j = 0; j < size; j += 16) {
        for (int k = 0; k < 16; k++) {
            i = j+k;
            if (i>=size) 
              printf_console(context,"   ");
            else
              printf_console(context,"%02x ", data[i]);
        }

        for (int k = 0; k < 16; k++) {
            i = j+k;
            if (i>=size) {
                printf_console(context," ");
            }
            else if ((data[i] ) < 32 || (data[i] & 0xff) >= 127) {
                printf_console(context,"?");
            } else {
                printf_console(context,"%c", data[i] & 0xff);
            }
        }
        printf_console(context,"\r\n");
      }
}
static int generic_debug_cmd(void *context,int argc, char **argv) {
#if 0
    printf_console(context,  "Task Name\tStatus\tPrio\tHWM\tTask\tAffinity\n");
    char *stats_buffer = malloc(1024);
    vTaskList(stats_buffer);
    printf_console(context, "%s\n", stats_buffer);
    free (stats_buffer);
#endif
    heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
    //printf_console(context, "Workphase %d\n",workphase);
    return(0);
}

static int do_debug_cmd(int argc, char **argv) {
  return generic_debug_cmd(NOSOCKET,argc,argv);
}

static int generic_cli_power(void *context, int argc, char **argv) {
  /*
  if (argc < 2)
    printf_console(context, "Power state is %s\n",powerState?"on":"off");
  else if (!strcmp("on",argv[1])) {
    printf_console(context, "Turning powerState ON\n");
    plasma_powerOn(REPORTFLAG_DESIRED);
  }
  else if (!strcmp("off",argv[1])) {
    printf_console(context, "Turning powerState OFF\n");
    plasma_powerOff(REPORTFLAG_DESIRED);
  }
  else printf_console(context, "Powerstate must be \"on\" or \"off\"\n");
  */
  return 0;
}

static int do_cli_power(int argc, char **argv) {
  return generic_cli_power(NOSOCKET,argc,argv);
}


/*** UPDATE ***/

static int generic_update_cmd(void *context, int argc, char **argv) {
printf_console(context, "Doing Update\n");
  ota_update(context);
  
  return 0;
}

static int do_update_cmd(int argc, char **argv) {
  return generic_update_cmd(NOSOCKET,argc,argv);
}


/*** VERSION ***/

static int generic_version_cmd(void *context, int argc, char **argv) {
  esp_err_t	err;
  nvs_handle_t h;
  managed_settings_t *man;
  size_t size;
  char ipaddr[16];
  int rssid;
    esp_app_desc_t running_app_info;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
	printf_console(context,"Running firmware version: %s\n", running_app_info.version);
    }

  getNetworkInfo(ipaddr,&rssid);
  printf_console(context,"IP Address %s RSSID %d\n",ipaddr,rssid);

  nvs_open(STORAGE_NAMESPACE,NVS_READONLY,&h);
  size=sizeof(*man);
  man = malloc(size);

  err = nvs_get_blob(h,"managed",man,&size);
  if (err) {
	  printf_console(context, "No NVS Settings\n");
  } else {

  	printf_console(context,"Managed setting: %d Temp: %d Overide Temp: %d Time %ld\n",
			man->mode,
			man->setpoint,
			man->override_setpoint,
			man->override_time);
  }
  nvs_close(h);
  free(man);
  
  return 0;
}

static int do_version_cmd(int argc, char **argv) {
  return generic_version_cmd(NOSOCKET,argc,argv);
}

/*** REBOOT ***/

static int generic_reboot_cmd(void *context, int argc, char **argv) {
  printf_console(context, "Rebooting\n");
  esp_restart();
  //uart_reset_rx_fifo(UART_NUM_1);
  esp_deep_sleep(5000*1000);
  return 0;
}

static int do_reboot_cmd(int argc, char **argv) {
  return generic_reboot_cmd(NOSOCKET,argc,argv);
}

/*** HOSTNAME ***/

static int generic_hostname_cmd(void *context, int argc, char **argv) {
  esp_err_t	err;
  size_t size;
  char hostname[20];
  nvs_handle_t h;
  nvs_open(STORAGE_NAMESPACE,NVS_READWRITE,&h);
  if (argc == 2) {
    	printf_console(context, "Setting Hostname to  \"%s\"\n",argv[1]);
	nvs_set_str(h,"hostname",argv[1]);
  }
  size=19;
  err = nvs_get_str(h,"hostname",hostname,&size);
  if (err) {
	  printf_console(context, "No hostname set\n");
  } else {
  	printf_console(context, "Hostname is  \"%s\"\n",hostname);
  }
  nvs_close(h);
  
  return 0;
}

static int do_hostname_cmd(int argc, char **argv) {
  return generic_hostname_cmd(NOSOCKET,argc,argv);
}

/*** UART BAUD ***/

static int generic_baud_cmd(void *context, int argc, char **argv) {
  int baud;
  if (argc == 2) {
    baud = strtoul(argv[1],0L,10);
    bkg_uart_set_baud(baud);
    printf_console(context, "Baudrate set to  %d\n",baud);
  }
  
  return 0;
}

static int do_baud_cmd(int argc, char **argv) {
  return generic_baud_cmd(NOSOCKET,argc,argv);
}

/*** UART RECV ***/

static int generic_uart_recv(void *context, int argc, char **argv) {
  int i;
  unsigned char buf[100];
  for (i=1;i<argc;i++) {
    printf_console(context, "RECV %s\n",argv[i]);
  }
  int res = bkg_uart_recv(buf,sizeof(buf)-1);
  if (res > 0) {
    printf_console(context, "Received:\n");
    dumpmem(context, 0, buf,res);
    printf_console(context, "Recieve returned %d\n",res);

    res = crunchPacket(buf,res);
    printf_console(context, "Processed= %d\n",res);
  }

  
  return 0;
}

static int do_uart_recv(int argc, char **argv) {
  return generic_uart_recv(NOSOCKET,argc,argv);
}
/*** UART SEND ***/

static int generic_uart_send(void *context, int argc, char **argv) {
  unsigned char data[256];
  int len = 0;
  int i=1;
  bool dosum=0;
  if (argc == 1) return(-1);
  if (!strcmp(argv[1],"-c")) {
    dosum=1;
    i++;
  }
  for (;i<argc;i++) {
    data[len++] = strtoul(argv[i],0L,16);
  }

  if (dosum) {
    data[len] = checkSum(data,len);
    len++;
  }

  if (len > 0) {
    printf_console(context, "Sending:\n");
    dumpmem(context,0,data,len);
    bkg_uart_xmit(data,len);
  }
  return 0;
}

static int do_uart_send(int argc, char **argv) {
  return generic_uart_send(NOSOCKET,argc,argv);
}

static int generic_save_preset_cmd(void *context,int argc, char **argv) {
  /*
  if (argc < 2)
    return 1;
  short slot;
  slot = strtoul(argv[1],0L,0);
  save_preset(slot);
  printf_console(context, "Save preset to %d\n",slot);
  */
  return (0);
}

static int do_save_preset_cmd(int argc, char **argv) {
  return generic_save_preset_cmd(NOSOCKET,argc,argv);
}
static int generic_set_cmd(void *context,int argc, char **argv) {
	int power = POWER_NONE;
	int mode = MODE_NONE;
	int temp = TEMP_NONE;
	int fan = FAN_NONE;
  if (argc < 2)
    return 0;

  int i;
  for (i=1;i< argc;i++) {
	  if (!strcmp("power",argv[i])) { 
	    power = strtoul(argv[++i],0L,0);
	  	printf_console(context, "Setting power to %d\n",power);
	  }
	  else if (!strcmp("mode",argv[i])) { 
	    mode = strtoul(argv[++i],0L,0);
	  	printf_console(context, "Setting mode to %d\n",power);
	  }
	  else if (!strcmp("temp",argv[i])) { 
	    temp = strtoul(argv[++i],0L,0);
	  	printf_console(context, "Setting temp to %d\n",temp);
	  }
	  else if (!strcmp("fan",argv[i])) { 
	    temp = strtoul(argv[++i],0L,0);
	  	printf_console(context, "Setting fan to %d\n",temp);
	  }
	  else if (!strcmp("setOverride",argv[i])) { 
	    temp = strtoul(argv[++i],0L,0);
	    managed.override_time = temp;
	    printf_console(context, "Override time set to %d\n",temp);
	  }
	  else if (!strcmp("pushOverride",argv[i])) { 
	  	printf_console(context, "Pressing override button\n");
		do_override();
	  }
 }
  
  unsigned char packet[PACKET_LEN];
  createPacket(packet,power,mode,fan,temp);
  dumpmem(context,0,packet,PACKET_LEN);
  bkg_uart_xmit(packet,PACKET_LEN);
  return 0;
}

static int do_set_cmd(int argc, char **argv) {
  return generic_set_cmd(NOSOCKET,argc,argv);
}

static int generic_dump_cmd(void *context, int argc, char **argv) {
  /*
  int i;
  ring_t *rng;

  printf_console(context, "sparkle=%d;\n",sparkle);
  printf_console(context, "globalDelay=%d;\n",globalDelay);
  printf_console(context, "flicker_r=%d;\n",flicker_r);
  printf_console(context, "flicker_g=%d;\n",flicker_g);
  printf_console(context, "flicker_b=%d;\n",flicker_b);
  printf_console(context, "numflicker=%d;\n",numflicker);

  for (i=0;i < RINGS;i++)
  {
    rng = &rings[i];
    printf_console(context, " rng[%d]->speed=%f; rng[%d]->pos=%d; rng[%d]->mode=%d; rng[%d]->seq=%f;\n",i,rng->speed,i,rng->pos,i,rng->mode,i,rng->seq);
    printf_console(context, " rng[%d]->red=%d; rng[%d]->green=%d; rng[%d]->blue=%d; rng[%d]->angle=%d;\n",i,rng->red,i,rng->green,i,rng->blue,i,rng->angle);
    printf_console(context, " rng[%d]->width=%d; rng[%d]->len=%d; rng[%d]->huespeed=%lu\n",i,rng->width,i,rng->len,i,rng->huespeed);
  } 
  */
  return(0);
}
static int do_dump_cmd(int argc, char **argv) {
  return generic_dump_cmd(NOSOCKET,argc,argv);
}

static int generic_showring_cmd(void *context, int argc, char **argv) {
  /*
  int r;
  char *str1, *token, *subtoken, *subtoken2;
  char *saveptr1, *saveptr2;
  int j,i;

  printf_console(context, "SHOWRINGS %d args\n",argc);
  ring_t *rng;
  for (i=0;i<argc;i++)
    printf_console(context, "  %d - %s\n",i,argv[i]);
  if (argc < 2)
    return 1;

   for (j = 1, str1 = argv[1]; ; j++, str1 = NULL) {
       token = strtok_r(str1, ",", &saveptr1);
       if (token == NULL)
           break;

        subtoken = strtok_r(token, "-", &saveptr2);
        if (subtoken) {
                subtoken2 = strtok_r(NULL, "-", &saveptr2);
                if (!subtoken2) subtoken2=subtoken;
                for (r=strtoul(subtoken,0L,0);r<=strtoul(subtoken2,0L,0) && r < RINGS;r++)
								{
												rng = &rings[r];
												printf_console(context, "Ring %d\n",r);
												printf_console(context, " Was Speed %f Pos %d Mode %d seq %f \n  color %2.2x:%2.2x:%2.2x Ang %d Width %d Len %d huespeed %lu\n",
													rng->speed,rng->pos,rng->mode,rng->seq,rng->red,rng->green,rng->blue,
													rng->angle,rng->width,rng->len,rng->huespeed);
												for (i=2;i<argc;i+=2) {
													if (!strcmp("speed",argv[i]))
														rng->speed=strtof(argv[i+1],0L);  
													else if (!strcmp("mode",argv[i]))
														rng->mode=strtoul(argv[i+1],0L,0);  
													else if (!strcmp("pos",argv[i]))
														rng->pos=strtoul(argv[i+1],0L,0);  
													else if (!strcmp("seq",argv[i]))
														rng->seq=strtof(argv[i+1],0L);  
													else if (!strcmp("angle",argv[i]))
														rng->angle=strtoul(argv[i+1],0L,0);  
													else if (!strcmp("width",argv[i]))
														rng->width=strtoul(argv[i+1],0L,0);  
													else if (!strcmp("huespeed",argv[i]))
														rng->huespeed=strtoul(argv[i+1],0L,0);  
													else if (!strcmp("len",argv[i]))
														rng->len=strtoul(argv[i+1],0L,0);  
													else if (!strcmp("rgb",argv[i])) {
														unsigned long cc = strtoul(argv[i+1],0L,16);
														rng->red = (cc >> 16);
														rng->green = (cc & 0x00ff00) >> 8;
														rng->blue = (cc & 0x0000ff);
													}
													else if (!strcmp("hue",argv[i])) {
														hue_to_rgb(strtoul(argv[i+1],0L,0), &rng->red, &rng->green, &rng->blue);
													}
												printf_console(context, " Now Speed %f Pos %d Mode %d seq %f \n  color %2.2x:%2.2x:%2.2x Ang %d Width %d Len %d huespeed %lu\n",
													rng->speed,rng->pos,rng->mode,rng->seq,rng->red,rng->green,rng->blue,
													rng->angle,rng->width,rng->len,rng->huespeed);
												}
								}
       }
   }
   */
  return 0;
}

static int do_showring_cmd(int argc, char **argv) {
  return generic_showring_cmd(NOSOCKET,argc,argv);
}

static void initialize_console(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    ESP_LOGI("CONSOLE","WifiInit to \"%s\" with \"%s\"\\n",SSID,SSID_PASSWORD);
    repl_config.prompt = "HeatPump> ";
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    const esp_console_cmd_t ring_cmd = {
        .command = "ring",
        .help = "SHow Ring Parameters",
        .hint = NULL,
        .func = &do_showring_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ring_cmd));
        const esp_console_cmd_t dump_cmd = {
        .command = "dump",
        .help = "Dump state",
        .hint = NULL,
        .func = &do_dump_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&dump_cmd));
    const esp_console_cmd_t set_cmd = {
        .command = "set",
        .help = "Set Parameters",
        .hint = NULL,
        .func = &do_set_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_cmd));
    const esp_console_cmd_t debug_cmd = {
        .command = "debug",
        .help = "debug info",
        .hint = NULL,
        .func = &do_debug_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&debug_cmd));
    const esp_console_cmd_t save_preset_cmd = {
        .command = "save_preset",
        .help = "Save current as preset (X)",
        .hint = NULL,
        .func = &do_save_preset_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&save_preset_cmd));
    const esp_console_cmd_t power_cmd = {
        .command = "power",
        .help = "Power State",
        .hint = NULL,
        .func = &do_cli_power,
        .argtable = 0L
    };    
    
    ESP_ERROR_CHECK(esp_console_cmd_register(&power_cmd));
    
    
    const esp_console_cmd_t uart_baud_cmd = {
        .command = "baud",
        .help = "Set Baudrate",
        .hint = NULL,
        .func = &do_baud_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&uart_baud_cmd));

    const esp_console_cmd_t uart_recv_cmd = {
        .command = "recv",
        .help = "Recv on UART",
        .hint = NULL,
        .func = &do_uart_recv,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&uart_recv_cmd));

    const esp_console_cmd_t uart_send_cmd = {
        .command = "send",
        .help = "Send on UART",
        .hint = NULL,
        .func = &do_uart_send,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&uart_send_cmd));

    const esp_console_cmd_t hostname_cmd = {
        .command = "hostname",
        .help = "Set or View Hostname",
        .hint = NULL,
        .func = &do_hostname_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&hostname_cmd));

    const esp_console_cmd_t reboot_cmd = {
        .command = "reboot",
        .help = "Reboot",
        .hint = NULL,
        .func = &do_reboot_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&reboot_cmd));

    const esp_console_cmd_t version_cmd = {
        .command = "version",
        .help = "View Firmware Version",
        .hint = NULL,
        .func = &do_version_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&version_cmd));

    const esp_console_cmd_t update_cmd = {
        .command = "update",
        .help = "upgrade firmware <url>",
        .hint = NULL,
        .func = &do_update_cmd,
        .argtable = 0L
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&update_cmd));

    ESP_LOGI(TAG,"Console handlers initialized\n");
}

static int generic_help_cmd(void *context, int argc, char **argv);

telnet_commands_t telnet_commands[] = {
  {
    .cmd="debug",
    .func=&generic_debug_cmd
  },
  {
    .cmd="dump",
    .func=&generic_dump_cmd
  },
  {
    .cmd="showring",
    .func=&generic_showring_cmd
  },
  {
    .cmd="save_preset",
    .func=&generic_save_preset_cmd
  },
  {
    .cmd="recv",
    .func=&generic_uart_recv
  },
  {
    .cmd="baud",
    .func=&generic_baud_cmd
  },
  {
    .cmd="send",
    .func=&generic_uart_send
  },
  {
    .cmd="set",
    .func=&generic_set_cmd
  },
  {
    .cmd="power",
    .func=&generic_cli_power
  },
  {
    .cmd="hostname",
    .func=&generic_hostname_cmd
  },
  {
    .cmd="version",
    .func=&generic_version_cmd
  },
  {
    .cmd="reboot",
    .func=&generic_reboot_cmd
  },
  {
    .cmd="update",
    .func=&generic_update_cmd
  },
  {
    .cmd="help",
    .func=&generic_help_cmd
  },
  {
    .cmd=0L,
    .func=0L
  }
};
void printf_socket_static(void *context, const char *arg1, ...)
{
   va_list ap;
   va_start(ap, arg1);
   printf_socket(context,arg1, ap);
   va_end(ap);
}
static int generic_help_cmd(void *context, int argc, char **argv) {
  telnet_commands_t  *cmd;
  for (cmd=telnet_commands;cmd->cmd;cmd++) {
    printf_socket_static(context,"%s\n",cmd->cmd);
  }
  return (0);
}

#define MAX_ARGV (10)
void telnet_command_handler(int sock, char *buffer) {
  char *saveptr;
  char *str1,*token;
  int argc=0;
  telnet_commands_t  *cmd;
  char *argv[MAX_ARGV];
  for (argc = 0, str1 = buffer ; (argc<MAX_ARGV);  argc++, str1 = NULL) {
      token = strtok_r(str1, " ", &saveptr);
      if (token == NULL)
          break;
      argv[argc]=token;
  }

  for (cmd=telnet_commands;cmd->cmd;cmd++) {
    if (!strcmp(argv[0],cmd->cmd)) {
      cmd->func((void *)sock,argc,argv);
      break;
    }
  }
}

void console(void *parameters) {
    /* Register commands */
    initialize_console();
    printf("Console init done\n");
    while(1) {
      vTaskDelay(1000*1000);
    }
}

