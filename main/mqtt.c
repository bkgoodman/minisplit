#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include <esp_system.h>
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_heap_trace.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_sleep.h"
#include <driver/gpio.h>
#include "wifi_sta.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "data.h"

#include "cJSON.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "uart.h"
#include <time.h>
#include <sys/time.h>

#include "bkg_mqtt.h"
#include "ota.h"

#define GPIO_LED GPIO_NUM_2
#define GPIO_BUTTON GPIO_NUM_34
static const char *TAG = "MQTT";
extern const uint8_t client_cert_pem_start[] asm("_binary_client_crt_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_pem_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_client_key_end");
extern const uint8_t CA_cert_pem_start[] asm("_binary_CA_crt_start");
extern const uint8_t CA_cert_pem_end[] asm("_binary_CA_crt_end");
extern char hostname[30];

#define STRINGIFY(x) #x
#define CAT3_STR(A,B,C) A B C
#define CAT2_STR(A,B) A B 

#define RETAIN 1


esp_mqtt_client_handle_t client_global = 0L;
#define NETWORK_RESTART()						xTaskCreate(network_restart_task, "Network Restart", 4096, NULL, 10, NULL)

static void network_restart_task(void* arg) {
	ESP_LOGI(TAG,"Network Restart");
	// DOESN'T WORK??? mqtt_restart();
	ESP_LOGI(TAG,"Finishing Network Restart");
	vTaskDelete(NULL);
}


void mqtt_app_start(void);
static void network_restart_task(void* arg);

#define MAX_REPORT_SIZE 511
void mqtt_report(char *power, int setpoint, int roomtemp, char *mode, char *fan) {
	char ipaddr[20];
	int rssid;
	char *t=malloc(MAX_REPORT_SIZE+1);
	char topic[60];
	char timestr[30];
	char ovend[30];
	char lastir[30];
	char modestr[10];
	getNetworkInfo(ipaddr,&rssid);
	    esp_app_desc_t running_app_info;
	    const esp_partition_t *running = esp_ota_get_running_partition();
	    char *fwversion=0L;
	    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
		fwversion =  running_app_info.version;
	    }

	ovend[0]=(char) 0;
	if (override_end != 0) {
		struct tm ov_tm;
		gmtime_r(&override_end,&ov_tm);
		asctime_r(&ov_tm,ovend);
		ovend[strlen(ovend)-1]=(char) 0;
	}

	if (calendar_override_end != 0) {
		struct tm ov_tm;
		gmtime_r(&calendar_override_end,&ov_tm);
		asctime_r(&ov_tm,ovend);
		ovend[strlen(ovend)-1]=(char) 0;
	}

	lastir[0]=(char) 0;
	if (last_ir != 0) {
		struct tm ir_tm;
		gmtime_r(&last_ir,&ir_tm);
		asctime_r(&ir_tm,lastir);
		lastir[strlen(lastir)-1]=(char) 0;
	}

	time_t tm = time(0L);
	asctime_r(gmtime(&tm),timestr);
	timestr[strlen(timestr)-1]=(char) 0;
	strcpy(modestr,"OFF");
	if (managed.mode == MANAGED_HEAT) strcpy(modestr,"HEAT");
	if (managed.mode == MANAGED_COOL) strcpy(modestr,"COOL");
	int reportsize = snprintf(t,MAX_REPORT_SIZE,"{\"power\":\"%s\",\"setpoint\":%d,\
\"roomTemp\":%d,\"mode\":\"%s\",\"fan\":\"%s\",\
\"ip\":\"%s\",\"rssid\":%d,\"fw\":\"%s\",\"time\":\"%s\",\
\"managed\":\"%s\",\"managed_setpoint\":%d,\"override_setpoint\":%d,\
\"override_time\":%d,\"override_end\":\"%s\",\
\"managed_setpoint_unoccupied\":%d,\"ir_interval\":%d,\
\"operating\":\"%s\",\
\"alarm\":\"%s\",\
\"last_ir\":\"%s\"\
}",
			power,setpoint,roomtemp,mode,fan,
			ipaddr,rssid,fwversion ? fwversion : "??",timestr,
			modestr,
			managed.setpoint,
			managed.override_setpoint,
			managed.override_time,
			ovend,
			managed.setpoint_unoccupied,
			managed.ir_interval,
			queried_operating ? "ON":"OFF",
			alarmStateString(),
			lastir
			);
	if (reportsize >= (MAX_REPORT_SIZE-20))
		ESP_LOGE(TAG, "MQTT Report size is %d\n",reportsize);
	ESP_LOGI(TAG, "Report: (%d) %s",reportsize,t);
	snprintf(topic,59,"facility/minisplit/report/%s",hostname);
	esp_mqtt_client_publish(client_global, topic, t, 0, 0, RETAIN);
	free(t);
}

static void got_msg(char *topic, int len, char *data, int datalen) {
	unsigned char packet[PACKET_LEN];
	bool writeManaged=false;
	char mytopic[60];
	snprintf(mytopic,59,"facility/minisplit/request/%s",hostname);
  	ESP_LOGI(TAG,"Got Topic \"%.*s\"",len,topic);
  	ESP_LOGI(TAG,"Got Message \"%.*s\"",datalen,data);
	
	if (!memcmp(topic,"facility/alarm/system",len)) {

		if (!memcmp(data,"armed",datalen)) {
			ESP_LOGI(TAG,"Alarm is armed");
			alarm_state = ALARM_STATE_ARMED;
		} 
		else if (!memcmp(data,"disarmed",datalen)) {
			ESP_LOGI(TAG,"Alarm is disarmed");
			alarm_state = ALARM_STATE_DISARMED;
		} 
	}
	else if (!memcmp(topic,mytopic,len)) {
			// On bootup read the last state of things
			cJSON *root = 0L;
      		ESP_LOGI(TAG, "Got request");
		int power=POWER_NONE;
		int mode=MODE_NONE;
		int fan=FAN_NONE;
		int temp=TEMP_NONE;

			root = cJSON_Parse(data);
			if (!root) return;
			cJSON *item = cJSON_GetObjectItem(root,"power");
			if (item) {
				ESP_LOGI(TAG,"Power set request to %s",item->valuestring);
				if (!strcmp(item->valuestring,"ON")) power=POWER_ON;
				if (!strcmp(item->valuestring,"OFF")) power=POWER_OFF;
			}
			item = cJSON_GetObjectItem(root,"mode");
			if (item) {
				ESP_LOGI(TAG,"Mode set request to %s",item->valuestring);
				if (!strcmp(item->valuestring,"HEAT")) mode=MODE_HEAT;
				if (!strcmp(item->valuestring,"COOL")) mode=MODE_COOL;
				if (!strcmp(item->valuestring,"DRY")) mode=MODE_DRY;
				if (!strcmp(item->valuestring,"FAN")) mode=MODE_FAN;
				if (!strcmp(item->valuestring,"AUTO")) mode=MODE_AUTO;
			}
			item = cJSON_GetObjectItem(root,"fan");
			if (item) {
				ESP_LOGI(TAG,"Fan set request to %s",item->valuestring);
				if (!strcmp(item->valuestring,"AUTO")) fan=FAN_AUTO;
				if (!strcmp(item->valuestring,"QUIET")) fan=FAN_QUIET;
				if (!strcmp(item->valuestring,"1")) fan=FAN_1;
				if (!strcmp(item->valuestring,"2")) fan=FAN_2;
				if (!strcmp(item->valuestring,"3")) fan=FAN_3;
				if (!strcmp(item->valuestring,"4")) fan=FAN_4;
			}
			item = cJSON_GetObjectItem(root,"temp");
			if (item) {
				ESP_LOGI(TAG,"Temp set request to %d",item->valueint);
				temp = item->valueint;
			}
			item = cJSON_GetObjectItem(root,"managed");
			if (item) {
				if (!strcmp(item->valuestring,"UNMANAGED")) managed.mode = MANAGED_UNMANAGED;
				if (!strcmp(item->valuestring,"HEAT")) managed.mode = MANAGED_HEAT;
				if (!strcmp(item->valuestring,"COOL")) managed.mode = MANAGED_COOL;
				if (!strcmp(item->valuestring,"OFF")) managed.mode = MANAGED_OFF;
				ESP_LOGI(TAG,"Managed Mode set to %d\n",managed.mode);
				writeManaged = true;
			}
			item = cJSON_GetObjectItem(root,"managed_setpoint");
			if (item) {
				managed.setpoint = item->valueint;
				ESP_LOGI(TAG,"Managed setpoint set to %d\n",managed.setpoint);
				writeManaged = true;
			}
			item = cJSON_GetObjectItem(root,"managed_setpoint_unoccupied");
			if (item) {
				managed.setpoint_unoccupied = item->valueint;
				ESP_LOGI(TAG,"Managed setpoint_unoccupied set to %d\n",managed.setpoint_unoccupied);
				writeManaged = true;
			}
			item = cJSON_GetObjectItem(root,"override_setpoint");
			if (item) {
				managed.override_setpoint = item->valueint;
				ESP_LOGI(TAG,"Managed override setpoint set to %d\n",managed.override_setpoint);
				writeManaged = true;
			}
			item = cJSON_GetObjectItem(root,"override_time");
			if (item) {
				managed.override_time = item->valueint;
				ESP_LOGI(TAG,"Managed oveide time set to %d\n",managed.override_time);
				writeManaged = true;
			}
			item = cJSON_GetObjectItem(root,"ir_interval");
			if (item) {
				managed.ir_interval = item->valueint;
				ESP_LOGI(TAG,"Managed ir_interval set to %d\n",managed.ir_interval);
				writeManaged = true;
			}
			item = cJSON_GetObjectItem(root,"fwupdate");
			if (item) {
			  ota_update(NOSOCKET);
			}
			item = cJSON_GetObjectItem(root,"do_override");
			if (item) {
			  do_override();
			}
			item = cJSON_GetObjectItem(root,"calendar_override");
			if (item) {
				do_calendar_override(item->valueint);
				ESP_LOGI(TAG,"Calendar override %d seconds\n",item->valueint);
			}

			item = cJSON_GetObjectItem(root,"trip_ir");
			if (item) {
			  do_ir();
			}
			item = cJSON_GetObjectItem(root,"cancel_override");
			if (item) {
			  ESP_LOGI(TAG,"Ending Override");
			  override_end=0L;
				setLed(LED_STATE_OFF);
			}

			if (writeManaged) {
			  nvs_handle_t h;
			  nvs_open(STORAGE_NAMESPACE,NVS_READWRITE,&h);
			  ESP_ERROR_CHECK(nvs_set_blob(h,"managed",&managed,sizeof(managed)));
			  nvs_close(h);
			  should_set();
			}
			item = cJSON_GetObjectItem(root,"reboot");
			if (item) {
			  esp_restart();
			}
		createPacket(packet, power, mode, fan, temp);
		bkg_uart_xmit(packet,PACKET_LEN);
		if (root) cJSON_Delete(root); // Delete entire TREE
	}


}


static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
	char topic[60];

    // your_context_t *context = event->context;

	gpio_set_level(GPIO_LED,!gpio_get_level(GPIO_LED));
	vTaskDelay(100);
	gpio_set_level(GPIO_LED,!gpio_get_level(GPIO_LED));
    
	
	switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            snprintf(topic,59,"facility/minisplit/request/%s",hostname);
            int msg_id = esp_mqtt_client_subscribe(client,topic, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client,"facility/alarm/system", 0);

            //msg_id = esp_mqtt_client_subscribe(client, "relay1", 0);
            //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            //msg_id = esp_mqtt_client_subscribe(client, "#", 0);
            //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            //msg_id = esp_mqtt_client_subscribe(client, "#", 3);
            //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;


        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
						printf("Entering deep sleep\n");
						ESP_LOGI(TAG, "Entering deep sleep\n");
// TODO BUG BOMB BKG FIXME
#if 0
						esp_wifi_disconnect();
						printf("Wifi stop\n");
						esp_wifi_stop();
						esp_deep_sleep(1000LL);
						//mqtt_restart();
			
						NETWORK_RESTART();
#endif
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
						// Get the shadow
            //msg_id = esp_mqtt_client_publish(client, CAT3_STR("$aws/things/", THING_NAME, "/shadow/get"), "", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
						//sleep_then_reboot();
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA offset %d total %u",event->current_data_offset,event->total_data_len);
            if (event->current_data_offset > 0) {
							ESP_LOGE(TAG, "MQTT TRUNCATED - INCREASE MAX LEN");
						}
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
						if (event->current_data_offset == 0) {
							got_msg(event->topic,event->topic_len,event->data,event->data_len);
							}
            break;
		case MQTT_EVENT_BEFORE_CONNECT:
			ESP_LOGI(TAG,"MQTT_EVENT_BEFORE_CONNECT");
			break;
		//case MQTT_EVENT_DELETED:
		//	ESP_LOGI(TAG,"MQTT_EVENT_DELETED");
		//	break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR last_tls=%d tls_stack=%d tls_flags=0x%x err_type=%d conn_rc=%d ",
				event->error_handle->esp_tls_last_esp_err,
				event->error_handle->esp_tls_stack_err,
				event->error_handle->esp_tls_cert_verify_flags,
				event->error_handle->error_type,
				event->error_handle->connect_return_code
				);
						//mqtt_restart();
						// Cannot be stopped inside MQTT task - schedule elsewhere??
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtts://mqtt.lan.makeitlabs.com:8883",
        .event_handle = mqtt_event_handler,
		.client_id = hostname,
        .client_cert_pem = (const char *)client_cert_pem_start,
        .client_key_pem = (const char *)client_key_pem_start,
				.cert_pem = (const char *) CA_cert_pem_start,
				.buffer_size=1500,
      .skip_cert_common_name_check = true,
    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK( esp_mqtt_client_start(client));
	
		client_global=client;
}
