#include 	<stdio.h>
#include	<string.h>
#include "esp_wifi_types.h"
#include	"esp_event.h"	//	for usleep

#include	<esp_log.h>
#include <math.h>

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
#include "uart.h"

#include "esp_ota_ops.h"
#include <esp_http_server.h>

// 15 is a STRAPPING PIN and outputs PWM on boot. It can be a button if DISCONNECTED during boot
#define GPIO_INPUT_BUTTON    15
#define GPIO_IO_TX2 17
#define GPIO_IO_RX2 16
#define GPIO_OUTPUT_NEOPIXEL 22
#define GPIO_INPUT_IR    21


#define ESP_INTR_FLAG_DEFAULT 0
#include "esp_log.h"
#include "esp_console.h"

#include "data.h"
#include "mqtt.h"
#include "ota.h"
#include "neopixel.h"


time_t	override_end=0L;
void wifi_init_sta(void);
char hostname[20];
managed_settings_t managed;
static const char *TAG = "Minisplit";

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle led_evt_queue = NULL;

#define NR_LED (1)
uint32_t		pixels[NR_LED];
pixel_settings_t px;
#define NEOPIXEL_RMT_CHANNEL (0)
#define NEOPIXEL_WS2812

void setLed(short led) {
    xQueueSend(led_evt_queue, &led, portMAX_DELAY );
}

void set_pixels(uint16_t r, uint16_t g, uint16_t b, uint16_t l) {
	int i;
	for (i=0;i<NR_LED;i++) {
		np_set_pixel_rgbw_level(&px, i , r,g,b,0,l);
	}
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
}

void init_leds() {
	int rc;
	rc = neopixel_init(GPIO_OUTPUT_NEOPIXEL, NEOPIXEL_RMT_CHANNEL);
	if (rc < 0)
		ESP_LOGE(TAG, "neopixel_init rc = %d", rc);

	px.pixels = (uint8_t *)pixels;
	px.pixel_count = NR_LED;
#ifdef	NEOPIXEL_WS2812
	strcpy(px.color_order, "GRB");
#else
	strcpy(px.color_order, "GRBW");
#endif

	memset(&px.timings, 0, sizeof(px.timings));
	px.timings.mark.level0 = 1;
	px.timings.space.level0 = 1;
	px.timings.mark.duration0 = 12;
#ifdef	NEOPIXEL_WS2812
	px.nbits = 24;
	px.timings.mark.duration1 = 14;
	px.timings.space.duration0 = 7;
	px.timings.space.duration1 = 16;
	px.timings.reset.duration0 = 600;
	px.timings.reset.duration1 = 600;
#endif
#ifdef	NEOPIXEL_SK6812
	px.nbits = 32;
	px.timings.mark.duration1 = 12;
	px.timings.space.duration0 = 6;
	px.timings.space.duration1 = 18;
	px.timings.reset.duration0 = 900;
	px.timings.reset.duration1 = 900;
#endif
	px.brightness = 0x80;
	np_show(&px, NEOPIXEL_RMT_CHANNEL);


	np_set_pixel_rgbw_level(&px, 0 , 64,64,64,0,255);
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	vTaskDelay(250 / portTICK_PERIOD_MS);
	
	np_set_pixel_rgbw_level(&px, 0 , 128,128,0,0,255);
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	vTaskDelay(250 / portTICK_PERIOD_MS);

	np_set_pixel_rgbw_level(&px, 0 , 0,0,0,0,255);
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
}

// Use led_evt_queue to change status
void led_task() {
  short led_evt;
	short led_state=0;
  init_leds();

	// These have to be done in priority order
	//
  uint32_t waitTime=30000;
  for(;;) {
	  if (xQueueReceive(led_evt_queue, &led_evt, waitTime / portTICK_PERIOD_MS)) {
    waitTime=30000;

		ESP_LOGI(TAG,"Got LED Event 0x%x",led_evt);
		// Change or Event Request
	  if (led_evt & LED_FLAG_BUTTON) {
			np_set_pixel_rgbw_level(&px, 0 , 128,128,0,0,255);
			np_show(&px, NEOPIXEL_RMT_CHANNEL);
			
			waitTime = 500;
			continue;
		}
    else if (led_evt & LED_FLAG_IR) {
			np_set_pixel_rgbw_level(&px, 0 , 128,128,128,0,255);
			np_show(&px, NEOPIXEL_RMT_CHANNEL);
			
			waitTime = 500;
			continue;
		}

		// Change state if requried
		if (led_evt & LED_STATE_MASK)
			led_state = led_evt & LED_STATE_MASK;
		// Change state if requried
	}

		// Make sure current state is applied
	  switch (led_state & LED_STATE_MASK) {
			case LED_STATE_OVERRIDE_HEAT:
				np_set_pixel_rgbw_level(&px, 0 , 128,0,0,0,255);
			break;

			case LED_STATE_OVERRIDE_COOL:
				np_set_pixel_rgbw_level(&px, 0 , 0,0,128,0,255);
			break;

			case LED_STATE_UNOCCUPIED:
				np_set_pixel_rgbw_level(&px, 0 , 0,64,0,0,255);
			break;

			default:
				np_set_pixel_rgbw_level(&px, 0 , 0,0,0,0,255);
			break;
		}
		np_show(&px, NEOPIXEL_RMT_CHANNEL);
	}

}


static void IRAM_ATTR gpio_isr_handler(void* arg) {
	//printf("INT should go to xQueue\r\n");
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			ESP_LOGI(TAG,"Event from GPIO 0x%x",io_num);
			if (io_num == GPIO_INPUT_BUTTON)
				do_override();
			if (io_num == GPIO_INPUT_IR)
				do_ir();
	}
    }
}

/*** WIFI AP  ****/

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}


#define EXAMPLE_ESP_WIFI_SSID "MakeIt Xmas Star"
// Remove this line or set your SSID!
int SSIDPWTEST[1/(sizeof(SSID_PASSWORD)-1)];
int SSIDTEST[1/(sizeof(SSID)-1)];

wifi_config_t wifi_config_ap = {
    .ap = {
        .ssid = SSID,
        .ssid_len = strlen(SSID),
        .channel = 1,
        .password = SSID_PASSWORD,
        .max_connection = 4,
        .authmode = WIFI_AUTH_WPA_WPA2_PSK
    },
};


void SetupAP() {
esp_netif_create_default_wifi_ap();
ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                    ESP_EVENT_ANY_ID,
                                                    &wifi_event_handler,
                                                    NULL,
                                                    NULL));
wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
ESP_ERROR_CHECK(esp_wifi_init(&cfg));

if (strlen(SSID_PASSWORD) == 0) {
    wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
}
ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));

ESP_ERROR_CHECK(esp_wifi_start());
}

static httpd_handle_t server = NULL;
/*** WIFI AP  ****/
extern	void
app_main (void)
{
	ota_preflight();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    ESP_LOGI(TAG,"LED Init");
  nvs_handle_t h;
  nvs_open(STORAGE_NAMESPACE,NVS_READONLY,&h);
  size_t size=19;
  err = nvs_get_str(h,"hostname",hostname,&size);
  if (err) {
	  ESP_LOGE(TAG,"NO HOSTNAME SET -default to \"minisplit\"");
	  strcpy(hostname,"minisplit");
  }
  memset((void *) &managed,0,sizeof(memset));
  size = sizeof(managed);
  err = nvs_get_blob(h,"managed",&managed,&size);
  if (err) {
	  ESP_LOGE(TAG,"NO Managed Settings");
  }
  nvs_close(h);
    esp_app_desc_t running_app_info;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
	ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }
  ESP_LOGI(TAG,"Hostname is \"%s\"",hostname);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    led_evt_queue = xQueueCreate(10, sizeof(short));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());
    //ESP_ERROR_CHECK(esp_wifi_start());
    //
    wifi_init_sta();
    //
    //SetupAP();
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    gpio_set_direction(GPIO_LED,GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED,1);

    // GPIO 0 is "Overide" button
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    io_conf.pin_bit_mask = 1<< GPIO_INPUT_BUTTON;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_BUTTON, gpio_isr_handler, (void*) GPIO_INPUT_BUTTON);

    // COnfigure IR Sensor
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    io_conf.pin_bit_mask = 1<< GPIO_INPUT_IR;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IR, gpio_isr_handler, (void*) GPIO_INPUT_IR);
	//test_neopixel();
    
    /* Start the server for the first time */
    server = start_webserver();



  bkg_uart_init();
  //xTaskCreatePinnedToCore(&test_neopixel, "Neopixels", 8192, NULL, 55, NULL,1);
  xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)0L, 5, NULL);
  xTaskCreate(&console, "Console", 8192, NULL, 1, NULL);
  xTaskCreate(&uart_monitor_thread, "uart_read", 8192, NULL, 1, NULL);
  xTaskCreate(gpio_task_example, "gpio_task",4096, NULL, 10, NULL);
  xTaskCreate(led_task, "led_task",4096, NULL, 10, NULL);
  ota_init();
  mqtt_app_start();
}

