#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "esp_log.h"
#include "mqtt_client.h"

//Temperature sensor settings
#define GPIO_DS18B20_0 (CONFIG_ONE_WIRE_GPIO)
#define MAX_DEVICES (8)
#define DS18B20_RESOLUTION (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD (10000) // milliseconds

//MQTT settings
#define MQTT_HOST CONFIG_MQTT_HOST
#define MQTT_PORT CONFIG_MQTT_PORT
#define MQTT_USER CONFIG_MQTT_USER
#define MQTT_PASSWORD CONFIG_MQTT_PASSWORD
#define MQTT_DEVICE_ID CONFIG_MQTT_DEVICE_ID

//WIFI settings
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASSWORD CONFIG_WIFI_PASSWORD
#define MAXIMUM_RETRY CONFIG_MAXIMUM_RETRY
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "ESP_TEMPERATURE";
static int s_retry_num = 0;

//************************************************************************
void app_main(void);
static void wifi_event_handler  (void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
static void wifi_start_connection(void);
void check_treshold_temp(float readings[], int num_devices);
//************************************************************************

void app_main(void)
{   
    //MQTT setup
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_INFO);
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_start_connection();

    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = MQTT_HOST,
        .port = MQTT_PORT,
        .username = MQTT_USER,
        .password = MQTT_PASSWORD
    }; 

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    //Configuration of DS18B20 1-Wire device
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    //Finding connected devices
    printf("Finding devices:\n");
    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    printf("Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");


    // Create DS18B20 devices on the 1-Wire bus
    DS18B20_Info * devices[MAX_DEVICES] = {0};
    for (int i = 0; i < num_devices; ++i)
    {
        DS18B20_Info * ds18b20_info = ds18b20_malloc();
        devices[i] = ds18b20_info;

        if (num_devices == 1)
        {
            printf("Single device optimisations enabled\n");
            ds18b20_init_solo(ds18b20_info, owb);       
        }
        else
        {
            ds18b20_init(ds18b20_info, owb, device_rom_codes[i]);
        }
        ds18b20_use_crc(ds18b20_info, true);
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
    }

    //Checking for parasitic power
    bool parasitic_power = false;
    ds18b20_check_for_parasite_power(owb, &parasitic_power);
    if (parasitic_power) {
        printf("Parasitic-powered devices detected");
        owb_use_parasitic_power(owb, parasitic_power);
    }

    // Reading temperature
    int errors_count[MAX_DEVICES] = {0};
    int sample_count = 0;

    if (num_devices > 0)
    {
        TickType_t last_wake_time = xTaskGetTickCount();

        while (1)
        {
            if(MQTT_EVENT_DISCONNECTED == true){
                esp_mqtt_client_reconnect(client);
            }

            ds18b20_convert_all(owb);
            ds18b20_wait_for_conversion(devices[0]);

            float readings[MAX_DEVICES] = { 0 };
            DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

            for (int i = 0; i < num_devices; ++i)
            {
                errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
            
            }

            // Print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C): sample %d\n", ++sample_count);
            
            
            for (int i = 0; i < num_devices; ++i)
            {
                char mqtt_path[50]="";
                char mqtt_reading[10];

                if (errors[i] != DS18B20_OK)
                {
                    ++errors_count[i];
                }

                printf("  %d: %.1f    %d errors\n", i, readings[i], errors_count[i]);
                snprintf(mqtt_path, 40, "ESP_TEMPERATURE/device_%d", i);
                sprintf(mqtt_reading, "%.2f", readings[i]);

                esp_mqtt_client_subscribe(client, mqtt_path, 0);
                esp_mqtt_client_publish(client, mqtt_path, mqtt_reading, 0, 0, 0);
            }

            check_treshold_temp(readings, num_devices);
            vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        }
    }
    else
    {
        printf("\nNo DS18B20 devices detected!\n");
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_start_connection(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {

        ESP_LOGI(TAG, "Connected to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASSWORD);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASSWORD);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void check_treshold_temp(float readings[], int num_devices)
{
    float highest_temperature = 0;
    
    for (int i = 0; i < num_devices; i++)
    {
        if (highest_temperature < readings[i])
        {
            highest_temperature = readings[i];
        }
    }

    if(highest_temperature >= 45){
            printf("Alarm ON!\n");
    }else{
            printf("Alarm OFF\n");
    }
}