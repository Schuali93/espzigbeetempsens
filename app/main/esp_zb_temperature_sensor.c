/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_temperature_sensor Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_temperature_sensor.h"
#include "temp_sensor_driver.h"
#include "switch_driver.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "time.h"
#include "sys/time.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

// #if !defined ZB_ED_ROLE
// #error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
// #endif

#if !defined CONFIG_ZB_ZCZR
#error Define CONFIG_ZB_ZCZR in idf.py menuconfig to compile light (End Device) source code.
#endif

#define USER_LED_PIN GPIO_NUM_15

static const char *TAG = "ESP_ZB_TEMP_SENSOR";
static RTC_DATA_ATTR struct timeval s_sleep_enter_time;
static esp_timer_handle_t s_oneshot_timer;
static bool deep_sleep_enabled = false;

void enable_deep_sleep(bool enable)
{
    deep_sleep_enabled = enable;
}

static void configure_led(void)
{
    gpio_reset_pin(USER_LED_PIN);
    gpio_set_direction(USER_LED_PIN, GPIO_MODE_OUTPUT);
}

static void set_led_state(bool state)
{
    gpio_set_level(USER_LED_PIN, state ? 1 : 0);
}

static void s_oneshot_timer_callback(void *arg)
{
    /* Enter deep sleep */
    ESP_LOGI(TAG, "Enter deep sleep");
    gettimeofday(&s_sleep_enter_time, NULL);
    set_led_state(true);
    esp_deep_sleep_start();
}

static void zb_deep_sleep_init(void)
{
    /* Within this function, we print the reason for the wake-up and configure the method of waking up from deep sleep.
    This example provides support for two wake-up sources from deep sleep: RTC timer and GPIO. */

    /* The one-shot timer will start when the device transitions to the CHILD state for the first time.
    After a 5-second delay, the device will enter deep sleep. */

    set_led_state(false);

    const esp_timer_create_args_t s_oneshot_timer_args = {
        .callback = &s_oneshot_timer_callback,
        .name = "one-shot"};

    ESP_ERROR_CHECK(esp_timer_create(&s_oneshot_timer_args, &s_oneshot_timer));

    // Print the wake-up reason:
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - s_sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - s_sleep_enter_time.tv_usec) / 1000;
    esp_sleep_wakeup_cause_t wake_up_cause = esp_sleep_get_wakeup_cause();
    switch (wake_up_cause)
    {
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        ESP_LOGI(TAG, "Wake up from GPIO. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        ESP_LOGI(TAG, "Not a deep sleep reset");
        break;
    }

    /* Set the methods of how to wake up: */
    /* 1. RTC timer waking-up */
    const int wakeup_time_sec = 300;
    ESP_LOGI(TAG, "Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
}

static void zb_deep_sleep_start(void)
{
    /* Start the one-shot timer */
    const int before_deep_sleep_time_sec = 60;
    ESP_LOGI(TAG, "Start one-shot timer for %ds to enter the deep sleep", before_deep_sleep_time_sec);
    ESP_ERROR_CHECK(esp_timer_start_once(s_oneshot_timer, before_deep_sleep_time_sec * 1000000));
}

static int16_t zb_sensval_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}};

static void esp_app_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL)
    {
        /* Send report attributes command */
        esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
        report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
        esp_zb_lock_release();
        ESP_EARLY_LOGI(TAG, "Send 'report attributes' command");
    }
}

static void esp_app_temp_sensor_handler(float temperature, float humidity)
{
    int16_t measured_temp_value = zb_sensval_to_s16(temperature);
    int16_t measured_hum_value = zb_sensval_to_s16(humidity); // Assuming you have a similar function for humidity

    set_led_state(true);
    /* Update temperature sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);
    ESP_LOGI(TAG, "Measured temperature: %d.%02d°C", measured_temp_value / 100, measured_temp_value % 100);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &measured_temp_value, false);

    /* Update humidity sensor measured value */
    ESP_LOGI(TAG, "Measured humidity: %d.%02d%%", measured_hum_value / 100, measured_hum_value % 100);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &measured_hum_value, false);

    esp_zb_lock_release();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    temperature_sensor_config_t temp_sensor_config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(ESP_TEMP_SENSOR_MIN_VALUE, ESP_TEMP_SENSOR_MAX_VALUE);
    if (!is_inited)
    {
        ESP_RETURN_ON_ERROR(
            temp_sensor_driver_init(&temp_sensor_config, ESP_TEMP_SENSOR_UPDATE_INTERVAL, esp_app_temp_sensor_handler),
            TAG, "Failed to initialize temperature sensor");
        ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), esp_app_buttons_handler),
                            ESP_FAIL, TAG, "Failed to initialize switch driver");
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    bool is_joined = esp_zb_bdb_dev_joined();
    ESP_LOGI(TAG, "device joined: %i", is_joined);

    ESP_LOGI(TAG, "Signal type: %d", sig_type);
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT err_status: %d", err_status);
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
                if(deep_sleep_enabled)
                {
                    zb_deep_sleep_start();
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_STEERING err_status: %d", err_status);
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());           
                if(deep_sleep_enabled)
                {
                    zb_deep_sleep_start();
                }
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK)
        {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p))
            {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            }
            else
            {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        ESP_LOGI(TAG, "Can sleep");
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_zb_cluster_list_t *custom_temperature_sensor_clusters_create(esp_zb_temperature_sensor_cfg_t *temperature_sensor)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(temperature_sensor->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(temperature_sensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(temperature_sensor->temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_humidity_meas_cluster_cfg_t hum_meas_cluster = {
        .min_value = zb_sensval_to_s16(ESP_HUM_SENSOR_MIN_VALUE),
        .max_value = zb_sensval_to_s16(ESP_HUM_SENSOR_MAX_VALUE),
    };

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(&(hum_meas_cluster)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static esp_zb_ep_list_t *custom_temperature_sensor_ep_create(uint8_t endpoint_id, esp_zb_temperature_sensor_cfg_t *temperature_sensor)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, custom_temperature_sensor_clusters_create(temperature_sensor), endpoint_config);
    return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create customized temperature sensor endpoint */
    esp_zb_temperature_sensor_cfg_t sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    /* Set (Min|Max)MeasuredValure */
    sensor_cfg.temp_meas_cfg.min_value = zb_sensval_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    sensor_cfg.temp_meas_cfg.max_value = zb_sensval_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);
    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_temperature_sensor_ep_create(HA_ESP_SENSOR_ENDPOINT, &sensor_cfg);

    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    /* Config the reporting info for temperature */
    esp_zb_zcl_reporting_info_t temp_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 100,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_zb_zcl_update_reporting_info(&temp_reporting_info);

    /* Config the reporting info for humidity */
    esp_zb_zcl_reporting_info_t hum_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 100,
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_zb_zcl_update_reporting_info(&hum_reporting_info);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

void app_main(void)
{   
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    configure_led();
    enable_deep_sleep(true);

    if(deep_sleep_enabled)
    {
        zb_deep_sleep_init();
    }

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
