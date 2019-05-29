/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "mqtt_client.h"
#include "mqtt_config.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "math.h"
#include <string.h>

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define MQTT_TARGET_SPEED_TOPIC "scottessner/feeds/fan-speed-target"

#define ADC_PIN 34

SemaphoreHandle_t print_mux = NULL;

struct pwm_struct {
    uint16_t on;
    uint16_t off;
};

int motor_speed = 0;
float smoker_temperature = 0;
float meat_temperature = 0;

bool mqtt_connected = false;
bool network_connected = false;

static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_register(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t byte)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write_byte(cmd, byte, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_set_motor_speed(i2c_port_t i2c_num, uint16_t speed)
{
    int ret = ESP_OK;
    struct pwm_struct pwm_cmd = {0, speed};
    struct pwm_struct in2_cmd = {0, 0};
    struct pwm_struct in1_cmd = {4096, 0};

    //-----------------------------------------------------------------
    //Motor 1
    ret = i2c_master_write_slave_reg(i2c_num, 0x60, 0x26, &pwm_cmd, 4);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_master_write_slave_reg(i2c_num, 0x60, 0x2A, &in2_cmd, 4);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_master_write_slave_reg(i2c_num, 0x60, 0x2E, &in1_cmd, 4);
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    ESP_LOGI(TAG, "SDA=%d, SCL=%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqtt_connected = true;

            msg_id = esp_mqtt_client_subscribe(client, MQTT_TARGET_SPEED_TOPIC, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = false;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA: {
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            char topic_local[event->topic_len];
            char data_local[event->data_len];

            sprintf(topic_local, "%.*s", event->topic_len, event->topic);
            sprintf(data_local, "%.*s", event->data_len, event->data);

            printf("Const(%d): %s\n", strlen(MQTT_TARGET_SPEED_TOPIC), MQTT_TARGET_SPEED_TOPIC);
            printf("Variable(%d): %s\n", event->topic_len, topic_local);
            printf("Compare Topic Result: %d\n", strncmp(topic_local, MQTT_TARGET_SPEED_TOPIC, 34));
            if (strncmp(topic_local, MQTT_TARGET_SPEED_TOPIC, 34) == 0) {
                motor_speed = strtol(data_local, NULL, 10);
                printf("Motor Speed String: %s, Motor Speed Int: %d", data_local, motor_speed);
                printf("Motor Speed Updated to: %d\n", motor_speed);
            }
            break;
        }
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
            .uri = "mqtt://io.adafruit.com",
            .username = "scottessner",
            .password = "d91746e2195148049abd59da8eaf662c",
            .event_handle = mqtt_event_handler,
            // .user_context = (void *)your_context
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
    return client;
}

static esp_err_t i2c_motor_init(i2c_port_t i2c_num)
{
    int ret;
    int freq = 50;
    uint8_t old_mode;
    uint8_t val;

    val = 0x00;
    ret = i2c_master_write_slave_reg(i2c_num, 0x60, 0x00, &val, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t prescale =  (uint8_t)(25000000.0 / 4096.0 / freq + 1);

    //Read current state of mode1
    ret = i2c_master_read_slave_reg(i2c_num, 0x60, 0x00, &old_mode, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    printf("Existing Mode1 Value: %x\n", old_mode);

    //Get value to turn on sleep and write it to mode 1
    val = ((old_mode & 0x7F) | 0x10);
    ret = i2c_master_write_slave_reg(i2c_num, 0x60, 0x00, &val, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    printf("Turned on Sleep setting Mode1 to: %x\n", val);

    //Set new prescale
    ret = i2c_master_write_slave_reg(i2c_num, 0x60, 0xFE, &prescale, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    printf("Wrote New Prescale Value of %x\n", prescale);

    //Set Mode1 back (turning off sleep)
    ret = i2c_master_write_slave_reg(i2c_num, 0x60, 0x00, &old_mode, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    printf("Woke Up with Value of %x\n", old_mode);
    vTaskDelay(5 / portTICK_RATE_MS);

    //Set autoincrement and reset
    val = old_mode | 0xA1;
    ret = i2c_master_write_slave_reg(i2c_num, 0x60, 0x00, &val, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    printf("Turned on AutoIncrement with %x\n", val);
    return ret;

}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id)
    {
        case(SYSTEM_EVENT_STA_GOT_IP):
            network_connected = true;
            printf("Handled Station Got IP Event\n");
            break;
        default:
            break;
    }
    return ESP_OK;
}

void connect_wifi(void){
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
            .sta = {
                    .ssid = "ssessner",
                    .password = "scottandsarah",
                    .bssid_set = false
            }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static void i2c_test_task(void *arg)
{

    int ret;
    uint32_t task_idx = (uint32_t)arg;
    int cnt = 0;

    i2c_motor_init(I2C_MASTER_NUM);

    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = i2c_set_motor_speed(I2C_MASTER_NUM, motor_speed);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER WRITE MOTOR\n", task_idx);
            printf("*******************\n");
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
        //---------------------------------------------------
    }
    vTaskDelete(NULL);
}

static void mqtt_task(void *arg)
{
    uint32_t task_idx = (uint32_t)arg;

    connect_wifi();

    while(!network_connected)
    {
        vTaskDelay(100/ portTICK_RATE_MS);
    }

    esp_mqtt_client_handle_t client = mqtt_app_start();

    int16_t smoker_int = (int16_t) smoker_temperature;
    int16_t meat_int = (int16_t) meat_temperature;

    char* speed = (char *)malloc(64);
    char* smoker = (char *)malloc(64);
    char* meat = (char *)malloc(64);

    printf("S: %d, M: %d\n", smoker_int, meat_int);

    while (1) {
        printf("*******************\n");
        printf("TASK[%d]  MQTT PUBLISH\n", task_idx);
        printf("*******************\n");
        if(mqtt_connected) {

            smoker_int = (int16_t) smoker_temperature;
            meat_int = (int16_t) meat_temperature;

            printf("Speed Int: %d\n", motor_speed);
            printf("Smoker Temp Int: %d\n", smoker_int);
            printf("Meat Temp Int: %d\n", meat_int);

            snprintf(speed, 64, "%d", motor_speed);
            snprintf(smoker, 64, "%d", smoker_int);
            snprintf(meat, 64, "%d", meat_int);

            printf("Speed: %s\n", speed);
            printf("Smoker: %s\n", smoker);
            printf("Meat: %s\n", meat);

            esp_mqtt_client_publish(client, "scottessner/feeds/speed", speed, 0, 0, 0);

            if(smoker_int > -1) {
                esp_mqtt_client_publish(client, "scottessner/feeds/smoker_temp", smoker, 0, 0, 0);
            }

            if(meat_int > -1) {
                esp_mqtt_client_publish(client, "scottessner/feeds/meat_temp", meat, 0, 0, 0);
            }
        }
        vTaskDelay((15000 * (task_idx + 1)) / portTICK_RATE_MS);
    }
}

static void analog_read_task(void *arg)
{

    uint32_t task_idx = (uint32_t)arg;

    float smoker_ref_res = 984579.77;
    float smoker_beta = 5447.86;
    float smoker_A = -0.4778775817E-3;
    float smoker_B = 3.460074606E-4;
    float smoker_C = -3.599084028E-7;

    float meat_ref_res = 1168038.07;
    float meat_beta = 5700.26;
    float meat_A = -0.3022536330E-3;
    float meat_B = 3.262681052E-4;
    float meat_C = -3.318749114E-7;

    //smoker
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

    //meat
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    float alpha = 0.001;

    float smoker_avg = adc1_get_raw(ADC1_CHANNEL_6);
    float meat_avg = adc1_get_raw(ADC1_CHANNEL_3);

    float smoker_resistance = 0;
    float meat_resistance = 0;

    float smoker_temperature_K;
    float meat_temperature_K;

    int val = 0;

    while (1) {

        for(int i=0; i<10; i++) {
            //smoker
            val = adc1_get_raw(ADC1_CHANNEL_6);
            smoker_avg = alpha * val + (1 - alpha) * smoker_avg;
            smoker_resistance = 82000 / (4095 / smoker_avg - 1);

            val = adc1_get_raw(ADC1_CHANNEL_3);
            meat_avg = alpha * val + (1 - alpha) * meat_avg;
            meat_resistance = 82000 / (4095 / meat_avg - 1);

            vTaskDelay((20 * (task_idx + 1)) / portTICK_RATE_MS);

        }

        smoker_temperature_K = 1 / (smoker_A + smoker_B * log(smoker_resistance) + smoker_C * pow(log(smoker_resistance), 3));
        meat_temperature_K = 1 / (meat_A + meat_B * log(meat_resistance) + meat_C * pow(log(meat_resistance), 3));

        smoker_temperature = 32 + 1.8 * (smoker_temperature_K - 273.15);
        meat_temperature = 32 + 1.8 * (meat_temperature_K - 273.15);

//        printf("Smoker: %f %f %f Meat: %f %f %f\n", smoker_avg, smoker_resistance, smoker_temperature, meat_avg, meat_resistance, meat_temperature);


        //---------------------------------------------------
    }
}

static void seven_seg_display_task(void *arg)
{

}

void app_main()
{
    print_mux = xSemaphoreCreateMutex();
//    ESP_ERROR_CHECK(i2c_slave_init());
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(mqtt_task, "mqtt_task_0", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(analog_read_task, "analog_task_0", 1024 * 2, (void *)0, 10, NULL);
}
