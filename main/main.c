#include <stdio.h>
#include <ep_wifi.h>
#include <esp32-dht11.h>
#include <driver/gpio.h>
#include <mqtt_client.h>
#include "main.h"
#include <freertos/portmacro.h>
#include <hal/adc_types.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>
#include <esp_adc/adc_continuous.h>
#include <ultrasonic.h>
#include "PID.h"

#define CONFIG_DHT11_PIN GPIO_NUM_4
#define CONFIG_CONNECTION_TIMEOUT 5
#define MAX_DISTANCE_CM 50 //maximum US

#define IR_TOPIC "IRarray"
#define US0_TOPIC "us0"
#define US1_TOPIC "us1"
#define US2_TOPIC "us2"
#define US3_TOPIC "us3"
#define MQ135_TOPIC "mq135_0"
#define MQ2_TOPIC "mq2_1"
#define IR0_PIN 19
#define IR1_PIN 21
#define IR2_PIN 22
#define IR3_PIN 23

#define TRIGGER_GPIO_0 25
#define ECHO_GPIO_0 16

#define TRIGGER_GPIO_1 26
#define ECHO_GPIO_1 17

#define TRIGGER_GPIO_2 27 
#define ECHO_GPIO_2 5

#define TRIGGER_GPIO_3 14
#define ECHO_GPIO_3 18

#define T
/*Structs*/
typedef struct 
{    
    int pwma_pin;
    int pwmb_pin;
    
    int a1_pin;
    int a2_pin;
    
    int b1_pin;
    int b2_pin;
    
    int curr_speed1;
    int curr_speed2;

    int max_speed1;
    int max_speed2;

    char motor_topic[32];
} motor_pair_t;

/*Global variables*/
esp_mqtt_client_handle_t mqtt_client;
bool mqtt_ready = false;
int retries = 0;
char temperature[6];
char humidity[6];
char ir_array[5] = "0000";
float us0_measure = 0.0;
float us1_measure = 0.0;
float us2_measure = 0.0;
float us3_measure = 0.0;
float IR_sum = 0.0;
PIDController main_controller =
{
    .Kd=0.01,
    .Kp=0.6,
    .Ki=0.00,
    .Ts= 0.005,
    .limMax = 100,
    .limMin = -100,
    .limMaxInt = 25,
    .limMinInt = -25,
    .prevError = 0.0

};
motor_pair_t motor_pair =
{
        .pwma_pin = GPIO_NUM_17,
        .pwmb_pin = GPIO_NUM_0,
        .a1_pin = GPIO_NUM_5,
        .a2_pin = GPIO_NUM_12,
        .b1_pin = GPIO_NUM_2,
        .b2_pin = GPIO_NUM_15,
        .max_speed1 = 35
};
    adc_oneshot_unit_handle_t adc_instance;
/*MOTORS*/
void config_motor_pair(motor_pair_t motor_pair);
void set_motor_pair(motor_pair_t motor_pair, float pwma, int a1, int a2, float pwmb, int b1, int b2);
/*MQTT*/
void mqtt_event_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data);
void mqtt_thread(void* args);
/*SENSORS*/
void update_dht_data(dht11_t* dht11_sensor,char* message1,char* message2);
void update_ir_data();
/*MISC*/
void setup_gpio();
int get_message_len(char* msg);
void init_adc(void);
float PIDController_Update(PIDController *pid);
void pid_thread(void* args);

void app_main(void)
{
    int count = 0;
    dht11_t dht11_sensor;
    dht11_sensor.dht11_pin = CONFIG_DHT11_PIN;
    
    ultrasonic_sensor_t us0 = 
    {
        .trigger_pin = TRIGGER_GPIO_0,
        .echo_pin = ECHO_GPIO_0
    }; 

    ultrasonic_sensor_t us1 = 
    {
        .trigger_pin = TRIGGER_GPIO_1,
        .echo_pin = ECHO_GPIO_1
    };

    ultrasonic_sensor_t us2 = 
    {
        .trigger_pin = TRIGGER_GPIO_2,
        .echo_pin = ECHO_GPIO_2
    };

    ultrasonic_sensor_t us3 = 
    {
        .trigger_pin = TRIGGER_GPIO_3,
        .echo_pin = ECHO_GPIO_3
    };   
    gpio_reset_pin(GPIO_NUM_14);
    gpio_reset_pin(GPIO_NUM_2);
    gpio_reset_pin(GPIO_NUM_15);
    ultrasonic_init(&us0);
    //ultrasonic_init(&us1);
    //ultrasonic_init(&us2);
    ultrasonic_init(&us3);


    
    access_point_t beacon = 
    {
    .ssid = "19 Dollar Fortnite Card",
    .password = "whowantsit?"
    };


    config_motor_pair(motor_pair);
    init_wifi('s',beacon);
    init_adc();
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(mqtt_thread,"mqtt thread",4096,NULL,1,NULL);
    //xTaskCreate(pid_thread,"pid thread",4096,NULL,1,NULL);
    memset(temperature,'\0',4);
    memset(humidity,'\0',4);
    while(1)
    {
      update_ir_data();

      ultrasonic_measure(&us0, MAX_DISTANCE_CM, &us0_measure);
      //ultrasonic_measure(&us1, MAX_DISTANCE_CM, &us1_measure);
      //ultrasonic_measure(&us2, MAX_DISTANCE_CM, &us2_measure);
      ultrasonic_measure(&us3, MAX_DISTANCE_CM, &us3_measure);
      if(count == 2000) 
      {
        update_dht_data(&dht11_sensor,temperature,humidity);
        count = 0;
      } 
      else count = count + 50;
      vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

void setup_gpio()
{
    gpio_set_direction(IR0_PIN,GPIO_MODE_INPUT);
    gpio_set_direction(IR1_PIN,GPIO_MODE_INPUT);
    gpio_set_direction(IR2_PIN,GPIO_MODE_INPUT);
    gpio_set_direction(IR3_PIN,GPIO_MODE_INPUT);
}

void mqtt_thread(void* args)
{
    esp_mqtt_client_config_t client_config =
    {
        .broker.address.uri = "mqtt://192.168.12.1:1886",
        .credentials.username = "Z.A.K.U"
    };
    mqtt_client = esp_mqtt_client_init(&client_config);
    esp_mqtt_client_register_event(mqtt_client,ESP_EVENT_ANY_ID,mqtt_event_handler,&mqtt_client);
    esp_mqtt_client_start(mqtt_client);

    esp_mqtt_client_subscribe(mqtt_client,MQTT_DHT_TEMP,0);
    esp_mqtt_client_subscribe(mqtt_client,MQTT_DHT_HUM,0);
    esp_mqtt_client_subscribe(mqtt_client,IR_TOPIC,0);
    
    esp_mqtt_client_subscribe(mqtt_client,US0_TOPIC,0);
    esp_mqtt_client_subscribe(mqtt_client,US1_TOPIC,0);
    esp_mqtt_client_subscribe(mqtt_client,US2_TOPIC,0);
    esp_mqtt_client_subscribe(mqtt_client,US3_TOPIC,0);

    esp_mqtt_client_subscribe(mqtt_client,MQ135_TOPIC,0);
    esp_mqtt_client_subscribe(mqtt_client,MQ2_TOPIC,0);
    char msg[10];
    
    while(1)
    {
        if(mqtt_ready)
        {
                esp_mqtt_client_publish(mqtt_client,MQTT_DHT_TEMP,temperature,4,0,0); 
                esp_mqtt_client_publish(mqtt_client,MQTT_DHT_HUM,humidity,4,0,0);
                esp_mqtt_client_publish(mqtt_client,IR_TOPIC,ir_array,5,0,0);

                memset(msg,'\0',10);
                sprintf(msg,"%.2f",us0_measure*100);
                esp_mqtt_client_publish(mqtt_client,US0_TOPIC,msg,10,0,0);
                memset(msg,'\0',10);
                sprintf(msg,"%.2f",us1_measure*100);
                esp_mqtt_client_publish(mqtt_client,US1_TOPIC,"-1.00\0",10,0,0);
                memset(msg,'\0',10);
                sprintf(msg,"%.2f",us2_measure*100);
                esp_mqtt_client_publish(mqtt_client,US3_TOPIC,"-1.00\0",10,0,0);
                memset(msg,'\0',10);
                sprintf(msg,"%.2f",us3_measure*100);
                esp_mqtt_client_publish(mqtt_client,US2_TOPIC,msg,10,0,0);


                int sensorOUT= 0;
                adc_channel_t target_channel;
                adc_unit_t target_unit = ADC_UNIT_1;

                adc_continuous_io_to_channel(GPIO_NUM_35,&target_unit,&target_channel);
                ESP_ERROR_CHECK(adc_oneshot_read(adc_instance,target_channel,&sensorOUT));
                memset(msg,'\0',10);
                sprintf(msg,"%d",sensorOUT);
                esp_mqtt_client_publish(mqtt_client,MQ135_TOPIC,msg,10,0,0);

                adc_continuous_io_to_channel(GPIO_NUM_32,&target_unit,&target_channel);
                ESP_ERROR_CHECK(adc_oneshot_read(adc_instance,target_channel,&sensorOUT));
                memset(msg,'\0',10);
                sprintf(msg,"%d",sensorOUT);
                esp_mqtt_client_publish(mqtt_client,MQ2_TOPIC,msg,10,0,0);
        } 
        vTaskDelay(50/portTICK_PERIOD_MS);
    } 
}
void pid_thread(void* args)
{
    while(1)
    {
        PIDController_Update(&main_controller);
        float speed1 = motor_pair.max_speed1 - main_controller.out;
        float speed2 = motor_pair.max_speed1 + main_controller.out ;
        if(speed1 > 100) speed1 = 100;
        else if(speed1 < 0) speed1 = 0;
        
        if(speed2 > 100) speed2 = 100;
        else if(speed2 < 0) speed2 = 0;

        set_motor_pair(motor_pair,speed1,0,1,speed2,0,1);
        //printf("%.2f + %.2f\n",speed1,speed2);
        vTaskDelay(5);
    }
}

void mqtt_event_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    switch(event_id)
	{
	case MQTT_EVENT_CONNECTED:
        mqtt_ready = true;
        printf("MQTT CONN ESTABLISHED\n");
		break;
	case MQTT_EVENT_DISCONNECTED:
        mqtt_ready = false;
        printf("MQTT CONN SEVERED\n");
        esp_mqtt_client_reconnect(mqtt_client);
        retries++;
        vTaskDelay(200/portTICK_PERIOD_MS);
		break;
	}
}

int get_message_len(char* msg)
{
    int index = 0;
    while(*(msg+index)!= '\0') index++;
    return index;
}

void update_dht_data(dht11_t* dht11_sensor,char* message1,char* message2)
{
      if(!dht11_read(dht11_sensor, CONFIG_CONNECTION_TIMEOUT))
      {  
        sprintf(message1,"%.2f",dht11_sensor->temperature);
        sprintf(message2,"%.2f",dht11_sensor->humidity );
      }
}

void init_adc(void)
{

    adc_oneshot_unit_init_cfg_t instance_config =
    {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&instance_config,&adc_instance));
    adc_oneshot_chan_cfg_t channel_config = 
    {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_11
    };
    adc_channel_t target_channel;
    adc_unit_t target_unit = ADC_UNIT_1;
    int testOUT= 0;

    adc_continuous_io_to_channel(GPIO_NUM_35,&target_unit,&target_channel);
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_instance,target_channel,&channel_config));
    adc_continuous_io_to_channel(GPIO_NUM_34,&target_unit,&target_channel);
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_instance,target_channel,&channel_config));
    adc_continuous_io_to_channel(GPIO_NUM_32,&target_unit,&target_channel);
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_instance,target_channel,&channel_config));
}

void config_motor_pair(motor_pair_t motor_pair)
{
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); 
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motor_pair.pwma_pin);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, motor_pair.pwmb_pin);
    printf("ok 1\n");
    gpio_reset_pin(motor_pair.a1_pin);
    gpio_reset_pin(motor_pair.a2_pin);
    gpio_reset_pin(motor_pair.b1_pin);
    gpio_reset_pin(motor_pair.b2_pin);
    printf("ok 2\n");
    
    gpio_set_direction(motor_pair.a1_pin,GPIO_MODE_OUTPUT);
    gpio_set_direction(motor_pair.a2_pin,GPIO_MODE_OUTPUT);
    gpio_set_direction(motor_pair.b1_pin,GPIO_MODE_OUTPUT);
    gpio_set_direction(motor_pair.b2_pin,GPIO_MODE_OUTPUT);
    mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,0);
    mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,0);
  
}

void set_motor_pair(motor_pair_t motor_pair, float pwma, int a1, int a2, float pwmb, int b1, int b2)
{
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,pwma);
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,pwmb);
  gpio_set_level(motor_pair.a1_pin,a1);
  gpio_set_level(motor_pair.a2_pin,a2);
  gpio_set_level(motor_pair.b1_pin,b1);
  gpio_set_level(motor_pair.b2_pin,b2);
}

void update_ir_data()
{
    ir_array[0] = gpio_get_level(IR0_PIN) + '0';
    ir_array[1] = gpio_get_level(IR1_PIN) + '0';
    ir_array[2] = gpio_get_level(IR3_PIN) + '0';
    ir_array[3] = gpio_get_level(IR2_PIN) + '0';
    ir_array[4] = '\0';
    IR_sum = gpio_get_level(IR0_PIN) * 15 + gpio_get_level(IR1_PIN) * 10 - 10 * gpio_get_level(IR3_PIN)  - 15* gpio_get_level(IR2_PIN);
    
}
float PIDController_Update(PIDController *pid) 
{
	/*
	* Error signal
	*/
    
    float measurement = IR_sum;
    float error = - measurement;
	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;
	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * (pid->Ki) * (pid->Ts)*(error + pid->prevError);
	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }
	/*
	* Derivative (band-limited differentiator)
	*/
    pid->differentiator = ( 0.5 * pid->Kd * (measurement - pid->prevMeasurement) / pid->Ts);	/* Note: derivative on measurement, therefore minus sign in front of equation! */
	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }
	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;
	/* Return controller output */
    return pid->out;
}
