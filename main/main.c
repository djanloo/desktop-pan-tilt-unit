#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "esp_err.h"
#include "esp_log.h"

// For PWM
#include "driver/ledc.h"

#include "mouse_ble.h"
#include "djanloo_led_strip.h"

// Stop-range stuff
#define PIN_STOP_RANGE_POLAR 11
static TaskHandle_t button_task_handle = NULL;

// Device pin
#define PWM_FREQ_HZ    500
#define PWM_RES        LEDC_TIMER_14_BIT 
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_MODE       LEDC_LOW_SPEED_MODE

#define PWM_CHANNEL  LEDC_CHANNEL_0
#define PWM_GPIO     9

// Motor pins
static const gpio_num_t mot1 = GPIO_NUM_4;
static const gpio_num_t mot2 = GPIO_NUM_5;
static const gpio_num_t mot3 = GPIO_NUM_6;
static const gpio_num_t mot4 = GPIO_NUM_7;

// Position variables
int theta_steps = 0;
int phi_steps = 0;

// Mechanical parameters
#define STEPS_PER_ROTATION      2038
#define ACCELERATION_STEPS      200
#define MIN_ROT_FREQ            0.2 // Hz
#define MAX_ROT_FREQ            0.6 // Hz
#define POLAR_GEAR_RATIO        150.0/18.0
#define MIN_TIME_STEP_WAIT_US   200
#define MAX_POLAR_ANGLE         120 // Degrees


static bool calibrated = false;

// Step selection (full-step)
#define N_STEPS 4
const int seq[N_STEPS][4] = {
    {1,1,0,0},
    {0,1,1,0},
    {0,0,1,1},
    {1,0,0,1}
};

void stepMotor(int step) {
    int _step = step % N_STEPS;
    if(_step < 0) _step += N_STEPS;  // modulo corretto per negativo

    gpio_set_level(mot1, seq[_step][0]);
    gpio_set_level(mot2, seq[_step][1]);
    gpio_set_level(mot3, seq[_step][2]);
    gpio_set_level(mot4, seq[_step][3]);
}

void motorStop() {
    gpio_set_level(mot1, 0);
    gpio_set_level(mot2, 0);
    gpio_set_level(mot3, 0);
    gpio_set_level(mot4, 0);
}

typedef enum {
    CMD_STOP,
    CMD_RIGHT,
    CMD_LEFT,
    CMD_CALIB
} motor_command_t;

QueueHandle_t motor_cmd_queue;


int T_func(int step){
    // step normalizzato
    float alpha = (float)step / (float)ACCELERATION_STEPS;
    alpha = alpha < 1 ? alpha : 1;
    // step frequency in Hz
    float f_step_min = MIN_ROT_FREQ * STEPS_PER_ROTATION; // ~2038 Hz
    float f_step_max = MAX_ROT_FREQ * STEPS_PER_ROTATION; // ~3057 Hz

    // linear ramp
    float f = f_step_min + (f_step_max - f_step_min) * alpha;

    // convert to microseconds
    int t_us = (int) (1e6 / f);

    // clamp
    if(t_us < MIN_TIME_STEP_WAIT_US){
        t_us = MIN_TIME_STEP_WAIT_US;
    }
    
    // if (step % 100 == 0) ESP_LOGI("T_FUNC", "accel_step is %d \t t is %d us \t f = %lf", step, t_us, f);


    return t_us;
}

float get_theta(void){
    return - (double)theta_steps * 360.0 / (STEPS_PER_ROTATION * POLAR_GEAR_RATIO);
}

void rotate_motor(void *pvParams) {
    gpio_set_direction(mot1, GPIO_MODE_OUTPUT);
    gpio_set_direction(mot2, GPIO_MODE_OUTPUT);
    gpio_set_direction(mot3, GPIO_MODE_OUTPUT);
    gpio_set_direction(mot4, GPIO_MODE_OUTPUT);
    
    motor_command_t cmd;
    int sign = 0; //<! sign of the rotation
    int s = 0;  //<! number of steps in th current movement (for acceleration)
    while (1) {
        // Ricevi comandi con timeout
        if (xQueueReceive(motor_cmd_queue, &cmd, 0) == pdTRUE) {
            if (cmd == CMD_RIGHT) {
                // If the movement is changed, refresh the acceleration variable
                if (sign < 0) {s = 0;}
                sign = +1;
                printf("MOTORE: Avviato -- RIGHT\n");
            } else if (cmd == CMD_LEFT) {
                // If the movement is changed, refresh the acceleration variable
                if (sign > 0) {s = 0;}
                sign = -1;
                printf("MOTORE: Avviato -- LEFT\n");
            } else if (cmd == CMD_STOP) {
                sign = 0;
                s = 0;
                printf("MOTORE: STOP\n");
            } else if (cmd == CMD_CALIB){
                sign = 0;
                s = 0;
                printf("MOTORE: CALIBRAZIONE\n");

            }
        }

        // Max angle
        if ((get_theta() > MAX_POLAR_ANGLE) & (sign<0)) {
            cmd = CMD_STOP;
            sign = 0;
            s = 0;
            motorStop();
        }


        // Controllo motore
        if (sign != 0) {
            if (s % 500 == 0 ) ESP_LOGI("ROTATE MOTORS", "theta = %lf (sign = %d)", get_theta(), sign);
            for (int i = 0; i < N_STEPS; i++) {
                stepMotor(sign*i);
                theta_steps += sign;
                s++;
                esp_rom_delay_us(T_func(s));
            }
        }else if (cmd != CMD_CALIB){
            motorStop();
        }
        

    }
}

void on_left_button(void){
    motor_command_t cmd = CMD_LEFT;
    xQueueOverwrite(motor_cmd_queue, &cmd);
    set_led_color(0, 10, 0);
    ESP_LOGI("MAIN", "LEFT button was pressed");
}

void on_right_button(void){
    motor_command_t cmd = CMD_RIGHT;
    xQueueOverwrite(motor_cmd_queue, &cmd);
    set_led_color(0, 0, 10);
    ESP_LOGI("MAIN", "RIGHT button was pressed");
}

void on_middle_button(void){
    motor_command_t cmd = CMD_STOP;
    xQueueOverwrite(motor_cmd_queue, &cmd);
    set_led_color(10, 0, 0);
    ESP_LOGI("MAIN", "MIDDLE button was pressed");
}

void on_wheel_move(int wheel){

}

// Stop-range button
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(button_task_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void polar_stop(void *arg)
{
    while (1) {
        // dorme finché NON arriva l'interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        motor_command_t cmd = CMD_CALIB;
        xQueueOverwrite(motor_cmd_queue, &cmd);

        for (int i = 0; i < 100; i++) {
            stepMotor(-i);        
            esp_rom_delay_us(T_func(0));
        }

        printf("Polar stop pressed\n");
        theta_steps = 0;
        calibrated = true;
        vTaskDelay(pdMS_TO_TICKS(200));

        cmd = CMD_STOP;
        xQueueOverwrite(motor_cmd_queue, &cmd);

    }
}

void setup_stop_range_button(void)
{
    gpio_set_direction(PIN_STOP_RANGE_POLAR, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_STOP_RANGE_POLAR, GPIO_PULLUP_ONLY);

    gpio_set_intr_type(PIN_STOP_RANGE_POLAR, GPIO_INTR_LOW_LEVEL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_STOP_RANGE_POLAR, button_isr_handler, NULL);
}

void pwm_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_RES,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

     ledc_channel_config_t channel = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,        // start OFF
        .hpoint     = 0
    };
    ledc_channel_config(&channel);
}

void pwm_set_duty_percent(float percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    uint32_t max_duty = (1 << 14) - 1;
    uint32_t duty = (percent / 100.0f) * max_duty;

    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}


void go_home(){
    calibrated = false;

    motor_command_t cmd = CMD_CALIB;
    xQueueOverwrite(motor_cmd_queue, &cmd);

    int s = 0;
    while (!calibrated){
        stepMotor(s);
        s++;
        esp_rom_delay_us(T_func(0));
    } 
}


void app_main(void) {

    configure_led();
    init_ble();
    pwm_init();

    pwm_set_duty_percent(50);

    // Configure stop-range buttons
    xTaskCreate(polar_stop, "polar_stop", 2048, NULL, 10, &button_task_handle);
    setup_stop_range_button();

    // Configure stop-range switch
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_10, GPIO_PULLUP_ONLY);

    // Coda di lunghezza 1
    motor_cmd_queue = xQueueCreate(1, sizeof(motor_command_t));
    if (motor_cmd_queue == NULL) {
        printf("ERRORE: Impossibile creare la coda!\n");
        return;
    }
    
    // Crea task motore
    xTaskCreatePinnedToCore(rotate_motor, "rotate_motor", 4096, NULL, 2, NULL, 1);
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    go_home();


    while(1){
        // ESP_LOGI("BUTTON POLL", "lvl = %d", gpio_get_level(PIN_STOP_RANGE_POLAR));
        // ESP_LOGI("MAIN", "theta_step = %d theta = %lf", theta_steps, get_theta());
        pwm_set_duty_percent(100);
        vTaskDelay(pdMS_TO_TICKS(500));
        pwm_set_duty_percent(10);
        vTaskDelay(pdMS_TO_TICKS(2000));

    }
}
