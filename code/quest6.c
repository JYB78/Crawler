/*
*   EC444 Quest 6 Final Quest
*   Team 19
*   Jiayue Bai, Jess Barry, Zach Carlson
*
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include <stdlib.h>
#include <string.h>
#include "esp_types.h"
#include "esp_vfs_dev.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "sdkconfig.h"
#include "esp_adc_cal.h"


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <sys/param.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "freertos/portmacro.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"

#define HOST_IP_ADDR "192.168.1.48"
#define PORT 3333

static const char *TAG = "example";

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

//Wheel speed
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      1000
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    0
#define PCNT_THRESH0_VAL   0
#define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO      15 // Output GPIO of a sample 1 Hz pulse generator (I changed this from 18 to 15)
xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;



// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// 14-Segment Display
#define SLAVE_ADDR2                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

//front rf (Lidar)
#define SLAVE_ADDR1 0x62 // Default I2C Address of LIDAR-Lite.
#define REGISTER 0x00 // Register to write to initiate ranging.
#define VALUE 0x04 // Value to initiate ranging.
#define HIGH_LOW 0x8f // Register to get both High and Low bytes in 1 call. // not working for unknown reason

//Timer
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (0.1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

static const int RX_BUF_SIZE = 1024;

//Side sensor
#define TXD_PIN1 (GPIO_NUM_17)     //white
#define RXD_PIN1 (GPIO_NUM_16)     //green

//IR
#define RXD_PIN0 (GPIO_NUM_25)    //A1
#define TXD_PIN0 (GPIO_NUM_26)    //A0

// SPEED
#define SPEED_F_1 1285
#define SPEED_F_2 1250
#define SPEED_F_3 1200
#define SPEED_B_1 1550
#define SPEED_B_2 1580
#define SPEED_B_3 1610


int mode = 0;   // 0 = stop & 1 = auto & 2 = manual_forward & 3 = manual_backward

// wheelSpeed
uint32_t speed;
float mywheelSpeed;
float setWheel;

// Timer
float totalTime = 0;
float curSplit = 0;
int flag = 0; // for timer every 0.1 sec
int splitCount = 0;
char str_split[10] = "00:00";
int over = 0;

// Steering & Side RF
const int setPoint_side = 75;
int measuredSide = 0;
int mySteering;
uint32_t angle;
int turn;
int turnCount;

// PID for wheelSpeed
float previous_error, integral, error, dt, output, derivative;
const float Kp = 70;
const float Ki = 1;
const float Kd = 1;

// Front RF
int front_Dist;

// UDP socket
int sock;
char rx_buffer[128];
char addr_str[128];
int addr_family;
int ip_protocol;
struct sockaddr_in dest_addr;

// Alpha Display
int ind1 = 0, ind2 = 0, ind3 = 0, ind4 = 0;
static const uint16_t alphafonttable[] = {
  0b0000110000111111, // 0
  0b0000000000000110, // 1
  0b0000000011011011, // 2
  0b0000000010001111, // 3
  0b0000000011100110, // 4
  0b0010000001101001, // 5
  0b0000000011111101, // 6
  0b0000000000000111, // 7
  0b0000000011111111, // 8
  0b0000000011101111, // 9
  0b0000000000000000, // SPACE (10)
};

////////////////////////////////////////////////////////////////////////////////
// Setup / Utility  Functions ///////////////////////////////////////////////////

static void i2c_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12);    //Set GPIO 12 as PWM0A (ESC)
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, 33);    // Set GPIO 14 (steering)
}


static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void uart_init(void) {
  //For Side RF [UART_NUM_1]
    const uart_config_t uart_config1 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config1);
    uart_set_pin(UART_NUM_1, TXD_PIN1, RXD_PIN1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

  //For IR Receiver [UART_NUM_0]
  const uart_config_t uart_config2 = {
      .baud_rate = 1200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_2, &uart_config2);
  uart_set_pin(UART_NUM_2, TXD_PIN0, RXD_PIN0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_line_inverse(UART_NUM_2, UART_INVERSE_RXD);  // Inverse
  uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

void IRAM_ATTR timer_isr(){
  TIMERG0.int_clr_timers.t0 = 1;  // clear timer
  flag = 1;
}

static void alarm_init() {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr,(void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz          = 1;  // set output frequency at 1 Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.timer_sel  = LEDC_TIMER_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = LEDC_OUTPUT_IO;
    ledc_channel.duty       = 100; // set duty at about 10%
    ledc_channel.hpoint     = 0;
    ledc_channel_config(&ledc_channel);
}

static void pcnt_example_init(void)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}


////////////////////////////////////////////////////////////////////////////////
// Setup / Utility  Functions ///////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Lidar Functions /////////////////////////////////////////////////////////////

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {

 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
 i2c_master_start(cmd);
 i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
 i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
 i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
 i2c_master_stop(cmd);
 i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
 i2c_cmd_link_delete(cmd);
 return 0;
}

// Read register
uint8_t readRegister(uint8_t reg, uint8_t* regData) {
 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
 i2c_master_start(cmd);
 i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
 i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
 i2c_master_start(cmd);
 i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | READ_BIT, ACK_CHECK_EN);
 i2c_master_read_byte(cmd, regData, ACK_CHECK_DIS);

 //printf("--Read in value: *regData = %d\n", *regData);

 i2c_master_stop(cmd);
 i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
 i2c_cmd_link_delete(cmd);

 return 0;
}

// read 16 bits (2 bytes) -- not working for unknown reason
int16_t read16(uint8_t reg, uint8_t *first, uint8_t *second) {

 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
 i2c_master_start(cmd);
 i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
 i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
 i2c_master_start(cmd);
 i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | READ_BIT, ACK_CHECK_EN);
 i2c_master_read_byte(cmd, first, ACK_CHECK_EN);
 i2c_master_read_byte(cmd, second, ACK_CHECK_DIS);
 i2c_master_stop(cmd);
 i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
 i2c_cmd_link_delete(cmd);

 printf("--Read in value: *first = %d\n", *first);
 printf("--Read in value: *second = %d\n", *second);

 return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Lidar Functions /////////////////////////////////////////////////////////////

// Front Lidar Rangefinder
static void test_lidar() {
 printf("\n>> Testing Lidar\n");
 int prev_front_Dist = -1;

 while(1) {
 uint8_t reg = REGISTER;
 uint8_t data = VALUE;
 writeRegister(reg, data);

 vTaskDelay(20 / portTICK_RATE_MS);

 uint8_t high_dist = 0;
 uint8_t low_dist = 0;
 uint8_t *first = &high_dist;
 uint8_t *second = &low_dist;

 uint8_t regH = 0x0f;
 uint8_t regL = 0x10;

 readRegister(regH, first);
 readRegister(regL, second);

 int distance = ((high_dist << 8) + low_dist); // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
 if(prev_front_Dist<0){
   prev_front_Dist = distance;
   front_Dist = distance;
 }
 int diff = abs(distance-prev_front_Dist);

 if(distance>0 && diff<50){
    prev_front_Dist = front_Dist;
    front_Dist = distance;

    if(front_Dist<250 && mode==1){    // starts to turn at 2.5m away from the wall
      turn = 1;
    }

 }

 prev_front_Dist = distance;

 // Print Distance
 //printf("*****FRONT: %d cm*****\n", distance);

 vTaskDelay(500 / portTICK_PERIOD_MS);
 }
}

// UDP Socket Send
static void udp_client_send(void *pvParameters){
    while (1) {
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
        int err;
        float prev_split = -1;
        while(1){
            char buf[20];
            itoa(mode, buf, 10);
            char smode[] = "mode,";
            strcat(smode,buf);
            err = sendto(sock, smode, strlen(smode), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            char buf1[20];
            sprintf(buf1, "%g", mywheelSpeed);
            char wheel[] = "wheelSpeed,";
            strcat(wheel,buf1);
            err = sendto(sock, wheel, strlen(wheel), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            char buf2[20];
            itoa((int)totalTime, buf2, 10);
            char time[] = "totalTime,";
            strcat(time,buf2);
            err = sendto(sock, time, strlen(time), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            char buf3[20];
            itoa(mySteering, buf3, 10);
            char steer[] = "steering,";
            strcat(steer,buf3);
            err = sendto(sock, steer, strlen(steer), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            char split[] = "curSplit,";
            strcat(split,str_split);
            if(curSplit>prev_split){
              err = sendto(sock, split, strlen(split), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
              prev_split = curSplit;
            }

            char buf5[20];
            itoa(measuredSide, buf5, 10);
            char side[] = "side,";
            strcat(side,buf5);
            err = sendto(sock, side, strlen(side), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            char buf6[20];
            itoa(front_Dist, buf6, 10);
            char front[] = "front,";
            strcat(front,buf6);
            err = sendto(sock, front, strlen(front), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

//UDP Socket Receive
static void udp_client_receive(){
  while(1){
    if(sock>0){
        struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            //break;
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            //ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
            ESP_LOGI(TAG, "%s", rx_buffer);
            if(rx_buffer[0]=='S'){
              mode = 0;
            } else if(rx_buffer[0]=='A'){
              mode = 1;
              setWheel = 0.1;
              speed = SPEED_F_1;
            } else if(rx_buffer[0]=='F'){
              mode = 2;
              setWheel = 0.1;
              speed = SPEED_F_1;
            } else if(rx_buffer[0]=='B'){
              mode = 3;
              speed = SPEED_B_1;
              setWheel = 0.1;
            } else if(rx_buffer[0]=='s'){
              if(rx_buffer[1]=='1' && mode==2){
                setWheel = 0.1;
                speed = SPEED_F_1;
              } else if(rx_buffer[1]=='2' && mode==2){
                setWheel = 0.3;
                speed = SPEED_F_2;
              } else if(rx_buffer[1]=='3' && mode==2){
                setWheel = 0.5;
                speed = SPEED_F_3;
              } else if(rx_buffer[1]=='1' && mode==3){
                setWheel = 0.1;
                speed = SPEED_B_1;
              } else if(rx_buffer[1]=='2' && mode==3){
                setWheel = 0.3;
                speed = SPEED_B_2;
              } else if(rx_buffer[1]=='3' && mode==3){
                setWheel = 0.5;
                speed = SPEED_B_3;
              }
            } else if(rx_buffer[0]=='L'){
              mySteering += 15;
            } else if(rx_buffer[0]=='R'){
              mySteering -= 15;
            } else if(rx_buffer[0]=='C'){
              mySteering = 45;
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  vTaskDelete(NULL);
}

// Alpha Display
static void test_alpha_display() {
    int ret;
    char str_sec[5];
    char str_min[5];
    printf(">> Test Alphanumeric Display: \n");
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    uint16_t displaybuffer[8];
    // Continually writes the same command
    while (1) {
      int sec = (int)curSplit%60;
      int min = (int)curSplit/60;
      itoa(sec, str_sec, 10);
      itoa(min, str_min, 10);

      if(min>9){
        ind1 = (int)(str_min[0]-'0');
        ind2 = (int)(str_min[1]-'0');
        str_split[0] = str_min[0];
        str_split[1] = str_min[1];
      } else {
        if(min==0){
          ind1=0;
          ind2=0;
          str_split[0] = '0';
          str_split[1] = '0';
        } else {
          ind1=0;
          ind2 = (int)(str_min[0]-'0');
          str_split[0] = '0';
          str_split[1] = str_min[0];
        }
      }

      if(sec>9){
        ind3 = (int)(str_sec[0]-'0');
        ind4 = (int)(str_sec[1]-'0');
        str_split[3] = str_sec[0];
        str_split[4] = str_sec[1];
      } else {
        if(sec==0){
          ind3=0;
          ind4=0;
          str_split[3] = '0';
          str_split[4] = '0';
        } else {
          ind3 = 0;
          ind4 = (int)(str_sec[0]-'0');
          str_split[3] = '0';
          str_split[4] = str_sec[0];
        }
      }
      //printf("Current Split: %s\n", str_split);
      //what to display
      displaybuffer[0] = alphafonttable[ind1];
      displaybuffer[1] = alphafonttable[ind2];
      displaybuffer[2] = alphafonttable[ind3];
      displaybuffer[3] = alphafonttable[ind4];

      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

      if(ret == ESP_OK) {
        //printf("- wrote %d%d:%d%d \n\n", ind1, ind2, ind3, ind4);
      }
    }
}

// Side microLIDAR RF
static void rx_task(void *arg){
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    int i;
    float dist = 0;
    while (1) {
        int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 300 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            for(i=0; i<1024; i++){
              if((i<=1020) && (data[i]==89) && (data[i+1]==89)){
                dist = ((255.0 * (float)data[i+3]) + (float)data[i+2]);   // in cm
              }
            }
            if (dist<1100.0) {
              measuredSide = (int)dist;
              //printf("***** SIDE: %i\n", measuredSide);
            }

        }
    }
    free(data);
}

// Steering-1: when not turning
void steering(void *arg){
  int STerr;
  // mySteering = 90 (LEFT); mySteering = 0 (RIGHT);
    while (1) {
      if(turn==0){
        if(mode==1){    // In Auto Mode: striaght
          STerr = setPoint_side - measuredSide;
          mySteering = 45+(1.0*STerr);
          if(mySteering<0){
            mySteering = 0;
          } else if(mySteering>90){
            mySteering = 90;
          }
          angle = servo_per_degree_init((uint32_t)mySteering);
          //printf("now mySteering: %i\n", mySteering);
          printf("Mode: %i, splitCount: %i, over: %i\n", mode, splitCount, over);
          mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {                            // Other mode
          if(mySteering<0){
            mySteering = 0;
          } else if(mySteering>90){
            mySteering = 90;
          }
          uint32_t angle = servo_per_degree_init((uint32_t)mySteering);
          //printf("now mySteering: %i, angle = %i\n", mySteering, angle);
          //printf("Mode: %i, splitCount: %i, over: %i\n", mode, splitCount, over);
          mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          vTaskDelay(300 / portTICK_PERIOD_MS);
        }

      }
    }
}

// Steering-2: when turning
void myTurn(){
  while(1){
    if(turn==1 && mode==1){
      printf("***** Turning Start *****\n");
      printf("Front: %i\n", front_Dist);
      angle = servo_per_degree_init(90);
      mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
      vTaskDelay(7500 / portTICK_PERIOD_MS);    // turn 90 degree to the left (for 7.5 sec)
      angle = servo_per_degree_init(45);
      mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
      mySteering = 45;                          // Go back to striaght
      turn = 0;
      printf("***** Turning End *****\n");
      vTaskDelay(9000 / portTICK_PERIOD_MS);  // Does not turn again in the next 9 sec
    }
  }
}

// Speed Control-1: PID control of wheelSpeed
void PID(){
  int check = 0;
  if(mode!=0){    // auto
    //printf("\nCurrent speed: %f meters per second. \n", mywheelSpeed);
    if(mode==1){
      error = 0.1 - mywheelSpeed;
    } else {
      //printf("setWheel: %f\n", setWheel);
      error = setWheel - mywheelSpeed;
    }
    integral = integral + error * dt;
    derivative = (error - previous_error) / dt;
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    //printf("P: %f, I: %f, D: %f, **OUTPUT: %f\n", error, integral, derivative, output);

    if((mode==2) | (mode==1)){
      check = (int)speed - (int)output;   // Going forward
    } else {
      check = (int)speed + (int)output;   // GOing backward
    }

    if(((mode==1) | (mode==2)) && check>=1290){
      speed = 1285;
    } else if(mode==3 && check<1550) {
      speed = 1550;
    } else {
      speed = (uint32_t)check;
    }
    //printf("****New SPEED: %i \n", check);
    //printf("****SPEED: %i\n", speed);
  }

}

// Speed Control-2: Update/assign speed of crawler
void runESC() {
  while (1) {
    /*
    Now full forward = 800, full backward = 2100, stop = 1400
    min forward: 1285
    min backward: 1550
    */
    if(mode==0){  // stop
      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
    } else if(turn==1) { // others
      //printf("****SPEED: %i\n", speed);
      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1265);
      speed = 1285;
    } else { // others
      //printf("****SPEED: %i\n", speed);
      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}

// Speed Control-3: Wheel-speed sening
void wheelSpeed(){
  int16_t count = 0;
  int16_t count1 = 0;
  int16_t countdiff = 0;
  float countdist = 0;
  float countspeed = 0;
  pcnt_evt_t evt;
  portBASE_TYPE res;
  while (1) {

      res = xQueueReceive(pcnt_evt_queue, &evt, 500 / portTICK_PERIOD_MS);
      {
          pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
          countdiff = count - count1;
          countdist = 0.05*(float)countdiff;    // 5cm per pulse
          countspeed = countdist/0.5; //meters per second
          mywheelSpeed = countspeed;
          count1 = count;

      }
  }
  if(user_isr_handle) {
      //Free the ISR service handle.
      esp_intr_free(user_isr_handle);
      user_isr_handle = NULL;
  }
}

// IR Receiver: Traffic Light
static void IR_task(void *arg){
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    int i;
    char c;
    char prev_c = '@';
    int prev_Time, tDiff;
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 100 / portTICK_RATE_MS);
        if (rxBytes > 0 && over==0 && ((mode==0) | (mode==1))) {   // in auto or stop mode
          data[rxBytes] = 0;
            for(i=0; i<rxBytes; i++){
              c = (char)data[i];
              if(c=='R'){
                if(prev_c=='@'){    // first Red
                  prev_c =c;
                  mode = 0;
                  printf("Waiting to start (R)\n");
                } else if(prev_c!=c) {    // other red
                  prev_c=c;
                  mode = 0;
                  printf("GOT REDDDDDD\n");
                  splitCount++;
                  curSplit=totalTime;
                  printf("New split: %f\n", curSplit);
                }
                if(splitCount==0){
                  curSplit=0;
                } else if (splitCount>=3){    // stop and enter manual mode
                  mode = 0;
                  over = 1;
                  mySteering = 45;
                }
                break;
              } else if(c=='G'){
                if(prev_c!=c){   // other green
                  prev_c=c;
                  if(splitCount==0){
                    splitCount++;
                  }
                  printf("GOT GREENNNNN\n");
                  if(splitCount<3){ // auto forward
                    speed = 1270;
                    mode = 1;
                  }
                }
                break;
              }
            }
        }
    }
    free(data);
}


void app_main(void)
{
    printf("Testing servo motor.......\n");
    alarm_init();
    uart_init();
    // socket io setup
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    //Set up for front rf
    i2c_master_init();
    i2c_scanner();

    //setup for wheel
    ledc_init();
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init();


    // Setup MCPWM
    mcpwm_example_gpio_initialize();
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    // for ESC (A)
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);    // for Steering (B)


    //Now forward = 800, backward = 2100, stop = 1400
    printf("PLEASE TURN ON CRAWLER\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    printf("Set High\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds (full forward)
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Set Low\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds (full backward)
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Set NEUTRAL\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);

    // Initialization
    speed = SPEED_F_1;
    mode = 0;   // start at stop mode
    mySteering = 45; // center from 0-90
    splitCount = 0;
    turn = 0;

    // All Tasks
    xTaskCreate(rx_task, "side sensor", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(steering, "steering", 4096, NULL, 5, NULL);
    xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(runESC, "runESC", 4096, NULL, 5, NULL);
    xTaskCreate(test_lidar,"front", 4096, NULL, 5, NULL);
    xTaskCreate(wheelSpeed,"wheelSpeed", 4096, NULL, 5, NULL);
    xTaskCreate(udp_client_receive, "udp_client_receive", 4096, NULL, 5, NULL);
    xTaskCreate(udp_client_send, "udp_client_send", 4096, NULL, 5, NULL);
    xTaskCreate(IR_task, "IR_task", 4096, NULL, 5, NULL);
    xTaskCreate(myTurn, "Turnning", 4096, NULL, 5, NULL);

    // Timer
    while(1){
      if(flag==1 && splitCount>=0){
        if(splitCount>0){
          totalTime+=0.1;
        }
        if (dt>=1.0) {
          PID();
          dt = 0;
        } else {
          dt += 0.1;
        }
        flag = 0;
        // After the alarm triggers, we need to re-enable it to trigger it next time
        TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS); // delay to keeps from idling
    }

}
