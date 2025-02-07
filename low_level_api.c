/***************************************************************
 * Low-Level API for Arduino Portenta H7 (STM32H747XI)        *
 * ----------------------------------------------------------- *
 * This code implements:                                        *
 *  • Real-time sensor data updates using timer interrupts      *
 *  • Non-blocking UART communication with HAL_UART_Transmit     *
 *  • A robust command packet protocol with checksum validation  *
 *  • A merged obstacle detection packet (LiDAR and extras)      *
 *  • A PID loop for line-following using five IR sensors with     *
 *    moving-average filtering and weighted error calculation     *
 *  • PWM–based motor control (SET_SPEED and ADJUST_HEADING)      *
 *  • Emergency stop logic and communication timeout detection    *
 ***************************************************************/

#include "stm32h7xx_hal.h"
// #include "main.h"   
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h> // for rand()

/* ------------------------ Definitions ------------------------ */
#define MAX_PAYLOAD_SIZE      8

/* Error Codes */
#define ERR_INVALID_COMMAND   0x01
#define ERR_OUT_OF_RANGE      0x02
#define ERR_SENSOR_FAILURE    0x03
#define ERR_UART_FAILURE      0x04

/* Communication timeout (ms) */
#define COMM_TIMEOUT_MS       1000

/* PID limits */
#define PID_MAX               255
#define PID_MIN              -255

/* IR sensor definitions */
#define NUM_IR_SENSORS        5
#define MOVING_AVG_SIZE       10

/* LiDAR definitions */
#define LIDAR_BUFFER_SIZE     5

/* Emergency stop reasons */
#define STOP_REASON_OBSTACLE      0x01
#define STOP_REASON_COMM_LOSS     0x02

/* --------------------- Command and Response Codes --------------------- */
typedef enum {
    /* Commands from CS Team to Hardware */
    CMD_SET_SPEED         = 0x01,  // Payload: [speed_value] (1 byte, 0-255)
    CMD_ADJUST_HEADING    = 0x02,  // Payload: [angle] (1 byte, signed, -30 to +30)
    CMD_STOP              = 0x03,  // Payload: None
    CMD_GET_IR_DATA       = 0x04,  // Payload: None
    CMD_GET_OBSTACLE_DATA = 0x05,  // Payload: None

    /* Responses from Hardware to CS Team */
    RSP_ACK               = 0x10,  // No payload
    RSP_NACK              = 0x11,  // Payload: [error_code]
    RSP_IR_DATA           = 0x12,  // Payload: [ir0_H,ir0_L, ir1_H,ir1_L, ...] (5 x 2 bytes)
    RSP_OBSTACLE_DATA     = 0x13,  // Payload: [distance_H, distance_L, obstacle_detected, pos_H, pos_L, size_H, size_L]
    RSP_EMERGENCY_STOP    = 0x15   // Payload: [stop_reason]
} CommandType;

/* ------------------------ Packet Structure ------------------------ */
typedef struct {
    uint8_t command;                   // Command or response code
    uint8_t length;                    // Payload length in bytes
    uint8_t payload[MAX_PAYLOAD_SIZE]; // Payload data
    uint8_t checksum;                  // Simple XOR checksum
} Packet;

/* ------------------------ Global Variables ------------------------ */

/* UART and Timer handles (must be defined/initialized elsewhere) */
extern UART_HandleTypeDef huart1;     // USART for serial communication
extern TIM_HandleTypeDef htim_pid;    // High-resolution timer for PID loop (100Hz)
extern TIM_HandleTypeDef htim_ir;     // Timer for IR sensor updates (1kHz)
extern TIM_HandleTypeDef htim_motor;  // Timer configured for PWM motor control

/* Communication watchdog */
volatile uint32_t lastCommTick = 0;

/* PID control variables */
float kp = 1.0f, ki = 0.1f, kd = 0.05f;
float pid_integral = 0.0f;
float pid_previous_error = 0.0f;
volatile int16_t pid_output = 0;

/* IR sensor raw and filtered data (5 sensors) */
volatile uint16_t ir_raw[NUM_IR_SENSORS] = {0};
volatile uint16_t ir_filtered[NUM_IR_SENSORS] = {0};
/* Moving-average filter history */
uint16_t ir_history[NUM_IR_SENSORS][MOVING_AVG_SIZE] = {0};
uint8_t ir_history_index = 0;

/* LiDAR sensor data and median filter buffer */
volatile uint16_t lidar_buffer[LIDAR_BUFFER_SIZE] = {0};
uint8_t lidar_index = 0;
volatile uint16_t lidar_distance = 0; // Median filtered value

/* ------------------------ Function Prototypes ------------------------ */
uint8_t calculate_checksum(Packet *pkt);
void send_packet(Packet *pkt);
void send_ack(void);
void send_nack(uint8_t error_code);
void process_packet(Packet *pkt);
void emergency_stop(uint8_t reason);

void set_motor_speed(uint8_t speed);
void adjust_motor_heading(int8_t angle);
void stop_motors(void);

void update_pid(void);
void update_ir_filters(void);
void update_lidar_median(void);

void read_ir_sensors(void);      // Asynchronously read IR sensors (via ADC/DMA in real use)
void read_lidar_sensor(void);    // Read LiDAR sensor (via SoftwareSerial or hardware UART)
void check_communication_timeout(void);
void watchdog_reload(void);

/* ------------------------ Implementation ------------------------ */

/* Calculate a simple XOR checksum over command, length, and payload */
uint8_t calculate_checksum(Packet *pkt) {
    uint8_t cs = pkt->command ^ pkt->length;
    for (int i = 0; i < pkt->length; i++) {
        cs ^= pkt->payload[i];
    }
    return cs;
}

/* Send a packet over UART using HAL_UART_Transmit */
void send_packet(Packet *pkt) {
    pkt->checksum = calculate_checksum(pkt);
    /* Transmit entire packet: command (1) + length (1) + payload + checksum (1) */
    uint8_t packetSize = 3 + pkt->length;
    if (HAL_UART_Transmit(&huart1, (uint8_t*)pkt, packetSize, 10) != HAL_OK) {
        send_nack(ERR_UART_FAILURE);
    }
}

/* Transmit an ACK response */
void send_ack(void) {
    Packet pkt;
    pkt.command = RSP_ACK;
    pkt.length = 0;
    send_packet(&pkt);
}

/* Transmit a NACK response with an error code */
void send_nack(uint8_t error_code) {
    Packet pkt;
    pkt.command = RSP_NACK;
    pkt.length = 1;
    pkt.payload[0] = error_code;
    send_packet(&pkt);
}

/* Process an incoming command packet */
void process_packet(Packet *pkt) {
    /* Verify checksum */
    if (calculate_checksum(pkt) != pkt->checksum) {
        send_nack(ERR_INVALID_COMMAND);
        return;
    }
    /* Reset the communication watchdog */
    lastCommTick = HAL_GetTick();
    
    switch (pkt->command) {
        case CMD_SET_SPEED:
            if (pkt->length != 1) {
                send_nack(ERR_INVALID_COMMAND);
                break;
            }
            set_motor_speed(pkt->payload[0]);
            send_ack();
            break;
            
        case CMD_ADJUST_HEADING:
            if (pkt->length != 1) {
                send_nack(ERR_INVALID_COMMAND);
                break;
            }
            if ((int8_t)pkt->payload[0] < -30 || (int8_t)pkt->payload[0] > 30) {
                send_nack(ERR_OUT_OF_RANGE);
            } else {
                adjust_motor_heading((int8_t)pkt->payload[0]);
                send_ack();
            }
            break;
            
        case CMD_STOP:
            if (pkt->length != 0) {
                send_nack(ERR_INVALID_COMMAND);
                break;
            }
            stop_motors();
            send_ack();
            break;
            
        case CMD_GET_IR_DATA:
            if (pkt->length != 0) {
                send_nack(ERR_INVALID_COMMAND);
                break;
            }
            {
                Packet rsp;
                rsp.command = RSP_IR_DATA;
                rsp.length = NUM_IR_SENSORS * 2; // 2 bytes per sensor reading
                for (int i = 0; i < NUM_IR_SENSORS; i++) {
                    rsp.payload[i * 2]     = (uint8_t)(ir_filtered[i] >> 8);
                    rsp.payload[i * 2 + 1] = (uint8_t)(ir_filtered[i] & 0xFF);
                }
                send_packet(&rsp);
            }
            break;
            
        case CMD_GET_OBSTACLE_DATA:
            if (pkt->length != 0) {
                send_nack(ERR_INVALID_COMMAND);
                break;
            }
            {
                /* Merge LiDAR data and obstacle info into one binary packet */
                Packet rsp;
                rsp.command = RSP_OBSTACLE_DATA;
                rsp.length = 7; // 2 bytes: distance, 1 byte: detected flag, 2 bytes: position, 2 bytes: size
                update_lidar_median(); // update median-filtered LiDAR value
                rsp.payload[0] = (uint8_t)(lidar_distance >> 8);
                rsp.payload[1] = (uint8_t)(lidar_distance & 0xFF);
                /* Determine if an obstacle is detected (threshold: 8000 mm) */
                uint8_t obstacle_detected = (lidar_distance < 8000) ? 1 : 0;
                rsp.payload[2] = obstacle_detected;
                /* For simulation purposes, use dummy values for position and size */
                uint16_t obstacle_position = 128; // centered
                uint16_t obstacle_size = 50;        // arbitrary size estimate
                rsp.payload[3] = (uint8_t)(obstacle_position >> 8);
                rsp.payload[4] = (uint8_t)(obstacle_position & 0xFF);
                rsp.payload[5] = (uint8_t)(obstacle_size >> 8);
                rsp.payload[6] = (uint8_t)(obstacle_size & 0xFF);
                send_packet(&rsp);
            }
            break;
            
        default:
            send_nack(ERR_INVALID_COMMAND);
            break;
    }
}

/* --------------------- Motor Control via PWM --------------------- */
/* These functions use HAL_TIM_PWM to adjust motor speed and heading. */

void set_motor_speed(uint8_t speed) {
    /* Assume motor speed is driven on TIM_motor, Channel 1 */
    __HAL_TIM_SET_COMPARE(&htim_motor, TIM_CHANNEL_1, speed);
}

void adjust_motor_heading(int8_t angle) {
    /* Map heading angle (-30 to 30) to PWM pulse width for steering servo.
       For example: 0 deg -> 1500 µs, ±30 deg -> ±300 µs offset. */
    uint16_t pwm_value = 1500 + (angle * 10); // Example mapping (adjust scaling as needed)
    __HAL_TIM_SET_COMPARE(&htim_motor, TIM_CHANNEL_2, pwm_value);
}

void stop_motors(void) {
    /* Stop motor PWM output and center steering */
    __HAL_TIM_SET_COMPARE(&htim_motor, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim_motor, TIM_CHANNEL_2, 1500);
}

/* --------------------- PID Control and IR Filtering --------------------- */

/* The PID loop reads the filtered IR sensor data to determine the error
   (using a weighted average over five sensors) and then adjusts motor control. */
void update_pid(void) {
    int32_t weighted_sum = 0;
    uint32_t sum = 0;
    int weights[NUM_IR_SENSORS] = { -2, -1, 0, 1, 2 };
    
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        uint16_t value = ir_filtered[i];
        weighted_sum += value * weights[i];
        sum += value;
    }
    int error = (sum > 0) ? (weighted_sum / sum) : 0;
    
    /* PID computations */
    pid_integral += error;
    float derivative = error - pid_previous_error;
    float output = kp * error + ki * pid_integral + kd * derivative;
    pid_previous_error = error;
    
    /* Clamp PID output */
    if (output > PID_MAX) output = PID_MAX;
    if (output < PID_MIN) output = PID_MIN;
    pid_output = (int16_t)output;
    
    /* Adjust motor control based on PID output:
       - When error is high, adjust steering via heading.
       - When error is low, drive faster.
    */
    if (abs(error) > 1) {
        adjust_motor_heading((int8_t)pid_output);
        set_motor_speed(128); // Moderate speed
    } else {
        adjust_motor_heading(0);
        set_motor_speed(200); // Increase speed
    }
}

/* Update IR sensor moving-average filters.
   In a real system, the IR sensors are read via ADC/DMA in the background. */
void update_ir_filters(void) {
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        ir_history[i][ir_history_index] = ir_raw[i];
        uint32_t sum = 0;
        for (int j = 0; j < MOVING_AVG_SIZE; j++) {
            sum += ir_history[i][j];
        }
        ir_filtered[i] = sum / MOVING_AVG_SIZE;
    }
    ir_history_index = (ir_history_index + 1) % MOVING_AVG_SIZE;
}

/* Update LiDAR median filter and trigger emergency stop if too close */
void update_lidar_median(void) {
    /* Get a new LiDAR reading and store it in the buffer */
    read_lidar_sensor(); // Updates lidar_buffer[lidar_index]
    lidar_index = (lidar_index + 1) % LIDAR_BUFFER_SIZE;
    
    uint16_t temp[LIDAR_BUFFER_SIZE];
    for (int i = 0; i < LIDAR_BUFFER_SIZE; i++) {
        temp[i] = lidar_buffer[i];
    }
    /* Simple bubble sort for median computation */
    for (int i = 0; i < LIDAR_BUFFER_SIZE - 1; i++) {
        for (int j = 0; j < LIDAR_BUFFER_SIZE - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                uint16_t swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    lidar_distance = temp[LIDAR_BUFFER_SIZE / 2];
    
    /* If an obstacle is too close (below 8000 mm), trigger an emergency stop */
    if (lidar_distance < 8000) {
        emergency_stop(STOP_REASON_OBSTACLE);
    }
}

/* --------------------- Simulated Sensor Reading Functions --------------------- */

/* Simulate asynchronous IR sensor readings.
   In practice, these would be updated via ADC conversions with DMA/interrupts. */
void read_ir_sensors(void) {
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        /* Simulate ADC readings with a baseline value plus random noise */
        ir_raw[i] = 1000 + (rand() % 100);
    }
}

/* Simulate LiDAR sensor reading.
   Replace with actual interface (SoftwareSerial read from TFmini Plus). */
void read_lidar_sensor(void) {
    /* Simulate a LiDAR distance reading between 5000 and 10000 mm */
    uint16_t simulated_distance = 5000 + (rand() % 5000);
    lidar_buffer[lidar_index] = simulated_distance;
}

/* --------------------- Communication Timeout and Watchdog --------------------- */

/* If no valid packet is received within COMM_TIMEOUT_MS, trigger emergency stop */
void check_communication_timeout(void) {
    if ((HAL_GetTick() - lastCommTick) > COMM_TIMEOUT_MS) {
        emergency_stop(STOP_REASON_COMM_LOSS);
    }
}

/* Reload the watchdog timer.
   Replace with platform’s watchdog refresh call (HAL_IWDG_Refresh(&hiwdg)). */
void watchdog_reload(void) {
    // HAL_IWDG_Refresh(&hiwdg);
}

/* --------------------- Interrupt Callbacks --------------------- */

/* Timer interrupt callback.
   This function is called by HAL from the appropriate IRQ handlers.
   Ensure that htim_ir is configured to trigger at 1kHz and htim_pid at 100Hz. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim_ir.Instance) {
        /* IR sensor update interrupt (1kHz) */
        read_ir_sensors();
        update_ir_filters();
    }
    else if (htim->Instance == htim_pid.Instance) {
        /* PID loop update interrupt (100Hz) */
        update_pid();
    }
}

/* UART receive complete callback.
   Called by HAL when a complete packet is received.
   (In a real implementation, you must manage buffering and packet framing.) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    Packet received_pkt;
    /* Copy the received data into received_pkt.
       The actual implementation depends on your UART receive buffer management. */
    process_packet(&received_pkt);
    
    /* Restart UART receive interrupt */
    HAL_UART_Receive_IT(&huart1, /* buffer pointer */, /* expected length */);
}

/* --------------------- Emergency Stop --------------------- */
/* Immediately halt motors and notify the CS team of the stop reason. */
void emergency_stop(uint8_t reason) {
    stop_motors();
    Packet pkt;
    pkt.command = RSP_EMERGENCY_STOP;
    pkt.length = 1;
    pkt.payload[0] = reason;
    send_packet(&pkt);
}

/* --------------------- Main Loop --------------------- */
int main(void) {
    /* HAL and system clock initialization */
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize all peripherals (GPIO, UART, ADC, Timers, PWM, Watchdog, etc.) */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM_PID_Init();
    MX_TIM_IR_Init();
    MX_TIM_Motor_Init();
    // Initialize ADC and any DMA channels as required.
    
    /* Start timer interrupts */
    HAL_TIM_Base_Start_IT(&htim_ir);
    HAL_TIM_Base_Start_IT(&htim_pid);
    
    /* Start PWM channels for motor control */
    HAL_TIM_PWM_Start(&htim_motor, TIM_CHANNEL_1); // Motor speed
    HAL_TIM_PWM_Start(&htim_motor, TIM_CHANNEL_2); // Steering
    
    /* Start UART receive interrupt */
    HAL_UART_Receive_IT(&huart1, /* buffer pointer */, /* expected length */);
    
    lastCommTick = HAL_GetTick();
    
    while (1) {
        /* Background loop:
           - Monitor communication timeout and reload watchdog
           - Additional background tasks placed here */
        check_communication_timeout();
        watchdog_reload();
        
    }
}
