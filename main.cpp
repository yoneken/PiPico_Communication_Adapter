#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "Modbus.h"
#include "hardware/pio.h"
#include "leptrino.h"
#include "hardware/uart.h"

extern "C" {
  #include "can2040.h"
  #include "hardware/irq.h"
  #include "rs422_tx.h"
}

#define MODBUS_TX_PIN 0 
#define MODBUS_RX_PIN 1
#define MODBUS_EN_PIN 2
#define CANBUS_RX_PIN 4
#define CANBUS_TX_PIN 3
#define RS422_RXP_PIN 5
#define RS422_TXP_PIN 6
#define RS422_TXM_PIN 7
#define LEPTRINO_TX_PIN 8
#define LEPTRINO_RX_PIN 9

// グローバル変数
LeptrinoSensor* g_leptrino_sensor = nullptr;

modbusHandler_t ModbusH;
uint16_t ModbusDATA[0x8ff];
// Modbus register address definitions
/*
Register address Description
Monitoring Group
0x000 status
0x001 speed (r / min)
0x002 current percentage
0x003 current (A)
0x004 command position (p)
0x006 motor position (p)
0x008 position error (p)
0x00F current alarm code
0x010 current value when alarm occurs
0x011 speed value when alarm occurs
0x012 input voltage value when alarm occurs
0x020 calibration value of the 6-axis force sensor (12 bytes)
0x030 X Force
0x034 Y Force
0x038 Z Force
0x040 X Torque
0x044 Y Torque
0x048 Z Torque

Fn1xx Control Parameters
Number, Name, Setting range, Unit, Factory setting, Effective time, Register address
Fn100, activated the gripper, 0-1, -, 0, immediately, 0x0100
Fn101, control mode, 0-2 0：location 1：speed, -, 0, immediately 0x0101
Fn109, fault reset, 0-1, -, 0, immediately, 0x0109
Fn110, calibrate gripper, 0-1, -, 0, immediately, 0x0110
Fn120, calibrate force sensor, 0-1, -, 0, immediately, 0x0120

Fn2xx Gain Parameters
Fn200, position loop gain, 10-20000, 0.1Hz, 200, immediately, 0x0200
Fn201, position loop feedforward, 0-1000, 0.1%, 200, immediately, 0x0201
Fn202, position loop feedforward filtering time, 0-1000, 1ms, 5, immediately, 0x0202
Fn203, speed loop gain, 10-20000, 0.1, 100, immediately, 0x0203
Fn204, speed loop integral, 10-10000, 0.1, 300, immediately, 0x0204

Fn3xx Position Parameters
Fn300, Position acceleration time, 1-2000, ms, 100, power on effective, 0x0300
Fn301, position deceleration time, 1-2000, ms, 100, power on effective, 0x0301
Fn302, position smoothing time, 1-200, ms, 10, power on effective, 0x0302
Fn303, position speed, 1-20000, r/min, 1500, immediately, 0x0303
Fn308, position error alarm value, 0x00000000-0xFFFFFFFF, Pulse, 0x20000, immediately, 0x0308
Fn310, position command alarm value, 0x00000000-0xFFFFFFFF, Pulse, 0x20000, immediately, 0x030A

Fn4xx Speed Parameters
Fn400, speed command, -20000-20000, r/min, 100, immediately, 0x0400
Fn403, speed limit, 0-20000, r/min, 5000, immediately, 0x0403

Fn5xx Torque Parameters
Fn505, sarting current limit, 5-100, -, 16, immediately, 0x0505
Fn506, hold current limit, 1-100, -, 10, immediately, 0x0506
Fn507, starting current operation time, 100-30000, ms, 1500, immediately, 0x0507

Fn6xx Communication Parameters

Fn7xx Position Command
Fn700, position command, 0x00000000-0xFFFFFFFF, pulse, 0x00000000, immediately, 0x0700
Fn702, position feedback, 0x00000000-0xFFFFFFFF, pulse, 0x00000000, read-only, 0x0702
Fn706, electronic gear ratio numerator, 1-30000, -, 100, power on effective, 0x0706
Fn707, electronic gear ratio denominator, 1-30000, -, 100, power on effective, 0x0707

Fn8xx Motor Parameters
Fn800, hardware version, -, -, 10, read-only, 0x0800
Fn801, software version, -, -, read-only, 0x0801
Fn804, motor ID, 0-999, -, 100, effective after power on, 0x0804
Fn805, rated power, 1-2000, W, 100, effective after power on, 0x0805
Fn806, rated voltage, 1-6000, 0.01V, 2400, effective after power on, 0x0806
Fn807, rated current, 1-2400, 0.01A, 800, effective after power on, 0x0807
*/

enum {
    REG_TARGET_POSITION=0x004,
    REG_MOTOR_POSITION=0x006,
    REG_X_FORCE=0x030,
    REG_Y_FORCE=0x034,
    REG_Z_FORCE=0x038,
    REG_X_TORQUE=0x040,
    REG_Y_TORQUE=0x044,
    REG_Z_TORQUE=0x048,
    REG_ACTIVATE=0x100,
    REG_MODE=0x101,
    REG_FAULT_RESET=0x109,
    REG_CALIBRATE_GRIPPER=0x110,
    REG_CALIBRATE_FORCE_SENSOR=0x120,
    REG_SET_SPEED=0x303,
    REG_SET_POSITION=0x700
};

enum { DEACTIVATE=0, ACTIVATE };
enum { IDLE_AUTO_RELEASE_CALIB=0, GO_TO };
enum { IN_MOTION=0, DETECTED_CLOSE, DETECTED_OPEN, AT_POSITION };
enum { NO_ERROR=0, TEMP_ERROR, TIMEOUT_ERROR, BOTH_ERROR };
enum { NOT_CALIBRATED_YET=0, CALIBRATED, CALIBRATING };
enum { MODE_POSITION=0, MODE_SPEED };

volatile uint8_t activated = DEACTIVATE;
volatile uint8_t action_state = IDLE_AUTO_RELEASE_CALIB;
volatile uint8_t object_detection = AT_POSITION;
volatile uint8_t error_state = NO_ERROR;
volatile uint8_t calibrated = NOT_CALIBRATED_YET;
volatile uint16_t target_speed = 1500;
volatile uint32_t target_position = 0;
volatile uint8_t current_position = 0;

SemaphoreHandle_t CanTxSphrHandle = NULL;
static struct can2040 cbus;

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch((msg->id >> 1) & 0x3F){
        case 60: //Respond_Gripper_data_pack
            current_position = msg->data[0];
            activated = (msg->data[3] >> 7) & 0x01;
            action_state = (msg->data[3] >> 6) & 0x01;
            object_detection = (msg->data[3] >> 4) & 0x03;
            error_state = (msg->data[3] >> 2) & 0x03;
            calibrated = (msg->data[3] >> 0) & 0x01;
            break;
    }

    xSemaphoreGiveFromISR(CanTxSphrHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void vTaskSubtask( void * pvParameters )
{
    for(;;)
    {
        printf("Core %u: %d | %d : %d : %d : %d\n", get_core_num(), current_position, target_position, activated, calibrated, error_state);
        vTaskDelay(pdMS_TO_TICKS(1000));

        /*
        ModbusDATA[REG_ACTIVATE] = ACTIVATE;
        printf("Activate: Core %u: %d | %d : %d : %d : %d\n", get_core_num(), current_position, target_position, activated, calibrated, error_state);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ModbusDATA[REG_CALIBRATE_GRIPPER] = 1;
        printf("Calibrate: Core %u: %d | %d : %d : %d : %d\n", get_core_num(), current_position, target_position, activated, calibrated, error_state);
        vTaskDelay(pdMS_TO_TICKS(6000));
        ModbusDATA[REG_SET_POSITION] = 0x20;
        printf("Set Position: Core %u: %d | %d : %d : %d : %d\n", get_core_num(), current_position, target_position, activated, calibrated, error_state);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ModbusDATA[REG_SET_POSITION] = 0xf0;
        printf("Set Position: Core %u: %d | %d : %d : %d : %d\n", get_core_num(), current_position, target_position, activated, calibrated, error_state);
        vTaskDelay(pdMS_TO_TICKS(1000));
        */
    }
}

void vTaskMessageTranslator( void * pvParameters )
{
    can2040_msg send_respond_encoder_data_msg;
    int result = 0;
    for(;;)
    {
        // Leptrinoセンサのデータ処理
        if (g_leptrino_sensor && g_leptrino_sensor->has_complete_packet()) {
            // パケットが完了したので、最新データを取得してModbusレジスタに格納
            ForceData force_data;
            if (g_leptrino_sensor->get_latest_force_data(force_data)) {
                // float値をint16_t形式で格納（スケール調整）
                ModbusDATA[REG_X_FORCE] = (int16_t)(force_data.fx * 100);  // X Force (0.01N単位)
                ModbusDATA[REG_X_FORCE + 1] = (int16_t)(force_data.fx * 100) >> 16;
                ModbusDATA[REG_Y_FORCE] = (int16_t)(force_data.fy * 100);  // Y Force
                ModbusDATA[REG_Y_FORCE + 1] = (int16_t)(force_data.fy * 100) >> 16;
                ModbusDATA[REG_Z_FORCE] = (int16_t)(force_data.fz * 100);  // Z Force
                ModbusDATA[REG_Z_FORCE + 1] = (int16_t)(force_data.fz * 100) >> 16;
                ModbusDATA[REG_X_TORQUE] = (int16_t)(force_data.mx * 1000); // X Torque (0.001Nm単位)
                ModbusDATA[REG_X_TORQUE + 1] = (int16_t)(force_data.mx * 1000) >> 16;
                ModbusDATA[REG_Y_TORQUE] = (int16_t)(force_data.my * 1000); // Y Torque
                ModbusDATA[REG_Y_TORQUE + 1] = (int16_t)(force_data.my * 1000) >> 16;
                ModbusDATA[REG_Z_TORQUE] = (int16_t)(force_data.mz * 1000); // Z Torque
                ModbusDATA[REG_Z_TORQUE + 1] = (int16_t)(force_data.mz * 1000) >> 16;
            }
        }
        
        if(xSemaphoreTake(CanTxSphrHandle , 20) == pdTRUE){
            ModbusDATA[REG_MOTOR_POSITION] = current_position;
            if(ModbusDATA[REG_ACTIVATE] == ACTIVATE){
                uint8_t position = 255 * ((ModbusDATA[REG_SET_POSITION] << 8) | ModbusDATA[REG_SET_POSITION+1]) / 0xFFFF;
                uint8_t speed = 255 * ((ModbusDATA[REG_SET_SPEED] << 8) | ModbusDATA[REG_SET_SPEED+1]) / 20000;

                if(error_state != NO_ERROR){
                    send_respond_encoder_data_msg.id = (0 << 7) | (1 << 1); // 0 is the slave address, 1 is the function code
                    send_respond_encoder_data_msg.dlc = 0; // Data Length Code
                }else if(ModbusDATA[REG_CALIBRATE_GRIPPER] == 0x01){
                    send_respond_encoder_data_msg.id = (0 << 7) | (62 << 1); // 0 is the slave address, 62 is the function code
                    send_respond_encoder_data_msg.dlc = 0; // Data Length Code
                    calibrated = CALIBRATING;
                }else if(calibrated == CALIBRATED){
                    if ((target_position != position) || (target_speed != speed)) {
                        if(ModbusDATA[REG_MODE] == MODE_POSITION){
                            send_respond_encoder_data_msg.id = (0 << 7) | (61 << 1); // 0 is the slave address, 61 is the function code
                            send_respond_encoder_data_msg.dlc = 5; // Data Length Code
                            send_respond_encoder_data_msg.data[0] = position;
                            send_respond_encoder_data_msg.data[1] = speed;
                            send_respond_encoder_data_msg.data[2] = 0x01;
                            send_respond_encoder_data_msg.data[3] = 0xF4;
                            send_respond_encoder_data_msg.data[4] = 0xC0;
                        }else if(ModbusDATA[REG_MODE] == MODE_SPEED){
                            send_respond_encoder_data_msg.id = (0 << 7) | (2 << 1); // 0 is the slave address, 2 is the function code
                            send_respond_encoder_data_msg.dlc = 6; // Data Length Code
                            send_respond_encoder_data_msg.data[0] = speed;
                            send_respond_encoder_data_msg.data[1] = 0;
                            send_respond_encoder_data_msg.data[2] = 0;
                            send_respond_encoder_data_msg.data[3] = 0x00;
                            send_respond_encoder_data_msg.data[4] = 0x00;
                            send_respond_encoder_data_msg.data[5] = 0x00;
                        }
                        ModbusDATA[REG_TARGET_POSITION] = target_position = position;
                        target_speed = speed;
                    }else{
                        // If the position and speed are the same, just send an empty request to update position data
                        send_respond_encoder_data_msg.id = (0 << 7) | (61 << 1) | CAN2040_ID_RTR;
                        send_respond_encoder_data_msg.dlc = 0;
                    }
                }else{
                    // Not calibrated yet
                }
            }else{
                if((activated == ACTIVATE) & (ModbusDATA[REG_ACTIVATE] == DEACTIVATE)){
                    send_respond_encoder_data_msg.id = (0 << 7) | (61 << 1); // 0 is the slave address, 61 is the function code
                    send_respond_encoder_data_msg.dlc = 5; // Data Length Code
                    send_respond_encoder_data_msg.data[0] = 0;
                    send_respond_encoder_data_msg.data[1] = 0;
                    send_respond_encoder_data_msg.data[2] = 0;
                    send_respond_encoder_data_msg.data[3] = 0;
                    send_respond_encoder_data_msg.data[4] = 0;
                }else{
                    // If the gripper is not activated, just send a request for encoder data
                    send_respond_encoder_data_msg.id = (0 << 7) | (61 << 1) | CAN2040_ID_RTR;
                    send_respond_encoder_data_msg.dlc = 0;
                }
            }

            if((calibrated == NOT_CALIBRATED_YET) && (ModbusDATA[REG_ACTIVATE] == DEACTIVATE)){
                // No CAN message
                xSemaphoreGive(CanTxSphrHandle);
            }else{
                result = can2040_transmit(&cbus, &send_respond_encoder_data_msg);
            }

            if(calibrated == CALIBRATING){
                vTaskDelay(pdMS_TO_TICKS(5000));
                calibrated = CALIBRATED;
                ModbusDATA[REG_CALIBRATE_GRIPPER] = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}



void initSerial()
{
    // Modbus Serial Port
    //uart_init(uart0, 2000000);
    //uart_init(uart0, 921600);
    uart_init(uart0, 460800);
    //uart_init(uart0, 230400);
    gpio_set_function(MODBUS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MODBUS_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(uart0, false);
}


void initCanbus(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 1000000;
    uint32_t gpio_rx = CANBUS_RX_PIN, gpio_tx = CANBUS_TX_PIN;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, 1);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

// UART2割り込みハンドラ
void on_uart2_rx() {
    while (uart_is_readable(uart1)) {  // uart1 = UART2相当
        uint8_t byte = uart_getc(uart1);
        if (g_leptrino_sensor) {
            g_leptrino_sensor->on_uart_rx_interrupt(byte);
        }
    }
}

// Leptrinoセンサ初期化
void init_leptrino_sensor() {
    g_leptrino_sensor = new LeptrinoSensor(uart1, 460800, LEPTRINO_TX_PIN, LEPTRINO_RX_PIN);
    
    if (g_leptrino_sensor->connect()) {
        // UART割り込み設定
        irq_set_exclusive_handler(UART1_IRQ, on_uart2_rx);
        irq_set_enabled(UART1_IRQ, true);
        uart_set_irq_enables(uart1, true, false);  // RX割り込みのみ有効
        
        printf("Leptrino sensor initialized successfully\n");
    } else {
        printf("Failed to initialize Leptrino sensor\n");
    }
}

int main()
{
    stdio_init_all();

    initSerial();
    initCanbus();
    rs422_init(460800, RS422_TXP_PIN);
    init_leptrino_sensor();  // Leptrinoセンサ初期化を追加

    BaseType_t xReturnedMessageTranslator, xReturnedSubtask;
    TaskHandle_t xHandleMessageTranslator = NULL, xHandleSubtask = NULL;
    UBaseType_t uxCoreAffinityMask;

    xReturnedMessageTranslator = xTaskCreate(
                    vTaskMessageTranslator,       /* Function that implements the task. */
                    "Message Translator task",    /* Text name for the task. */
                    512,                    /* Stack size in words, not bytes. */
                    ( void * ) 1,           /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,       /* Priority at which the task is created. */
                    &xHandleMessageTranslator );


    // force to run Modbus Slave task on core1
    uxCoreAffinityMask = ( ( 1 << 1 ));
    vTaskCoreAffinitySet( xHandleMessageTranslator, uxCoreAffinityMask );

    xReturnedSubtask = xTaskCreate(
                    vTaskSubtask,       /* Function that implements the task. */
                    "Subtask",    /* Text name for the task. */
                    512,                    /* Stack size in words, not bytes. */
                    ( void * ) 1,           /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,       /* Priority at which the task is created. */
                    &xHandleSubtask );

    ModbusDATA[REG_SET_SPEED] = target_speed * 255 / 20000;

    ModbusH.uModbusType = MB_SLAVE;
    ModbusH.port = uart0;
    ModbusH.u8id = 8; // For master it must be 0
    ModbusH.u16timeOut = 1000;
    // ModbusH.EN_Port = NULL;
    ModbusH.EN_Port = (uint16_t *) 1; //enables the RS485 ChipSelect
    ModbusH.EN_Pin = MODBUS_EN_PIN; //Pi controlling RS485 ChipSelect
    ModbusH.u16regs = ModbusDATA;
    ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
    ModbusH.xTypeHW = USART_HW;
    //Initialize Modbus library
    ModbusInit(&ModbusH);
    //Start capturing traffic on serial Port
    ModbusStart(&ModbusH);

    CanTxSphrHandle = xSemaphoreCreateBinary();
    xSemaphoreGive(CanTxSphrHandle);

    vTaskStartScheduler();

    while(1)
    {
        configASSERT(0);    /* We should never get here */
    }
}
