#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "Modbus.h"

extern "C" {
  #include "can2040.h"
  #include "hardware/irq.h"
}

#define MODBUS_TX_PIN 4
#define MODBUS_RX_PIN 5
#define MODBUS_EN_PIN 3
#define CANBUS_RX_PIN 6
#define CANBUS_TX_PIN 7

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
Fn100, enable the gripper, 0-1, -, 0, immediately, 0x0100
Fn101, control mode, 0-2 0：location 1：speed, -, 0, immediately 0x0101
Fn109, fault reset, 0-1, -, 0, immediately, 0x0109

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

static struct can2040 cbus;

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    printf("Core %u: Can Receive ID %d :(%d) %d\n", get_core_num(), (msg->id >> 7) & 0xF, (msg->id >> 1) & 0x3F, msg->dlc);
}

static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void vTaskModbusSlave( void * pvParameters )
{
    for(;;)
    {
        if(xSemaphoreTake(ModbusH.ModBusSphrHandle , portMAX_DELAY) == pdTRUE){
            xSemaphoreGive(ModbusH.ModBusSphrHandle);
        }
        vTaskDelay(10);
    }
}

SemaphoreHandle_t CanTxSphrHandle = NULL;

void vTaskCanTransmit( void * pvParameters )
{
    can2040_msg send_respond_encoder_data_msg;
    send_respond_encoder_data_msg.id = (0 << 7) | (61 << 1); // 0 is the slave address, 61 is the function code
    send_respond_encoder_data_msg.dlc = 5; // Data Length Code
    send_respond_encoder_data_msg.data[0] = 0x0A;
    send_respond_encoder_data_msg.data[1] = 0x14;
    send_respond_encoder_data_msg.data[2] = 0x01;
    send_respond_encoder_data_msg.data[3] = 0xF4;
    send_respond_encoder_data_msg.data[4] = 0xC0;
    int result = 0;
    for(;;)
    {
        if(xSemaphoreTake(CanTxSphrHandle , portMAX_DELAY) == pdTRUE){
            switch (flag) {
                case 0:
                    // Calibration
                    send_respond_encoder_data_msg.id = (0 << 7) | (62 << 1);
                    send_respond_encoder_data_msg.dlc = 0;
                    flag = 1;
                    break;
                case 1:
                    send_respond_encoder_data_msg.id = (0 << 7) | (61 << 1);
                    send_respond_encoder_data_msg.dlc = 5;
                    send_respond_encoder_data_msg.data[0] = 0x64;
                    flag = 2;
                    break;
                case 2:
                    send_respond_encoder_data_msg.id = (0 << 7) | (61 << 1);
                    send_respond_encoder_data_msg.dlc = 5;
                    send_respond_encoder_data_msg.data[0] = 0x0A;
                    flag = 3;
                    break;
                case 3:
                    send_respond_encoder_data_msg.id = (0 << 7) | (28 << 1) | CAN2040_ID_RTR;
                    send_respond_encoder_data_msg.dlc = 0;
                    flag = 0;
                    break;
            }
            result = can2040_transmit(&cbus, &send_respond_encoder_data_msg);
            printf("Core %u: CanTransmit %d %d\n", get_core_num(), result, counter++);
            xSemaphoreGive(CanTxSphrHandle);
        }
        vTaskDelay(10);
    }
}



void initSerial()
{
    // Modbus Serial Port
    uart_init(uart1, 100000);
    gpio_set_function(MODBUS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MODBUS_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(uart1, false);
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


int main()
{
    stdio_init_all();

    initSerial();
    initCanbus();

    BaseType_t xReturnedModbusSlave, xReturnedCanTransmit;
    TaskHandle_t xHandleModbusSlave = NULL, xHandleCanTransmit = NULL;
    UBaseType_t uxCoreAffinityMask;

    xReturnedModbusSlave = xTaskCreate(
                    vTaskModbusSlave,       /* Function that implements the task. */
                    "Modbus Slave task",    /* Text name for the task. */
                    512,                    /* Stack size in words, not bytes. */
                    ( void * ) 1,           /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,       /* Priority at which the task is created. */
                    &xHandleModbusSlave );


    // force to run Modbus Slave task on core1
    uxCoreAffinityMask = ( ( 1 << 1 ));
    vTaskCoreAffinitySet( xHandleModbusSlave, uxCoreAffinityMask );

    ModbusH.uModbusType = MB_SLAVE;
    ModbusH.port = uart1;
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

    xReturnedCanTransmit = xTaskCreate(
                    vTaskCanTransmit,
                    "Can Transmit task",
                    512,             /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,/* Priority at which the task is created. */
                    &xHandleCanTransmit );

    CanTxSphrHandle = xSemaphoreCreateBinary();
    xSemaphoreGive(CanTxSphrHandle);


    vTaskStartScheduler();

    while(1)
    {
        configASSERT(0);    /* We should never get here */
    }
}
