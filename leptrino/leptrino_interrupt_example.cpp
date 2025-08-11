/*
 * Leptrino力覚センサ割り込み受信使用例
 * Raspberry Pi Pico用
 */

#include "leptrino.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <stdio.h>

// グローバル変数
LeptrinoSensor* g_sensor = nullptr;

// UART割り込みハンドラ
void on_uart_rx() {
    while (uart_is_readable(uart1)) {
        uint8_t byte = uart_getc(uart1);
        if (g_sensor) {
            g_sensor->on_uart_rx_interrupt(byte);
        }
    }
}

int main() {
    // 標準入出力初期化
    stdio_init_all();

    printf("=== Leptrino力覚センサ割り込み受信テスト ===\n");

    // Leptrinoセンサ初期化（UART1, TX=8, RX=9）
    g_sensor = new LeptrinoSensor(uart1, 460800, 8, 9);

    if (!g_sensor->connect()) {
        printf("センサへの接続に失敗しました\n");
        return -1;
    }

    // UART割り込み設定
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);  // RX割り込みのみ有効

    printf("センサに接続しました（割り込み受信モード）\n");

    // センサ定格値を取得（ブロッキング）
    SensorRatingData rating;
    if (g_sensor->get_sensor_rating(rating)) {
        printf("センサ定格値を取得しました\n");
        printf("X方向の定格最大値: %.2f N\n", rating.fx);
        printf("Y方向の定格最大値: %.2f N\n", rating.fy);
        printf("Z方向の定格最大値: %.2f N\n", rating.fz);
        printf("X軸回りの定格最大値: %.2f Nm\n", rating.mx);
        printf("Y軸回りの定格最大値: %.2f Nm\n", rating.my);
        printf("Z軸回りの定格最大値: %.2f Nm\n", rating.mz);
    }

    // オフセット取得
    if (g_sensor->capture_current_offset(5)) {
        printf("オフセット値を設定しました\n");
    }

    // 連続出力モード開始
    if (g_sensor->start_continuous_mode()) {
        printf("連続出力モードを開始しました\n");
        printf("割り込みでデータを受信中...\n");

        uint32_t last_print_time = to_ms_since_boot(get_absolute_time());
        int data_count = 0;

        for (;;) {
            // 完全なパケットが受信されたかチェック
            if (g_sensor->has_complete_packet()) {
                data_count++;
            }

            // 最新の力覚データを取得
            ForceData force_data;
            if (g_sensor->get_latest_force_data(force_data)) {
                uint32_t current_time = to_ms_since_boot(get_absolute_time());
                
                // 1秒毎にデータを表示
                if (current_time - last_print_time >= 1000) {
                    printf("[%d] Fx=%.2fN, Fy=%.2fN, Fz=%.2fN, Mx=%.2fNm, My=%.2fNm, Mz=%.2fNm (受信数:%d)\n",
                           (int)(current_time / 1000), 
                           force_data.fx, force_data.fy, force_data.fz,
                           force_data.mx, force_data.my, force_data.mz,
                           data_count);
                    last_print_time = current_time;
                    data_count = 0;
                }
            }

            sleep_ms(10);  // 短い待機
        }
    } else {
        printf("連続出力モードの開始に失敗しました\n");
    }

    // センサ切断
    g_sensor->disconnect();
    delete g_sensor;

    return 0;
}
