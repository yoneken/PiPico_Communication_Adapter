#ifndef LEPTRINO_H
#define LEPTRINO_H

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <cstring>
#include <cstdint>

/**
 * @brief センサ定格値構造体
 */
struct SensorRatingData {
    float fx;  // X方向の定格最大値 [N]
    float fy;  // Y方向の定格最大値 [N]
    float fz;  // Z方向の定格最大値 [N]
    float mx;  // X軸回りの定格最大値 [Nm]
    float my;  // Y軸回りの定格最大値 [Nm]
    float mz;  // Z軸回りの定格最大値 [Nm]
};

/**
 * @brief 力覚データ構造体
 */
struct ForceData {
    float fx;  // X方向の力 [N]
    float fy;  // Y方向の力 [N]
    float fz;  // Z方向の力 [N]
    float mx;  // X軸回りのモーメント [Nm]
    float my;  // Y軸回りのモーメント [Nm]
    float mz;  // Z軸回りのモーメント [Nm]
    uint32_t timestamp;  // タイムスタンプ [ms]
};

/**
 * @brief Leptrino力覚センサ通信クラス
 */
class LeptrinoSensor {
public:
    // コマンド定義
    static const uint8_t CMD_CHECK_RATING = 0x2B;      // 定格値確認
    static const uint8_t CMD_START_HANDSHAKE = 0x30;   // データ取得（ハンドシェイク）
    static const uint8_t CMD_START_CONTINUOUS = 0x32;  // データ送信開始（連続出力）
    static const uint8_t CMD_STOP_CONTINUOUS = 0x33;   // データ送信停止（連続出力）
    
    // 応答コード
    static const uint8_t RESP_OK = 0x00;           // 正常終了
    static const uint8_t RESP_ERROR_LEN = 0x01;    // 伝文長異常
    static const uint8_t RESP_ERROR_CMD = 0x02;    // 未定義コマンド受信
    static const uint8_t RESP_ERROR = 0x04;        // その他エラー

    // パケット構成定数
    static const uint8_t DLE = 0x10;  // Data Link Escape
    static const uint8_t STX = 0x02;  // Start of Text
    static const uint8_t ETX = 0x03;  // End of Text
    static const uint8_t RSV1 = 0xFF; // 予約フィールド1
    static const uint8_t RSV2 = 0x00; // 予約フィールド2

    // バッファサイズ
    static const size_t MAX_PACKET_SIZE = 256;
    static const size_t MAX_RESPONSE_SIZE = 128;
    static const size_t UART_RX_BUFFER_SIZE = 120;

    /**
     * @brief コンストラクタ
     * @param uart_inst UART インスタンス（受信専用）
     * @param baudrate ボーレート
     * @param tx_pin RS422送信ピン番号
     * @param rx_pin UART受信ピン番号
     */
    LeptrinoSensor(uart_inst_t* uart_inst, uint32_t baudrate = 460800, 
                   uint8_t tx_pin = 0, uint8_t rx_pin = 1);

    /**
     * @brief デストラクタ
     */
    ~LeptrinoSensor();

    /**
     * @brief センサに接続
     * @return 接続成功時true
     */
    bool connect();

    /**
     * @brief センサから切断
     */
    void disconnect();

    /**
     * @brief 接続状態確認
     * @return 接続中の場合true
     */
    bool is_connected() const { return connected_; }

    /**
     * @brief センサ定格値取得
     * @param rating 定格値格納先
     * @return 取得成功時true
     */
    bool get_sensor_rating(SensorRatingData& rating);

    /**
     * @brief ハンドシェイク方式でデータ取得
     * @param data 力覚データ格納先
     * @return 取得成功時true
     */
    bool get_handshake_data(ForceData& data);

    /**
     * @brief 連続出力方式開始
     * @return 開始成功時true
     */
    bool start_continuous_mode();

    /**
     * @brief 連続出力方式停止
     * @return 停止成功時true
     */
    bool stop_continuous_mode();

    /**
     * @brief 連続出力データ取得（割り込み方式）
     * @param data 力覚データ格納先
     * @return データ取得成功時true
     */
    bool get_continuous_data(ForceData& data);

    /**
     * @brief オフセット値を設定
     * @param offset_data オフセット値
     */
    void set_offset(const ForceData& offset_data);

    /**
     * @brief オフセット値をクリア
     */
    void clear_offset();

    /**
     * @brief 現在のセンサ値をオフセットとして設定
     * @param samples 平均計算に使用するサンプル数
     * @return オフセット取得成功時true
     */
    bool capture_current_offset(uint8_t samples = 10);

    /**
     * @brief オフセット設定状態確認
     * @return オフセット設定済みの場合true
     */
    bool has_offset() const { return has_offset_; }

    /**
     * @brief センサ定格値設定状態確認
     * @return 定格値取得済みの場合true
     */
    bool has_rating() const { return has_rating_; }

    /**
     * @brief UART割り込み用受信関数（ノンブロッキング）
     * @param byte 受信した1バイト
     */
    void on_uart_rx_interrupt(uint8_t byte);

    /**
     * @brief 最新の力覚データ取得
     * @param data 力覚データ格納先
     * @return データが有効な場合true
     */
    bool get_latest_force_data(ForceData& data);

    /**
     * @brief 最新のセンサ定格値取得
     * @param rating 定格値格納先
     * @return データが有効な場合true
     */
    bool get_latest_sensor_rating(SensorRatingData& rating);

private:
    uart_inst_t* uart_;
    uint32_t baudrate_;
    uint8_t tx_pin_;
    uint8_t rx_pin_;
    bool connected_;
    bool continuous_mode_;
    bool has_rating_;
    bool has_offset_;

    SensorRatingData sensor_rating_;
    ForceData offset_data_;
    
    // 最新データ保持用
    SensorRatingData latest_sensor_rating_;
    ForceData latest_force_data_;
    bool has_latest_rating_;
    bool has_latest_force_data_;

    uint8_t tx_buffer_[MAX_PACKET_SIZE];
    
    // パケット解析用状態管理
    enum PacketState {
        PACKET_STATE_IDLE,
        PACKET_STATE_DLE1,
        PACKET_STATE_STX,
        PACKET_STATE_DATA
    };

    PacketState packet_state_;
    uint8_t packet_buffer_[MAX_RESPONSE_SIZE];
    size_t packet_length_;
    size_t expected_packet_length_;

    /**
     * @brief コマンド送信
     * @param command コマンドバイト
     * @param data 追加データ
     * @param data_len データ長
     * @return 送信成功時true
     */
    bool send_command(uint8_t command, const uint8_t* data = nullptr, size_t data_len = 0);

    /**
     * @brief 力覚データ解析
     * @param data 受信データ
     * @param data_len データ長
     * @param force_data 力覚データ格納先
     * @return 解析成功時true
     */
    bool parse_force_data(const uint8_t* data, size_t data_len, ForceData& force_data);

    /**
     * @brief BCCチェックサム計算
     * @param data データ
     * @param len データ長
     * @return チェックサム値
     */
    uint8_t calculate_bcc(const uint8_t* data, size_t len);

    /**
     * @brief オフセット補正適用
     * @param data 補正対象の力覚データ
     */
    void apply_offset_correction(ForceData& data);

    /**
     * @brief ミリ秒取得
     * @return 現在のミリ秒
     */
    uint32_t get_time_ms();

    /**
     * @brief パケット検証とデコード
     * @return 検証成功時true
     */
    bool validate_and_decode_packet();

    /**
     * @brief メッセージデコードと内部データ更新
     * @param data 受信したパケットデータ
     * @param data_len データ長
     */
    void decode_and_update_data(const uint8_t* data, size_t data_len);
};

#endif // LEPTRINO_H
