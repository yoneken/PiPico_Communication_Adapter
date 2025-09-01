#include "leptrino.h"
#include "pico/time.h"
#include <algorithm>
#include <cstring>

extern "C" {
  #include "../rs422/rs422_tx.h"
}

LeptrinoSensor::LeptrinoSensor(uart_inst_t* uart_inst, uint32_t baudrate, 
                               uint8_t tx_pin, uint8_t rx_pin)
    : uart_(uart_inst), baudrate_(baudrate), tx_pin_(tx_pin), rx_pin_(rx_pin),
      connected_(false), continuous_mode_(false), has_rating_(false), has_offset_(false),
      has_latest_rating_(false), has_latest_force_data_(false),
      packet_length_(0), packet_state_(PACKET_STATE_IDLE), expected_packet_length_(0) {
    memset(&sensor_rating_, 0, sizeof(sensor_rating_));
    memset(&offset_data_, 0, sizeof(offset_data_));
    memset(&latest_sensor_rating_, 0, sizeof(latest_sensor_rating_));
    memset(&latest_force_data_, 0, sizeof(latest_force_data_));
}

LeptrinoSensor::~LeptrinoSensor() {
    disconnect();
}

bool LeptrinoSensor::connect() {
    if (connected_) {
        return true;
    }

    // 送信RS422設定
    rs422_init(baudrate_, tx_pin_);

    // 受信UART初期化
    uart_init(uart_, baudrate_);
    
    // 受信ピン設定
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);
    
    // UART設定
    uart_set_hw_flow(uart_, false, false);
    uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_, true);

    connected_ = true;
    
    return true;
}

void LeptrinoSensor::disconnect() {
    if (!connected_) {
        return;
    }

    if (continuous_mode_) {
        stop_continuous_mode();
    }

    uart_deinit(uart_);
    connected_ = false;
}

bool LeptrinoSensor::get_sensor_rating(SensorRatingData& rating) {
    if (!connected_) {
        return false;
    }

    if (!send_command(CMD_CHECK_RATING)) {
        return false;
    }

    // 割り込みで受信されるまで待機
    uint32_t start_time = get_time_ms();
    const uint32_t timeout_ms = 1000;
    
    while ((get_time_ms() - start_time) < timeout_ms) {
        if (has_latest_rating_) {
            rating = latest_sensor_rating_;
            has_latest_rating_ = false;  // フラグをクリア
            return true;
        }
        sleep_ms(1);
    }
    
    return false;
}

bool LeptrinoSensor::get_handshake_data(ForceData& data) {
    if (!connected_ || !has_rating_) {
        return false;
    }

    if (!send_command(CMD_START_HANDSHAKE)) {
        return false;
    }

    // 割り込みで受信されるまで待機
    uint32_t start_time = get_time_ms();
    const uint32_t timeout_ms = 1000;
    
    while ((get_time_ms() - start_time) < timeout_ms) {
        if (has_latest_force_data_) {
            data = latest_force_data_;
            has_latest_force_data_ = false;  // フラグをクリア
            return true;
        }
        sleep_ms(1);
    }
    
    return false;
}

bool LeptrinoSensor::start_continuous_mode() {
    if (!connected_) {
        return false;
    }

    if (!send_command(CMD_START_CONTINUOUS)) {
        return false;
    }

    // 成功応答の確認は割り込みで行われる
    // 簡易的にコマンド送信成功で判定
    continuous_mode_ = true;
    return true;
}

bool LeptrinoSensor::stop_continuous_mode() {
    if (!connected_ || !continuous_mode_) {
        return true;
    }

    if (!send_command(CMD_STOP_CONTINUOUS)) {
        return false;
    }

    // 成功応答の確認は割り込みで行われる
    // 簡易的にコマンド送信成功で判定
    continuous_mode_ = false;
    return true;
}

bool LeptrinoSensor::get_continuous_data(ForceData& data) {
    if (!connected_ || !continuous_mode_ || !has_rating_) {
        return false;
    }

    return get_latest_force_data(data);
}

void LeptrinoSensor::set_offset(const ForceData& offset_data) {
    offset_data_ = offset_data;
    has_offset_ = true;
}

void LeptrinoSensor::clear_offset() {
    memset(&offset_data_, 0, sizeof(offset_data_));
    has_offset_ = false;
}

bool LeptrinoSensor::capture_current_offset(uint8_t samples) {
    if (!connected_ || !has_rating_) {
        return false;
    }

    float fx_sum = 0.0f, fy_sum = 0.0f, fz_sum = 0.0f;
    float mx_sum = 0.0f, my_sum = 0.0f, mz_sum = 0.0f;
    uint8_t success_count = 0;

    // 一時的にオフセットを無効化
    bool temp_has_offset = has_offset_;
    has_offset_ = false;

    for (uint8_t i = 0; i < samples; i++) {
        ForceData force_data;
        if (get_handshake_data(force_data)) {
            fx_sum += force_data.fx;
            fy_sum += force_data.fy;
            fz_sum += force_data.fz;
            mx_sum += force_data.mx;
            my_sum += force_data.my;
            mz_sum += force_data.mz;
            success_count++;
        }
        sleep_ms(50);  // 短い間隔で取得
    }

    // オフセット状態を復元
    has_offset_ = temp_has_offset;

    if (success_count > 0) {
        ForceData avg_data;
        avg_data.fx = fx_sum / success_count;
        avg_data.fy = fy_sum / success_count;
        avg_data.fz = fz_sum / success_count;
        avg_data.mx = mx_sum / success_count;
        avg_data.my = my_sum / success_count;
        avg_data.mz = mz_sum / success_count;
        avg_data.timestamp = get_time_ms();

        set_offset(avg_data);
        return true;
    }

    return false;
}

bool LeptrinoSensor::send_command(uint8_t command, const uint8_t* data, size_t data_len) {
    if (!connected_) {
        return false;
    }

    // パケット構成: [DLE][STX][LEN][RSV1][CMD][RSV2][DATA][DLE][ETX][BCC]
    size_t packet_idx = 0;
    
    // DLE + STX
    tx_buffer_[packet_idx++] = DLE;
    tx_buffer_[packet_idx++] = STX;
    
    // LEN: RSV1(1) + CMD(1) + RSV2(1) + DATA
    uint8_t length = 3 + data_len;
    tx_buffer_[packet_idx++] = length;
    
    // RSV1 + CMD + RSV2
    tx_buffer_[packet_idx++] = RSV1;
    tx_buffer_[packet_idx++] = command;
    tx_buffer_[packet_idx++] = RSV2;
    
    // DATA
    if (data && data_len > 0) {
        memcpy(&tx_buffer_[packet_idx], data, data_len);
        packet_idx += data_len;
    }
    
    // DLE + ETX
    tx_buffer_[packet_idx++] = DLE;
    tx_buffer_[packet_idx++] = ETX;
    
    // BCC計算（LEN + RSV1 + CMD + RSV2 + DATA + DLE + ETX）
    // ただし、DLEは除外
    uint8_t bcc = calculate_bcc(&tx_buffer_[2], packet_idx - 4);  // LEN から ETX まで（DLE除外）
    tx_buffer_[packet_idx++] = bcc;
    
    // 送信
    rs422_puts((const char*)tx_buffer_, packet_idx);

    return true;
}

bool LeptrinoSensor::get_latest_force_data(ForceData& data) {
    if (has_latest_force_data_) {
        data = latest_force_data_;
        has_latest_force_data_ = false;  // 読み取り後にフラグをクリア
        return true;
    }
    return false;
}

bool LeptrinoSensor::get_latest_sensor_rating(SensorRatingData& rating) {
    if (has_latest_rating_) {
        rating = latest_sensor_rating_;
        has_latest_rating_ = false;  // 読み取り後にフラグをクリア
        return true;
    }
    return false;
}

bool LeptrinoSensor::parse_force_data(const uint8_t* data, size_t data_len, ForceData& force_data) {
    if (data_len < 4) {  // 最小サイズチェック
        return false;
    }

    // 受信データ: RSV1 + CMD + RSV2 + DATA
    const uint8_t* data_part = data + 3;  // RSV1(1) + CMD(1) + RSV2(1) を除く
    size_t payload_len = data_len - 3;

    if (payload_len < 16) {  // 力覚データの最小サイズ
        return false;
    }

    uint8_t receive_status = data[2];  // RSV2（ステータス）
    if (receive_status != RESP_OK) {
        return false;
    }

    // データステータスチェック
    uint8_t data_status = data_part[payload_len - 2];  // 最後から2バイト目がステータス
    if (data_status & 0x01) {  // ROM上の補正データ異常
        return false;
    }
    if (data_status & 0x02) {  // センサ異常
        return false;
    }
    if (data_status & 0x04) {  // センサの出力値が定格値を超過
        return false;
    }

    // 力覚データ解析（Little-endian, int16_t）
    int16_t fx_raw, fy_raw, fz_raw, mx_raw, my_raw, mz_raw;
    
    memcpy(&fx_raw, data_part, 2);
    memcpy(&fy_raw, data_part + 2, 2);
    memcpy(&fz_raw, data_part + 4, 2);
    memcpy(&mx_raw, data_part + 6, 2);
    memcpy(&my_raw, data_part + 8, 2);
    memcpy(&mz_raw, data_part + 10, 2);

    // 物理値に変換
    force_data.fx = fx_raw * sensor_rating_.fx / 10000.0f;
    force_data.fy = fy_raw * sensor_rating_.fy / 10000.0f;
    force_data.fz = fz_raw * sensor_rating_.fz / 10000.0f;
    force_data.mx = mx_raw * sensor_rating_.mx / 10000.0f;
    force_data.my = my_raw * sensor_rating_.my / 10000.0f;
    force_data.mz = mz_raw * sensor_rating_.mz / 10000.0f;
    force_data.timestamp = get_time_ms();

    // オフセット補正
    apply_offset_correction(force_data);

    return true;
}

uint8_t LeptrinoSensor::calculate_bcc(const uint8_t* data, size_t len) {
    uint8_t bcc = 0;
    for (size_t i = 0; i < len; i++) {
        if (data[i] != DLE) {  // DLEは除外
            bcc ^= data[i];
        }
    }
    return bcc;
}

void LeptrinoSensor::apply_offset_correction(ForceData& data) {
    if (has_offset_) {
        data.fx -= offset_data_.fx;
        data.fy -= offset_data_.fy;
        data.fz -= offset_data_.fz;
        data.mx -= offset_data_.mx;
        data.my -= offset_data_.my;
        data.mz -= offset_data_.mz;
    }
}

uint32_t LeptrinoSensor::get_time_ms() {
    return to_ms_since_boot(get_absolute_time());
}

void LeptrinoSensor::on_uart_rx_interrupt(uint8_t byte) {
    switch (packet_state_) {
        case PACKET_STATE_IDLE:
            if (byte == DLE) {
                packet_state_ = PACKET_STATE_DLE1;
                packet_length_ = 0;
            }
            break;
            
        case PACKET_STATE_DLE1:
            if (byte == STX) {
                packet_state_ = PACKET_STATE_STX;
                packet_buffer_[0] = DLE;
                packet_buffer_[1] = STX;
                packet_length_ = 2;
            } else {
                packet_state_ = PACKET_STATE_IDLE;
            }
            break;
            
        case PACKET_STATE_STX:
            packet_buffer_[packet_length_++] = byte;
            expected_packet_length_ = byte + 5; // LEN + DLE + STX + DATA + DLE + ETX + BCC
            packet_state_ = PACKET_STATE_DATA;
            break;
            
        case PACKET_STATE_DATA:
            if (packet_length_ < MAX_RESPONSE_SIZE) {
                packet_buffer_[packet_length_++] = byte;
                
                if (packet_length_ >= expected_packet_length_) {
                    // パケット完了、検証を実行
                    if (validate_and_decode_packet()) {
                        // 有効なパケットが受信され、デコード完了
                    }
                    packet_state_ = PACKET_STATE_IDLE;
                }
            } else {
                // バッファオーバーフロー
                packet_state_ = PACKET_STATE_IDLE;
            }
            break;
    }
}

bool LeptrinoSensor::validate_and_decode_packet() {
    if (packet_length_ < 5) {
        return false;
    }
    
    // DLE + ETX の確認
    uint8_t dle2 = packet_buffer_[packet_length_ - 3];
    uint8_t etx = packet_buffer_[packet_length_ - 2];
    
    if (dle2 != DLE || etx != ETX) {
        return false;
    }
    
    // BCC検証
    uint8_t calculated_bcc = calculate_bcc(&packet_buffer_[2], packet_length_ - 5);
    uint8_t received_bcc = packet_buffer_[packet_length_ - 1];
    
    if (calculated_bcc != received_bcc) {
        return false;
    }
    
    // 有効なパケット、デコード実行
    decode_and_update_data(packet_buffer_, packet_length_);
    return true;
}

void LeptrinoSensor::decode_and_update_data(const uint8_t* data, size_t data_len) {
    if (data_len < 5) {  // 最小パケットサイズ
        return;
    }
    
    // DLE + STX + LEN + ... の形式を期待
    if (data[0] != DLE || data[1] != STX) {
        return;
    }
    
    uint8_t length = data[2];
    if (data_len < length + 5) {  // DLE+STX+LEN+DATA+DLE+ETX+BCC
        return;
    }
    
    // BCCチェック
    uint8_t calculated_bcc = calculate_bcc(&data[2], data_len - 5);  // LEN から ETX まで（DLE除外）
    uint8_t received_bcc = data[data_len - 1];
    if (calculated_bcc != received_bcc) {
        return;  // BCCエラー
    }
    
    // データ部分を抽出: RSV1 + CMD + RSV2 + DATA
    const uint8_t* payload = &data[3];
    size_t payload_len = length;
    
    if (payload_len < 3) {  // RSV1 + CMD + RSV2 の最小サイズ
        return;
    }
    
    uint8_t cmd = payload[1];  // CMD
    uint8_t status = payload[2];  // RSV2（ステータス）
    
    if (status != RESP_OK) {
        return;  // エラー応答
    }
    
    // コマンドに応じてデータを更新
    switch (cmd) {
        case CMD_CHECK_RATING: {
            // センサ定格値の更新
            if (payload_len >= 27) {  // RSV1(1) + CMD(1) + RSV2(1) + STATUS(1) + 6軸データ(4*6)
                const uint8_t* rating_data = &payload[4];  // STATUS(1)をスキップ
                
                memcpy(&latest_sensor_rating_.fx, rating_data, 4);
                memcpy(&latest_sensor_rating_.fy, rating_data + 4, 4);
                memcpy(&latest_sensor_rating_.fz, rating_data + 8, 4);
                memcpy(&latest_sensor_rating_.mx, rating_data + 12, 4);
                memcpy(&latest_sensor_rating_.my, rating_data + 16, 4);
                memcpy(&latest_sensor_rating_.mz, rating_data + 20, 4);
                
                sensor_rating_ = latest_sensor_rating_;
                has_latest_rating_ = true;
                has_rating_ = true;
            }
            break;
        }
        
        case CMD_START_HANDSHAKE:
        case 0x40:  // 連続出力データの場合（実際のコマンド値に合わせて調整）
        {
            // 力覚データの更新
            if (payload_len >= 19) {  // RSV1(1) + CMD(1) + RSV2(1) + 6軸データ(2*6) + 予備(2) + ステータス(1) + 予備(1)
                const uint8_t* force_data_raw = &payload[3];  // RSV1+CMD+RSV2をスキップ
                
                // データステータスチェック
                if (payload_len >= 18) {
                    uint8_t data_status = force_data_raw[payload_len - 6];  // 最後から2バイト目がステータス
                    if (data_status & 0x07) {  // エラービットチェック
                        return;
                    }
                }
                
                // 力覚データ解析（Little-endian, int16_t）
                int16_t fx_raw, fy_raw, fz_raw, mx_raw, my_raw, mz_raw;
                
                memcpy(&fx_raw, force_data_raw, 2);
                memcpy(&fy_raw, force_data_raw + 2, 2);
                memcpy(&fz_raw, force_data_raw + 4, 2);
                memcpy(&mx_raw, force_data_raw + 6, 2);
                memcpy(&my_raw, force_data_raw + 8, 2);
                memcpy(&mz_raw, force_data_raw + 10, 2);
                
                if (has_rating_) {
                    // 物理値に変換
                    latest_force_data_.fx = fx_raw * sensor_rating_.fx / 10000.0f;
                    latest_force_data_.fy = fy_raw * sensor_rating_.fy / 10000.0f;
                    latest_force_data_.fz = fz_raw * sensor_rating_.fz / 10000.0f;
                    latest_force_data_.mx = mx_raw * sensor_rating_.mx / 10000.0f;
                    latest_force_data_.my = my_raw * sensor_rating_.my / 10000.0f;
                    latest_force_data_.mz = mz_raw * sensor_rating_.mz / 10000.0f;
                    latest_force_data_.timestamp = get_time_ms();
                    
                    // オフセット補正
                    apply_offset_correction(latest_force_data_);
                    
                    has_latest_force_data_ = true;
                }
            }
            break;
        }
    }
}
