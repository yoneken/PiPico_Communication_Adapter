#include "leptrino.h"
#include "pico/time.h"
#include <algorithm>
#include <cstring>

LeptrinoSensor::LeptrinoSensor(uart_inst_t* uart_inst, uint32_t baudrate, 
                               uint8_t tx_pin, uint8_t rx_pin)
    : uart_(uart_inst), baudrate_(baudrate), tx_pin_(tx_pin), rx_pin_(rx_pin),
      connected_(false), continuous_mode_(false), has_rating_(false), has_offset_(false),
      has_latest_rating_(false), has_latest_force_data_(false),
      uart_rx_head_(0), uart_rx_tail_(0), uart_rx_count_(0),
      packet_length_(0), packet_in_progress_(false) {
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

    // UART初期化
    uart_init(uart_, baudrate_);
    
    // ピン設定
    gpio_set_function(tx_pin_, GPIO_FUNC_UART);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);
    
    // UART設定
    uart_set_hw_flow(uart_, false, false);
    uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_, true);

    connected_ = true;
    sleep_ms(100);  // 接続待機
    
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

bool LeptrinoSensor::get_product_info(ProductInfo& info) {
    if (!connected_) {
        return false;
    }

    if (!send_command(CMD_PRODUCT_INFO)) {
        return false;
    }

    size_t response_len = receive_response(rx_buffer_, sizeof(rx_buffer_));
    if (response_len < 4) {
        return false;
    }

    // 受信データ: RSV1 + CMD + RSV2 + DATA
    const uint8_t* data_part = rx_buffer_ + 2;  // RSV1(1) + CMD(1) を除く
    size_t data_len = response_len - 2;

    if (data_len < 35) {  // 最小データサイズ
        return false;
    }

    uint8_t status = data_part[0];  // RSV2（ステータス）
    if (status != RESP_OK) {
        return false;
    }

    // データ構造: STATUS(1) + 製品型式(16) + シリアル番号(8) + ファームウェア(4) + 出力レート(6)
    const uint8_t* payload = data_part + 1;
    
    memcpy(info.product_name, payload, 16);
    info.product_name[16] = '\0';
    
    memcpy(info.serial_number, payload + 16, 8);
    info.serial_number[8] = '\0';
    
    memcpy(info.firmware_version, payload + 24, 4);
    info.firmware_version[4] = '\0';
    
    memcpy(info.output_rate, payload + 28, 6);
    info.output_rate[6] = '\0';

    return true;
}

bool LeptrinoSensor::get_sensor_rating(SensorRatingData& rating) {
    if (!connected_) {
        return false;
    }

    if (!send_command(CMD_CHECK_RATING)) {
        return false;
    }

    size_t response_len = receive_response(rx_buffer_, sizeof(rx_buffer_));
    if (response_len < 4) {
        return false;
    }

    // 受信データ: RSV1 + CMD + RSV2 + DATA
    const uint8_t* data_part = rx_buffer_ + 2;  // RSV1(1) + CMD(1) を除く
    size_t data_len = response_len - 2;

    if (data_len < 25) {  // 最小データサイズ: STATUS(1) + 6軸データ(4*6)
        return false;
    }

    uint8_t status = data_part[0];  // RSV2（ステータス）
    if (status != RESP_OK) {
        return false;
    }

    // データ構造: STATUS(1) + Fx(4) + Fy(4) + Fz(4) + Mx(4) + My(4) + Mz(4)
    const uint8_t* payload = data_part + 1;
    
    // Little-endianでfloat値を読み取り
    memcpy(&rating.fx, payload, 4);
    memcpy(&rating.fy, payload + 4, 4);
    memcpy(&rating.fz, payload + 8, 4);
    memcpy(&rating.mx, payload + 12, 4);
    memcpy(&rating.my, payload + 16, 4);
    memcpy(&rating.mz, payload + 20, 4);

    sensor_rating_ = rating;
    has_rating_ = true;

    return true;
}

bool LeptrinoSensor::get_handshake_data(ForceData& data) {
    if (!connected_ || !has_rating_) {
        return false;
    }

    if (!send_command(CMD_START_HANDSHAKE)) {
        return false;
    }

    size_t response_len = receive_response(rx_buffer_, sizeof(rx_buffer_));
    if (response_len == 0) {
        return false;
    }

    return parse_force_data(rx_buffer_, response_len, data);
}

bool LeptrinoSensor::start_continuous_mode() {
    if (!connected_) {
        return false;
    }

    if (!send_command(CMD_START_CONTINUOUS)) {
        return false;
    }

    size_t response_len = receive_response(rx_buffer_, sizeof(rx_buffer_));
    if (response_len < 3) {
        return false;
    }

    uint8_t status = rx_buffer_[2];  // RSV2（ステータス）
    if (status == RESP_OK) {
        continuous_mode_ = true;
        return true;
    }

    return false;
}

bool LeptrinoSensor::stop_continuous_mode() {
    if (!connected_ || !continuous_mode_) {
        return true;
    }

    if (!send_command(CMD_STOP_CONTINUOUS)) {
        return false;
    }

    size_t response_len = receive_response(rx_buffer_, sizeof(rx_buffer_));
    if (response_len < 3) {
        return false;
    }

    uint8_t status = rx_buffer_[2];  // RSV2（ステータス）
    if (status == RESP_OK) {
        continuous_mode_ = false;
        return true;
    }

    return false;
}

bool LeptrinoSensor::get_continuous_data(ForceData& data, uint32_t timeout_ms) {
    if (!connected_ || !continuous_mode_ || !has_rating_) {
        return false;
    }

    size_t response_len = receive_response(rx_buffer_, sizeof(rx_buffer_), timeout_ms);
    if (response_len == 0) {
        return false;
    }

    return parse_force_data(rx_buffer_, response_len, data);
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
    uart_write_blocking(uart_, tx_buffer_, packet_idx);
    
    return true;
}

size_t LeptrinoSensor::receive_response(uint8_t* buffer, size_t buffer_size, uint32_t timeout_ms) {
    if (!connected_) {
        return 0;
    }

    uint32_t start_time = get_time_ms();
    size_t packet_idx = 0;

    // DLE待機
    while (get_time_ms() - start_time < timeout_ms) {
        if (uart_read_timeout(&buffer[packet_idx], 1, 10)) {
            if (buffer[packet_idx] == DLE) {
                packet_idx++;
                break;
            }
        }
    }

    if (packet_idx == 0) {
        return 0;  // DLE受信タイムアウト
    }

    // STX受信
    if (!uart_read_timeout(&buffer[packet_idx], 1, timeout_ms - (get_time_ms() - start_time))) {
        return 0;
    }
    if (buffer[packet_idx] != STX) {
        return 0;
    }
    packet_idx++;

    // LEN受信
    if (!uart_read_timeout(&buffer[packet_idx], 1, timeout_ms - (get_time_ms() - start_time))) {
        return 0;
    }
    uint8_t length = buffer[packet_idx];
    packet_idx++;

    // 残りのデータ受信 (RSV1 + CMD + RSV2 + DATA + DLE + ETX + BCC)
    size_t remaining = length + 3;  // DATA + DLE + ETX + BCC
    if (remaining > buffer_size - packet_idx) {
        return 0;  // バッファオーバーフロー
    }

    size_t received = uart_read_timeout(&buffer[packet_idx], remaining, 
                                       timeout_ms - (get_time_ms() - start_time));
    if (received != remaining) {
        return 0;
    }
    packet_idx += received;

    // パケット検証
    uint8_t dle2 = buffer[packet_idx - 3];
    uint8_t etx = buffer[packet_idx - 2];
    uint8_t received_bcc = buffer[packet_idx - 1];

    if (dle2 != DLE || etx != ETX) {
        return 0;
    }

    // BCC検証
    uint8_t calculated_bcc = calculate_bcc(&buffer[2], packet_idx - 5);  // LEN から ETX まで（DLE除外）
    if (calculated_bcc != received_bcc) {
        return 0;
    }

    // RSV1 + CMD + RSV2 + DATA 部分を返す
    size_t data_start = 3;  // DLE + STX + LEN をスキップ
    size_t data_len = length;
    memmove(buffer, &buffer[data_start], data_len);

    return data_len;
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

size_t LeptrinoSensor::uart_read_timeout(uint8_t* buffer, size_t len, uint32_t timeout_ms) {
    uint32_t start_time = get_time_ms();
    size_t received = 0;

    while (received < len && (get_time_ms() - start_time) < timeout_ms) {
        if (uart_is_readable(uart_)) {
            buffer[received++] = uart_getc(uart_);
        } else {
            sleep_ms(1);
        }
    }

    return received;
}

uint32_t LeptrinoSensor::get_time_ms() {
    return to_ms_since_boot(get_absolute_time());
}

void LeptrinoSensor::on_uart_rx_interrupt(uint8_t byte) {
    // リングバッファに1バイト追加（ノンブロッキング）
    if (uart_rx_count_ < UART_RX_BUFFER_SIZE) {
        uart_rx_buffer_[uart_rx_head_] = byte;
        uart_rx_head_ = (uart_rx_head_ + 1) % UART_RX_BUFFER_SIZE;
        uart_rx_count_++;
    }
    // バッファオーバーフローの場合、古いデータを破棄
    else {
        uart_rx_buffer_[uart_rx_head_] = byte;
        uart_rx_head_ = (uart_rx_head_ + 1) % UART_RX_BUFFER_SIZE;
        uart_rx_tail_ = (uart_rx_tail_ + 1) % UART_RX_BUFFER_SIZE;
    }
}

bool LeptrinoSensor::has_complete_packet() {
    // 受信バッファから完全なパケットを抽出を試行
    uint8_t temp_packet[MAX_RESPONSE_SIZE];
    size_t packet_size = extract_complete_packet(temp_packet, sizeof(temp_packet));
    
    if (packet_size > 0) {
        // 完全なパケットが見つかった場合、デコードして内部データを更新
        decode_and_update_data(temp_packet, packet_size);
        return true;
    }
    
    return false;
}

size_t LeptrinoSensor::extract_complete_packet(uint8_t* packet_data, size_t max_size) {
    if (uart_rx_count_ < 4) {  // 最小パケットサイズ
        return 0;
    }
    
    size_t temp_tail = uart_rx_tail_;
    size_t temp_count = uart_rx_count_;
    size_t packet_idx = 0;
    bool found_dle_stx = false;
    uint8_t expected_length = 0;
    
    // DLE + STX を探す
    while (temp_count > 0 && packet_idx < max_size - 1) {
        uint8_t byte1 = uart_rx_buffer_[temp_tail];
        temp_tail = (temp_tail + 1) % UART_RX_BUFFER_SIZE;
        temp_count--;
        
        if (!found_dle_stx) {
            if (byte1 == DLE && temp_count > 0) {
                uint8_t byte2 = uart_rx_buffer_[temp_tail];
                if (byte2 == STX) {
                    packet_data[packet_idx++] = byte1;  // DLE
                    packet_data[packet_idx++] = byte2;  // STX
                    temp_tail = (temp_tail + 1) % UART_RX_BUFFER_SIZE;
                    temp_count--;
                    found_dle_stx = true;
                }
            }
        } else {
            packet_data[packet_idx++] = byte1;
            
            // LENを取得（DLE+STXの次のバイト）
            if (packet_idx == 3) {
                expected_length = byte1;
            }
            
            // 期待される長さに達したかチェック
            if (packet_idx >= 3 && packet_idx >= (expected_length + 5)) {  // DLE+STX+LEN+DATA+DLE+ETX+BCC
                // パケット完了の検証
                if (packet_idx >= 5) {
                    uint8_t dle2 = packet_data[packet_idx - 3];
                    uint8_t etx = packet_data[packet_idx - 2];
                    
                    if (dle2 == DLE && etx == ETX) {
                        // 完全なパケットが見つかった
                        // リングバッファの読み取り位置を更新
                        size_t consumed = uart_rx_count_ - temp_count;
                        uart_rx_tail_ = (uart_rx_tail_ + consumed) % UART_RX_BUFFER_SIZE;
                        uart_rx_count_ = temp_count;
                        
                        return packet_idx;
                    }
                }
            }
        }
    }
    
    return 0;  // 完全なパケットが見つからない
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

bool LeptrinoSensor::get_latest_force_data(ForceData& data) {
    if (has_latest_force_data_) {
        data = latest_force_data_;
        return true;
    }
    return false;
}

bool LeptrinoSensor::get_latest_sensor_rating(SensorRatingData& rating) {
    if (has_latest_rating_) {
        rating = latest_sensor_rating_;
        return true;
    }
    return false;
}
