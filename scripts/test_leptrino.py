#!/usr/bin/env python3
"""
Leptrino力覚センサ通信サンプルプログラム
- 製品情報取得
- ハンドシェイク方式でのデータ取得
- 連続出力方式でのデータ取得
"""

import serial
import time
import struct
import threading
from typing import Optional, Tuple, List, Callable
from dataclasses import dataclass

@dataclass
class SensorRatingData:
    """センサ定格値クラス"""
    fx: float  # X方向の定格最大値 [N]
    fy: float  # Y方向の定格最大値 [N]
    fz: float  # Z方向の定格最大値 [N]
    mx: float  # X軸回りの定格最大値 [Nm]
    my: float  # Y軸回りの定格最大値 [Nm]
    mz: float  # Z軸回りの定格最大値 [Nm]

@dataclass
class ForceData:
    """力覚データクラス"""
    fx: float  # X方向の力 [N]
    fy: float  # Y方向の力 [N]
    fz: float  # Z方向の力 [N]
    mx: float  # X軸回りのモーメント [Nm]
    my: float  # Y軸回りのモーメント [Nm]
    mz: float  # Z軸回りのモーメント [Nm]
    timestamp: float

@dataclass
class ProductInfo:
    """製品情報クラス"""
    product_name: str
    serial_number: str
    firmware_version: str
    output_rate: str

class LeptrinoSensor:
    """Leptrino力覚センサ通信クラス"""
    
    # コマンド定義（一般的なLeptrinoセンサのコマンド例）
    CMD_PRODUCT_INFO = b'\x2A'      # 製品情報取得
    CMD_CHECK_RATING = b'\x2B'      # 定格値確認
    CMD_START_HANDSHAKE = b'\x30'   # データ取得（ハンドシェイク）
    CMD_START_CONTINUOUS = b'\x32'  # データ送信開始（連続出力）
    CMD_STOP_CONTINUOUS = b'\x33'   # データ送信停止（連続出力）
    
    # 応答コード
    RESP_OK = b'\x00'           # 正常終了
    RESP_ERROR_LEN = b'\x01'    # 伝文長異常
    RESP_ERROR_CMD = b'\x02'    # 未定義コマンド受信
    RESP_ERROR = b'\x04'        # その他エラー

    # パケット構成: [DLE][STX][LEN][RSV1][CMD][RSV2][DATA][DLE][ETX][BCC]
    DLE = b'\x10'  # Data Link Escape
    STX = b'\x02'  # Start of Text
    ETX = b'\x03'  # End of Text
    RSV1 = b'\xFF'  # 予約フィールド1
    RSV2 = b'\x00'  # 予約フィールド2

    def __init__(self, port: str = 'COM8', baudrate: int = 460800, timeout: float = 1.0) -> None:
        """
        初期化
        
        Args:
            port: シリアルポート名
            baudrate: ボーレート
            timeout: タイムアウト時間[秒]
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.continuous_mode = False
        self.data_callback: Optional[Callable[[ForceData], None]] = None
        self.receive_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.sensor_rating_data: Optional[SensorRatingData] = None
        
    def connect(self) -> bool:
        """
        センサに接続
        
        Returns:
            接続成功時True
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=8,
                parity='N',
                stopbits=1
            )
            print(f"Leptrino力覚センサに接続しました: {self.port} @ {self.baudrate}bps")
            time.sleep(0.1)  # 接続待機
            return True
            
        except serial.SerialException as e:
            print(f"接続エラー: {e}")
            return False
    
    def disconnect(self) -> None:
        """センサから切断"""
        self.stop_continuous_mode()
        
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("センサから切断しました")
    
    def _send_command(self, command: bytes, data: bytes = b'') -> bool:
        """
        コマンド送信
        パケット構成: [DLE][STX][LEN][RSV1][CMD][RSV2][DATA][DLE][ETX][BCC]
        
        Args:
            command: コマンドバイト
            data: 追加データ
            
        Returns:
            送信成功時True
        """
        if not self.serial or not self.serial.is_open:
            print("センサが接続されていません")
            return False
        
        try:            
            # LEN + RSV1 + CMD + RSV2 + DATA の長さ
            length = len(data) + 4  # LEN + RSV1(1) + CMD(1) + RSV2(1) + DATA
            rsv1_cmd_rsv2_data = self.RSV1 + command + self.RSV2 + data
            
            # BCCの対象: LEN + RSV1 + CMD + RSV2 + DATA + DLE + ETX
            bcc_target = struct.pack('B', length) + rsv1_cmd_rsv2_data + self.DLE + self.ETX
            # bcc_target の中にDLEが含まれたら除外する処理
            bcc_target = bcc_target.replace(self.DLE, b'')

            # BCC計算（XOR）
            bcc = 0
            for byte in bcc_target:
                bcc ^= byte
            
            # パケット構成: [DLE][STX][LEN][RSV1][CMD][RSV2][DATA][DLE][ETX][BCC]
            packet = (self.DLE + self.STX + struct.pack('B', length) + 
                     rsv1_cmd_rsv2_data + self.DLE + self.ETX + struct.pack('B', bcc))
            
            self.serial.write(packet)
            print(f"送信: {packet.hex()}")
            return True
            
        except Exception as e:
            print(f"コマンド送信エラー: {e}")
            return False
    
    def _receive_response(self, timeout: float = None) -> Optional[bytes]:
        """
        応答受信
        パケット構成: [DLE][STX][LEN][RSV1][CMD][RSV2][DATA][DLE][ETX][BCC]
        
        Args:
            timeout: タイムアウト時間
            
        Returns:
            受信データ（RSV1+CMD+RSV2+DATA部分）、エラー時None
        """
        if not self.serial or not self.serial.is_open:
            return None
        
        original_timeout = self.serial.timeout
        if timeout:
            self.serial.timeout = timeout
        
        try:
            # DLEを待機
            while True:
                dle1 = self.serial.read(1)
                if not dle1:
                    return None
                if dle1 == self.DLE:
                    break
            
            # STX読み取り
            stx = self.serial.read(1)
            if stx != self.STX:
                return None
            
            # 長さ読み取り
            length_byte = self.serial.read(1)
            if not length_byte:
                return None
            
            length = struct.unpack('B', length_byte)[0]
            
            # 残りのパケットを一括読み取り
            # (LEN + RSV1 + CMD + RSV2 + DATA) + DLE + ETX= length + 2バイト
            remaining_data = self.serial.read(length + 2)
            if len(remaining_data) != length + 2:
               print(f"データ長不足: 期待値{length + 2}, 実際{len(remaining_data)}")
            
            # パケット全体をバッファに格納
            full_packet = dle1 + stx + length_byte + remaining_data
            
            # パケット解析
            rsv1_cmd_rsv2_data = remaining_data[:length+3]  # RSV1 + CMD + RSV2 + DATA
            rsv2 = rsv1_cmd_rsv2_data[2:3]  # RSV2
            dle2 = remaining_data[-3:-2]  # DLE
            etx = remaining_data[-2:-1]  # ETX
            bcc = remaining_data[-1:]  # BCC

            # RSV2確認
            if rsv2 == self.RESP_OK:
                print("応答: 正常終了")
            elif rsv2 == self.RESP_ERROR_LEN:
                print("応答: データ長エラー")
            elif rsv2 == self.RESP_ERROR_CMD:
                print("応答: 未定義コマンド受信")
            elif rsv2 == self.RESP_ERROR:
                print("応答: その他エラー")

            # BCC検証（LEN + RSV1 + CMD + RSV2 + DATA + DLE + ETX）
            calculated_bcc = 0
            bcc_target = length_byte + rsv1_cmd_rsv2_data[:-3] + dle2 + etx
            # bcc_target の中にDLEが含まれたら除外する処理
            bcc_target = bcc_target.replace(self.DLE, b'')
            for byte in bcc_target:
                calculated_bcc ^= byte
            
            received_bcc = struct.unpack('B', bcc)[0]
            if calculated_bcc != received_bcc:
                print(f"BCCエラー: 期待値{calculated_bcc:02x}, 実際{received_bcc:02x}")
                return None
            
            print(f"受信: {full_packet.hex()}")
            return rsv1_cmd_rsv2_data[:-3]
            
        except Exception as e:
            print(f"応答受信エラー: {e}")
            return None
        
        finally:
            self.serial.timeout = original_timeout
    
    def get_product_info(self) -> Optional[ProductInfo]:
        """
        製品情報取得
        
        Returns:
            製品情報、エラー時None
        """
        if not self._send_command(self.CMD_PRODUCT_INFO):
            return None
        
        response = self._receive_response()
        if not response:
            print("製品情報の取得に失敗しました")
            return None
        
        try:
            # 受信データ: RSV1 + CMD + RSV2 + DATA
            if len(response) < 4:  # 最小サイズチェック（RSV1 + CMD + RSV2 + 最小DATA）
                print("製品情報データが不正です")
                return None
            
            # RSV1(1) + CMD(1) を除いた実際のDATA部分
            data_part = response[2:]
            
            if len(data_part) < 35:  # DATAの最小サイズチェック
                print(f"データ部分が不足: {len(data_part)}バイト")
                return None
            
            # データ構造例（実際の仕様に合わせて調整）
            # 応答ステータス(1) + 製品型式(16) + シリアル番号(8) + ファームウェア(4) + 出力レート(6)
            status = data_part[0]
            if status != 0x00:  # 正常応答でない場合
                print(f"センサ応答エラー: 0x{status:02x}")
                return None
                
            product_name = data_part[1:17].decode('ascii', errors='ignore').strip('\x00')
            serial_number = data_part[17:25].decode('ascii', errors='ignore').strip('\x00')
            firmware_version = data_part[25:29].decode('ascii', errors='ignore').strip('\x00')
            output_rate = data_part[29:35].decode('ascii', errors='ignore').strip('\x00')

            return ProductInfo(
                product_name=product_name,
                serial_number=serial_number,
                firmware_version=firmware_version,
                output_rate=output_rate,
            )
            
        except Exception as e:
            print(f"製品情報解析エラー: {e}")
            print(f"受信データ: {response.hex()}")
            return None
    
    def get_sensor_rating(self) -> Optional[SensorRatingData]:
        """
        センサ定格値取得
        
        Returns:
            センサ定格値、エラー時None
        """
        if not self._send_command(self.CMD_CHECK_RATING):
            return None
        
        response = self._receive_response()
        if not response:
            print("センサ定格値の取得に失敗しました")
            return None
        
        try:
            # 受信データ: RSV1 + CMD + RSV2 + DATA
            if len(response) < 4:  # 最小サイズチェック（RSV1 + CMD + RSV2 + 最小DATA）
                print("センサ定格値データが不正です")
                return None
            
            # RSV1(1) + CMD(1) を除いた実際のDATA部分
            data_part = response[2:]
            
            if len(data_part) < 21:  # DATAの最小サイズチェック
                print(f"データ部分が不足: {len(data_part)}バイト")
                return None
            
            # データ構造例（実際の仕様に合わせて調整）
            # STATUS(1) + Fx(4) + Fy(4) + Fz(4) + Mx(4) + My(4) + Mz(4)
            status = data_part[0]
            if status != 0x00:  # 正常応答でない場合
                print(f"センサ応答エラー: 0x{status:02x}")
                return None
            
            fx = struct.unpack('<f', data_part[1:5])[0]
            fy = struct.unpack('<f', data_part[5:9])[0]
            fz = struct.unpack('<f', data_part[9:13])[0]
            mx = struct.unpack('<f', data_part[13:17])[0]
            my = struct.unpack('<f', data_part[17:21])[0]
            mz = struct.unpack('<f', data_part[21:25])[0]

            self.sensor_rating_data = SensorRatingData(
                fx=fx,
                fy=fy,
                fz=fz,
                mx=mx,
                my=my,
                mz=mz,
            )
            return self.sensor_rating_data
            
        except Exception as e:
            print(f"センサ定格値解析エラー: {e}")
        
    def get_handshake_data(self) -> Optional[ForceData]:
        """
        ハンドシェイク方式でデータ取得
        
        Returns:
            力覚データ、エラー時None
        """        
        if not self._send_command(self.CMD_START_HANDSHAKE):
            return None
        
        response = self._receive_response()
        if not response:
            return None
        
        return self._parse_force_data(response)
    
    def start_continuous_mode(self, callback: Optional[Callable[[ForceData], None]] = None) -> bool:
        """
        連続出力方式開始
        
        Args:
            callback: データ受信時のコールバック関数
            
        Returns:
            開始成功時True
        """
        if not self._send_command(self.CMD_START_CONTINUOUS):
            return False
        
        response = self._receive_response()
        if response and len(response) >= 3:
            # RSV1 + CMD + DATA(STATUS)
            status = response[2]  # RSV2（ステータス）
            if status == 0x00:  # 正常終了
                self.continuous_mode = True
                self.data_callback = callback
                
                # 受信スレッド開始
                self.stop_event.clear()
                self.receive_thread = threading.Thread(target=self._continuous_receive_loop)
                self.receive_thread.daemon = True
                self.receive_thread.start()
                
                print("連続出力方式を開始しました")
                return True
            else:
                print(f"連続出力開始エラー: 0x{status:02x}")
                return False
        else:
            print("連続出力方式の開始に失敗しました")
            return False
    
    def stop_continuous_mode(self) -> bool:
        """
        連続出力方式停止
        
        Returns:
            停止成功時True
        """
        if not self.continuous_mode:
            return True
        
        # 受信スレッド停止
        self.stop_event.set()
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)
        
        if not self._send_command(self.CMD_STOP_CONTINUOUS):
            return False
        
        response = self._receive_response()
        if response and len(response) >= 3:
            # RSV1 + CMD + DATA(STATUS)
            status = response[2]  # RSV2（ステータス）
            if status == 0x00:  # 正常終了
                self.continuous_mode = False
                print("連続出力方式を停止しました")
                return True
            else:
                print(f"連続出力停止エラー: 0x{status:02x}")
                return False
        else:
            print("連続出力方式の停止に失敗しました")
            return False
    
    def _continuous_receive_loop(self) -> None:
        """連続出力データ受信ループ"""
        while not self.stop_event.is_set():
            try:
                response = self._receive_response(timeout=0.1)
                if response:
                    force_data = self._parse_force_data(response)
                    if force_data and self.data_callback:
                        self.data_callback(force_data)
                        
            except Exception as e:
                print(f"連続受信エラー: {e}")
                time.sleep(0.01)
    
    def _parse_force_data(self, data: bytes) -> Optional[ForceData]:
        """
        力覚データ解析
        
        Args:
            data: 受信データ（RSV1 + CMD + RSV2 + DATA）
            
        Returns:
            力覚データ、エラー時None
        """
        try:
            # 受信データ: RSV1 + CMD + RSV2 + DATA
            if len(data) < 4:  # 最小サイズチェック（RSV1 + CMD + RSV2 + 最小DATA）
                print(f"データサイズ不足: {len(data)}バイト")
                return None
            
            # RSV1(1) + CMD(1) + RSV2(1) を除いた実際のDATA部分
            data_part = data[3:]
            
            # データ構造: 6軸データ(Fx,Fy,Fz,Mx,My,Mz: 各2バイト) + 予備(2バイト) + ステータス(1バイト) + 予備(1バイト) = 最低16バイト
            if len(data_part) < 16:
                print(f"力覚データ部分が不足: {len(data_part)}バイト")
                return None
            
            # 受信ステータスチェック
            receive_status = data[2]  # RSV2（ステータス）
            if receive_status != 0x00:  # 正常応答でない場合
                print(f"力覚データ応答エラー: 0x{receive_status:02x}")
                return None
            
            # データステータスチェック
            # Bit0: ROM上の補正データ異常
            # Bit1: センサ異常
            # Bit2: センサの出力値が定格値を超過
            # Bit3: 予備（不定）
            # Bit4: 予備（不定）
            # Bit5: 外部入力信号検出（対応機種のみ）
            # Bit6: 予備（不定）
            # Bit7: 予備（不定）
            data_status = data_part[-2]  # 最後から2バイト目がステータス
            if data_status & 0x01:
                print("ROM上の補正データ異常")
                return None
            if data_status & 0x02:
                print("センサ異常")
                return None
            if data_status & 0x04:
                print("センサの出力値が定格値を超過")
                return None

            # 力覚データ解析（Little-endian, int型）
            # Fx(2) + Fy(2) + Fz(2) + Mx(2) + My(2) + Mz(2) + 予備(2) + ステータス(1) + 予備(1)
            fx = struct.unpack('<h', data_part[0:2])[0]
            fy = struct.unpack('<h', data_part[2:4])[0]
            fz = struct.unpack('<h', data_part[4:6])[0]
            mx = struct.unpack('<h', data_part[6:8])[0]
            my = struct.unpack('<h', data_part[8:10])[0]
            mz = struct.unpack('<h', data_part[10:12])[0]
            
            return ForceData(
                fx=fx * self.sensor_rating_data.fx / 10000, fy=fy * self.sensor_rating_data.fy / 10000, fz=fz * self.sensor_rating_data.fz / 10000,
                mx=mx * self.sensor_rating_data.mx / 10000, my=my * self.sensor_rating_data.my / 10000, mz=mz * self.sensor_rating_data.mz / 10000,
                timestamp=time.time()
            )
            
        except Exception as e:
            print(f"データ解析エラー: {e}")
            print(f"受信データ: {data.hex()}")
            return None
    

def continuous_data_callback(force_data: ForceData) -> None:
    """連続出力データ受信時のコールバック関数"""
    print(f"連続データ: Fx={force_data.fx:6.2f}N, Fy={force_data.fy:6.2f}N, Fz={force_data.fz:6.2f}N, "
          f"Mx={force_data.mx:6.2f}Nm, My={force_data.my:6.2f}Nm, Mz={force_data.mz:6.2f}Nm")


def main() -> None:
    """メイン関数"""
    # シリアルポート設定（環境に合わせて変更）
    SERIAL_PORT = 'COM6'
    BAUDRATE = 460800
    
    print("=== Leptrino力覚センサ通信テスト ===")
    
    # センサインスタンス作成
    sensor = LeptrinoSensor(port=SERIAL_PORT, baudrate=BAUDRATE)
    
    try:
        # 接続
        if not sensor.connect():
            print("センサへの接続に失敗しました")
            return
        
        # 1. 製品情報取得
        print("\n1. 製品情報取得")
        print("-" * 30)
        product_info = sensor.get_product_info()
        if product_info:
            print(f"製品名: {product_info.product_name}")
            print(f"シリアル番号: {product_info.serial_number}")
            print(f"ファームウェアバージョン: {product_info.firmware_version}")
            print(f"出力レート: {product_info.output_rate}N")
        else:
            print("製品情報の取得に失敗しました")
            return

        # 2. センサ定格値取得
        print("\n2. センサ定格値取得")
        print("-" * 30)
        sensor_rating = sensor.get_sensor_rating()
        if sensor_rating:
            print(f"X方向の定格最大値: {sensor_rating.fx:6.2f}N")
            print(f"Y方向の定格最大値: {sensor_rating.fy:6.2f}N")
            print(f"Z方向の定格最大値: {sensor_rating.fz:6.2f}N")
            print(f"X軸回りの定格最大値: {sensor_rating.mx:6.2f}Nm")
            print(f"Y軸回りの定格最大値: {sensor_rating.my:6.2f}Nm")
            print(f"Z軸回りの定格最大値: {sensor_rating.mz:6.2f}Nm")
        else:
            print("センサ定格値の取得に失敗しました")
            return

        # 3. ハンドシェイク方式テスト
        print("\n3. ハンドシェイク方式テスト")
        print("-" * 30)
        print("5回データを取得します...")
        for i in range(5):
            force_data = sensor.get_handshake_data()
            if force_data:
                print(f"[{i+1}] Fx={force_data.fx:6.2f}N, Fy={force_data.fy:6.2f}N, Fz={force_data.fz:6.2f}N, "
                        f"Mx={force_data.mx:6.2f}Nm, My={force_data.my:6.2f}Nm, Mz={force_data.mz:6.2f}Nm")
            else:
                print(f"[{i+1}] データ取得に失敗しました")
            time.sleep(0.1)

        # 4. 連続出力方式テスト
        print("\n4. 連続出力方式テスト")
        print("-" * 30)
        if sensor.start_continuous_mode(callback=continuous_data_callback):
            print("5秒間連続出力を受信します...")
            time.sleep(5.0)
            sensor.stop_continuous_mode()
        
        print("\nテスト完了")
        
    except KeyboardInterrupt:
        print("\nユーザーによって中断されました")
        
    finally:
        # 切断
        sensor.disconnect()


if __name__ == "__main__":
    main()
