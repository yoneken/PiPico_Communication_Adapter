from time import sleep
from pymodbus.client import ModbusSerialClient

SERIAL_PORT = 'COM8'
BAUD_RATE = 100000
DEVICE_ID = 0x08
REGISTER_ADDRESS = 0x100
VALUE_TO_WRITE = 1
# -----------------

def main_rtu():
    print("\n--- Modbus RTU クライアントを開始します ---")
    
    client = ModbusSerialClient(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=1
    )
    
    try:
        if not client.connect():
            print(f"エラー: シリアルポート {SERIAL_PORT} への接続に失敗しました。")
            return
            
        print(f"接続成功: {SERIAL_PORT}")

        # 単一レジスタへの書き込み
        print(f"デバイスID: {DEVICE_ID} のレジスタアドレス: {hex(REGISTER_ADDRESS)} に値: {VALUE_TO_WRITE} を書き込みます。")
        response = client.write_register(
            address=REGISTER_ADDRESS,
            value=VALUE_TO_WRITE,
            slave=DEVICE_ID
        )

        sleep(1000)

        print(f"デバイスID: {DEVICE_ID} のレジスタアドレス: 0x110 に値: {VALUE_TO_WRITE} を書き込みます。")
        response = client.write_register(
            address=0x110,
            value=VALUE_TO_WRITE,
            slave=DEVICE_ID
        )

        if response.isError():
            print(f"エラー: Modbusリクエストでエラーが発生しました。応答: {response}")
        else:
            print(f"書き込み成功！ 応答: {response}")
            
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")

    finally:
        if client.is_socket_open():
            client.close()
            print("接続を閉じました。")
        print("--- Modbus RTU クライアントを終了します ---")


if __name__ == "__main__":
    main_rtu()
