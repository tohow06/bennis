import time, struct, serial
from cobs import cobs

PORT = "/dev/ttyACM0"
BAUD = 115200

def open_port():
    ser = serial.Serial(PORT, BAUD)
    # 讓 Arduino 不要被 DTR 立刻拉 reset（多數板子還是會重啟，但我們做完整流程）
    ser.setDTR(False)
    time.sleep(0.4)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.setDTR(True)
    # 等 bootloader 結束
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser

def write_packet(ser, payload: bytes):
    ser.write(cobs.encode(payload) + b"\x00")

def read_packet(ser, timeout=2.0) -> bytes:
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\x00":
            if not buf:
                continue
            return cobs.decode(bytes(buf))
        buf.extend(b)
    raise TimeoutError("timeout")

def send_speeds(ser, fl, fr, rl, rr):
    payload = struct.pack("<chhhh", b'm', fl, fr, rl, rr)
    write_packet(ser, payload)
    resp = read_packet(ser, timeout=2.0)
    return struct.unpack("<hhhh", resp)

def read_speeds(ser):
    payload = struct.pack("<c", b'e')
    write_packet(ser, payload)
    resp = read_packet(ser, timeout=2.0)
    return struct.unpack("<hhhh", resp)

if __name__ == "__main__":
    ser = open_port()

    # ---- 暖機/SYNC：送 0,0,0,0，丟掉回覆 ----
    try:
        _ = send_speeds(ser, 0, 0, 0, 0)
    except Exception:
        pass  # 有些板子這包仍可能被吃掉，無所謂

    # ---- 正式開始 ----
    print("reply:", send_speeds(ser, 0, 0, 0, 0))
    for i in range(50000):
        print("reply:", read_speeds(ser), i)
    print("reply:", send_speeds(ser, 0, 0, 0, 0))
    # print("reply:", send_speeds(ser, -100, -100, -100, -100))
    # print("reply:", send_speeds(ser, 200, -200, 200, -200))
