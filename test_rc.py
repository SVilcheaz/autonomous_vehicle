import serial

def unpack_channels(payload):
    # payload = 22 bytes of packed 11-bit channel data
    bits = int.from_bytes(payload, byteorder='little')
    channels = []
    for i in range(16):
        channels.append(bits & 0x7FF)
        bits >>= 11
    return channels

ser = serial.Serial('/dev/serial0', 420000, timeout=1)
buf = bytearray()

while True:
    buf += ser.read(64)
    while len(buf) >= 2 and buf[0] == 0xC8:
        length = buf[1]
        if len(buf) < length + 2:
            break
        frame = buf[:length + 2]
        frame_type = frame[2]
        if frame_type == 0x16:
            payload = frame[3:3+22]
            channels = unpack_channels(payload)
            print(channels)
        buf = buf[length + 2:]
