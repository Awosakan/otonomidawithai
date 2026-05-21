import struct
import logging

# Logger Setup
logger = logging.getLogger("IDA_Protocol")
logger.setLevel(logging.INFO)

# Protocol Constants
SYNC_BYTE_1 = 0xAA
SYNC_BYTE_2 = 0x55

# Message IDs
MSG_HEARTBEAT = 0x01
MSG_PHONE_COMMANDS = 0x02
MSG_STM32_TELEMETRY = 0x03
MSG_PID_TUNING = 0x04

# System Modes
MODE_IDLE = 0x00
MODE_AUTO = 0x01
MODE_MANUAL = 0x02
MODE_FAILSAFE = 0x03
MODE_EMERGENCY = 0x04

# CRC16 Modbus Calculation
def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

class IDAPacket:
    def __init__(self, msg_id: int, payload: bytes):
        self.msg_id = msg_id
        self.payload = payload
        self.length = len(payload)
        
    def pack(self) -> bytes:
        # Format: Sync1, Sync2, MsgID, Length, Payload, CRC16_LSB, CRC16_MSB
        header = struct.pack("<BBBB", SYNC_BYTE_1, SYNC_BYTE_2, self.msg_id, self.length)
        packet_without_crc = header + self.payload
        crc = calculate_crc16(packet_without_crc)
        crc_bytes = struct.pack("<H", crc)
        return packet_without_crc + crc_bytes

class IDAParser:
    """
    Durum makinesi (State-machine) tabanlı seri paket ayrıştırıcı.
    Kayıp baytları ve veri bozulmalarını F-35 standartlarında tolere edecek şekilde tasarlanmıştır.
    """
    STATE_WAIT_SYNC1 = 0
    STATE_WAIT_SYNC2 = 1
    STATE_WAIT_MSG_ID = 2
    STATE_WAIT_LEN = 3
    STATE_WAIT_PAYLOAD = 4
    STATE_WAIT_CRC = 5

    def __init__(self, callback):
        self.state = self.STATE_WAIT_SYNC1
        self.msg_id = 0
        self.payload_len = 0
        self.payload = bytearray()
        self.crc_buffer = bytearray()
        self.callback = callback  # Başarılı paket geldiğinde çağrılacak fonksiyon

    def feed_byte(self, b: int):
        if self.state == self.STATE_WAIT_SYNC1:
            if b == SYNC_BYTE_1:
                self.state = self.STATE_WAIT_SYNC2
                
        elif self.state == self.STATE_WAIT_SYNC2:
            if b == SYNC_BYTE_2:
                self.state = self.STATE_WAIT_MSG_ID
            else:
                self.state = self.STATE_WAIT_SYNC1 # Reset
                
        elif self.state == self.STATE_WAIT_MSG_ID:
            self.msg_id = b
            self.state = self.STATE_WAIT_LEN
            
        elif self.state == self.STATE_WAIT_LEN:
            self.payload_len = b
            self.payload.clear()
            self.crc_buffer.clear()
            if self.payload_len > 0:
                self.state = self.STATE_WAIT_PAYLOAD
            else:
                self.state = self.STATE_WAIT_CRC
                
        elif self.state == self.STATE_WAIT_PAYLOAD:
            self.payload.append(b)
            if len(self.payload) >= self.payload_len:
                self.state = self.STATE_WAIT_CRC
                
        elif self.state == self.STATE_WAIT_CRC:
            self.crc_buffer.append(b)
            if len(self.crc_buffer) >= 2:
                # Paketin tamamını oluştur ve CRC hesabı yap
                header = struct.pack("<BBBB", SYNC_BYTE_1, SYNC_BYTE_2, self.msg_id, self.payload_len)
                full_packet = header + bytes(self.payload)
                expected_crc = calculate_crc16(full_packet)
                
                received_crc = struct.unpack("<H", self.crc_buffer)[0]
                
                if received_crc == expected_crc:
                    # CRC Doğrulandı, Callback tetikle
                    try:
                        self.callback(self.msg_id, bytes(self.payload))
                    except Exception as e:
                        logger.error(f"Callback execution failed: {e}")
                else:
                    logger.warning(f"CRC Error! Expected: 0x{expected_crc:04X}, Received: 0x{received_crc:04X}")
                
                # Reset Parser
                self.state = self.STATE_WAIT_SYNC1

    def feed_data(self, data: bytes):
        for b in data:
            self.feed_byte(b)

# --- Message Payloads Pack/Unpack Helpers ---

def pack_heartbeat(status: int, mode: int) -> bytes:
    # 1 byte status, 1 byte mode
    return struct.pack("<BB", status, mode)

def unpack_heartbeat(payload: bytes) -> tuple:
    if len(payload) != 2:
        raise ValueError("Invalid heartbeat size")
    return struct.unpack("<BB", payload)

def pack_phone_commands(control_mode: int, val1: float, val2: float) -> bytes:
    """
    control_mode: 0 = Diferansiyel Itki (val1: Sol Motor %, val2: Sag Motor %)
                  1 = Açısal Dümen (val1: İtme %, val2: Dümen Açısı veya Yönelim İsteği)
    val1, val2: float değerler.
    """
    return struct.pack("<Bff", control_mode, val1, val2)

def unpack_phone_commands(payload: bytes) -> tuple:
    if len(payload) != 9:
        raise ValueError("Invalid phone commands size")
    return struct.unpack("<Bff", payload)

def pack_stm32_telemetry(lat: float, lon: float, sog: float, cog: float, gps_lock: int,
                         roll: float, pitch: float, yaw: float, roll_rate: float, 
                         pitch_rate: float, yaw_rate: float, battery: float, mode: int) -> bytes:
    """
    STM32'den telefona gönderilen kritik sensör verileri.
    """
    return struct.pack("<ddffBfffffffB", lat, lon, sog, cog, gps_lock, 
                       roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, battery, mode)

def unpack_stm32_telemetry(payload: bytes) -> dict:
    if len(payload) != 54:  # 8+8+4+4+1+4+4+4+4+4+4+4+1 = 54 bytes
        raise ValueError(f"Invalid telemetry size: {len(payload)} (expected 54)")
    data = struct.unpack("<ddffBfffffffB", payload)
    return {
        "lat": data[0],
        "lon": data[1],
        "sog": data[2],
        "cog": data[3],
        "gps_lock": data[4],
        "roll": data[5],
        "pitch": data[6],
        "yaw": data[7],
        "roll_rate": data[8],
        "pitch_rate": data[9],
        "yaw_rate": data[10],
        "battery": data[11],
        "mode": data[12]
    }

def pack_pid_tuning(kp: float, ki: float, kd: float) -> bytes:
    return struct.pack("<fff", kp, ki, kd)

def unpack_pid_tuning(payload: bytes) -> tuple:
    if len(payload) != 12:
        raise ValueError("Invalid PID tuning size")
    return struct.unpack("<fff", payload)
