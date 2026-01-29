import serial
import struct
import threading
import numpy as np


BAUD_RATE = 230400             # Typical for LD19 variants
PACKET_LEN = 47                # Fixed packet length
HEADER_BYTE = 0x54
MEAS_PER_PACKET = 12
LIDAR_MAX_RANGE_M = 12.0       # LD19 advertised max range, tune if needed
DEG_PER_SCAN = 360             # 1-degree resolution
DISTANCE_NO_OBSTACLE_MM = int(LIDAR_MAX_RANGE_M * 1000)

# -----------------------------
# Serial reader producing 360° scans
# -----------------------------
class LD19Reader(threading.Thread):
    # -----------------------------
    # LD19 parsing (based on known packet format)
    # -----------------------------
    _MESSAGE_FORMAT = "<xBHH" + "HB" * MEAS_PER_PACKET + "HHB"
    # Fields:
    # length (B), speed (H), start_angle (H), [12x distance(H), confidence(B)], stop_angle(H), timestamp(H), crc(B)
    # distances in mm; angles in centi-degrees

    def __init__(self, port, baud=BAUD_RATE):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.2)
        self.buffer = bytearray()
        self.latest_scan = None
        self.running = True
        # A 360-slot distance array (mm), initialized to "no obstacle"
        self.scan_accum = np.full(DEG_PER_SCAN, DISTANCE_NO_OBSTACLE_MM, dtype=np.int32)
        self.filled = np.zeros(DEG_PER_SCAN, dtype=np.bool_)

    def run(self):
        while self.running:
            data = self.ser.read(256)
            if not data:
                continue
            self.buffer.extend(data)
            # Try to extract packets
            self._consume_packets()

    def stop(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


    def _parse_ld19_packet(self, packet_bytes):
        # struct.unpack result
        # We do not validate CRC here for simplicity; add CRC if required
        length, speed, start_angle_cd, *rest = struct.unpack(self._MESSAGE_FORMAT, packet_bytes)
        # Unpack tail
        stop_angle_cd, timestamp, crc = rest[-3], rest[-2], rest[-1]
        meas_raw = rest[:-3]  # 12 pairs (distance, confidence)

        start_angle = start_angle_cd / 100.0
        stop_angle = stop_angle_cd / 100.0
        # unwrap stop angle
        if stop_angle < start_angle:
            stop_angle += 360.0

        # Build arrays of angles and distances
        step = (stop_angle - start_angle) / (MEAS_PER_PACKET - 1)
        distances_mm = []
        angles_deg = []
        for i in range(MEAS_PER_PACKET):
            d_mm = meas_raw[2*i]        # distance in mm
            conf = meas_raw[2*i + 1]    # confidence (unused here)
            angle = start_angle + i * step
            # Normalize to [0,360)
            angle = angle % 360.0
            distances_mm.append(d_mm)
            angles_deg.append(angle)
        return angles_deg, distances_mm, speed
    
    def _consume_packets(self):
        # Find header positions and extract fixed-size packets
        i = 0
        while i + PACKET_LEN <= len(self.buffer):
            if self.buffer[i] == HEADER_BYTE and self.buffer[i+1] == 0x2C:
                packet = self.buffer[i:i+PACKET_LEN]
                try:
                    angles_deg, distances_mm, speed = self._parse_ld19_packet(packet)
                    # Insert measurements into accumulation
                    for angle, dist in zip(angles_deg, distances_mm):
                        idx = int(round(angle)) % DEG_PER_SCAN
                        # Keep the smallest distance observed for that degree
                        if dist > 0 and dist < self.scan_accum[idx]:
                            self.scan_accum[idx] = dist
                        self.filled[idx] = True
                    # If a large portion is filled, publish a scan
                    if self.filled.sum() > 350:  # threshold for "complete" scan
                        self.latest_scan = np.copy(self.scan_accum)
                        # Reset accumulators for the next revolution
                        self.scan_accum[:] = DISTANCE_NO_OBSTACLE_MM
                        self.filled[:] = False
                except struct.error:
                    # If unpack fails, skip this header
                    pass
                i += PACKET_LEN
            else:
                i += 1
        # Trim buffer
        if i > 0:
            del self.buffer[:i]
