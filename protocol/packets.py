"""Packet encode/decode for the drone <-> ground link (spec: CLAUDE.md section 4).

Wire format, little-endian:
    header  magic u8 | type u8 | seq u16
    body    depends on type (see BODIES)
    trailer crc16 u16 over header + body

Packets are plain dicts, e.g. {"type": DETECTION, "seq": 7, "conf": 0.91, ...},
so they drop straight into JSON for the dashboard.
"""

import binascii
import struct
import time

MAGIC = 0xD2

DETECTION = 0x01
ACK = 0x02
HEARTBEAT = 0x03
COMMAND = 0x04

# COMMAND.cmd values
CMD_OPERATOR_ACK = 0
CMD_HOLD = 1
CMD_RESUME = 2
CMD_RTB = 3

# HEARTBEAT.mode values
MODE_PATROL = 0
MODE_HOLD = 1
MODE_RTB = 2

HEADER = struct.Struct("<BBH")
CRC = struct.Struct("<H")

# type -> (body struct, field names in wire order)
BODIES = {
    DETECTION: (struct.Struct("<IBBBii"), ("ts_ms", "cls", "conf", "count", "lat", "lon")),
    ACK: (struct.Struct("<"), ()),
    HEARTBEAT: (struct.Struct("<IBB"), ("ts_ms", "mode", "buffered")),
    COMMAND: (struct.Struct("<BH"), ("cmd", "ref_seq")),
}

# Fields sent as scaled integers: conf 0-1 -> u8, lat/lon degrees -> 1e-7 deg i32.
SCALE = {"conf": 255, "lat": 1e7, "lon": 1e7}


def now_ms():
    """Wall-clock milliseconds, wrapped to fit the u32 ts_ms field."""
    return int(time.time() * 1000) & 0xFFFFFFFF


def crc16(data):
    return binascii.crc_hqx(data, 0xFFFF)


def encode(ptype, seq, **fields):
    body_struct, names = BODIES[ptype]
    values = []
    for name in names:
        value = fields[name]
        if name in SCALE:
            value = int(round(value * SCALE[name]))
        values.append(value)
    data = HEADER.pack(MAGIC, ptype, seq & 0xFFFF) + body_struct.pack(*values)
    return data + CRC.pack(crc16(data))


def decode(data):
    """Return the packet dict, or None if the bytes aren't a valid packet of ours."""
    if len(data) < HEADER.size + CRC.size:
        return None
    magic, ptype, seq = HEADER.unpack_from(data)
    if magic != MAGIC or ptype not in BODIES:
        return None
    body_struct, names = BODIES[ptype]
    if len(data) != HEADER.size + body_struct.size + CRC.size:
        return None
    (crc,) = CRC.unpack_from(data, len(data) - CRC.size)
    if crc != crc16(data[:-CRC.size]):
        return None

    packet = {"type": ptype, "seq": seq}
    for name, value in zip(names, body_struct.unpack_from(data, HEADER.size)):
        packet[name] = value / SCALE[name] if name in SCALE else value
    return packet


def to_line(data):
    """Bytes -> one serial line for the ESP32 bridge."""
    return data.hex().upper() + "\n"


def from_line(line):
    """One serial line from the bridge -> bytes, or None for diagnostics/garbage."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    try:
        return bytes.fromhex(line)
    except ValueError:
        return None
