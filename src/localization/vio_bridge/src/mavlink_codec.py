"""MAVROS router ROS-endpoint frames, preserving the MAVLink 2 wire format."""
import struct
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v20 import common


def frame_identity(serialized):
    """Peek MAVROS CDR1 metadata before deserializing unrelated telemetry.

    Header contains an 8-byte stamp followed by a CDR string. The eight
    byte-sized MAVLink fields follow it; msgid is aligned to four bytes.
    """
    if len(serialized) < 17 or serialized[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('invalid MAVROS CDR1 header')
    endian = '<' if serialized[1] == 1 else '>'
    length = struct.unpack_from(endian+'I', serialized, 12)[0]
    start = 16+length
    msgid_offset = (start+8+3) & ~3
    if length < 1 or msgid_offset+4 > len(serialized):
        raise ValueError('truncated MAVROS CDR1 metadata')
    return (serialized[start+6], serialized[start+7],
            struct.unpack_from(endian+'I', serialized, msgid_offset)[0])


def decode_frame(frame):
    if frame.framing_status != Mavlink.FRAMING_OK:
        raise ValueError('invalid MAVLink framing')
    payload = b''.join(struct.pack('<Q', int(v)) for v in frame.payload64)[:frame.len]
    if len(payload) != frame.len:
        raise ValueError('truncated MAVLink payload')
    if frame.magic == 253:
        header = bytes((253, frame.len, frame.incompat_flags, frame.compat_flags,
                        frame.seq, frame.sysid, frame.compid)) + int(frame.msgid).to_bytes(3, 'little')
    elif frame.magic == 254 and frame.msgid < 256:
        header = bytes((254, frame.len, frame.seq, frame.sysid, frame.compid, frame.msgid))
    else:
        raise ValueError('invalid MAVLink header')
    wire = header + payload + struct.pack('<H', frame.checksum) + bytes(frame.signature)
    try:
        return common.MAVLink(None).decode(bytearray(wire))
    except Exception as error:
        raise ValueError('invalid MAVLink packet: ' + str(error)) from error


def encode_frame(message, stamp, source_system=1, source_component=191, sequence=0):
    mav = common.MAVLink(None, srcSystem=source_system, srcComponent=source_component)
    mav.seq = sequence
    wire = bytes(message.pack(mav))
    out = Mavlink()
    out.header.stamp = stamp
    out.framing_status = Mavlink.FRAMING_OK
    out.magic, out.len = wire[0], wire[1]
    out.incompat_flags, out.compat_flags, out.seq = wire[2], wire[3], wire[4]
    out.sysid, out.compid = wire[5], wire[6]
    out.msgid = int.from_bytes(wire[7:10], 'little')
    payload = wire[10:10+out.len]
    padded = payload + b'\0' * ((-len(payload)) % 8)
    out.payload64 = list(struct.unpack('<' + 'Q'*(len(padded)//8), padded))
    out.checksum = int.from_bytes(wire[10+out.len:12+out.len], 'little')
    out.signature = list(wire[12+out.len:])
    return out
