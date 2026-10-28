"""CDR metadata filtering must agree with ROS's real MAVROS serialization."""
import pytest
import struct
from mavros_msgs.msg import Mavlink
from rclpy.serialization import serialize_message
from vio_bridge.mavlink_codec import frame_identity


@pytest.mark.parametrize('frame_name', ['', 'x', 'uas1', 'mavros', 'a'*101])
@pytest.mark.parametrize('message_id', [0, 31, 132, 230, 331])
def test_raw_frame_identity_matches_ros_message(frame_name, message_id):
    msg = Mavlink(sysid=255, compid=191, msgid=message_id)
    msg.header.frame_id = frame_name
    msg.payload64 = [1, 2, 3]
    assert frame_identity(serialize_message(msg)) == (255, 191, message_id)


@pytest.mark.parametrize('data', [b'', b'\0'*16, b'\0\1\0\0'+b'\0'*8+b'\xff'*4,
                                b'\0\7'+b'\0'*100])
def test_invalid_or_truncated_metadata_is_rejected(data):
    with pytest.raises(ValueError):frame_identity(data)


def test_big_endian_cdr_metadata():
    wire = bytearray(b'\0\0\0\0'+struct.pack('>iiI', 0, 0, 6)+b'frame\0')
    wire.extend(bytes((1, 253, 0, 0, 0, 1, 42, 191)))
    wire.extend(b'\0' * ((-len(wire)) % 4))
    wire.extend(struct.pack('>I', 331))
    assert frame_identity(wire) == (42, 191, 331)
