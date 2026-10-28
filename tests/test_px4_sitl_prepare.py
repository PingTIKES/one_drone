"""Check the pinned PX4 startup patch without building or running a flight controller."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('prepare_sitl_test', ROOT/'tools/prepare_algorithm_sim.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


def test_mavlink_only_patch_is_idempotent_and_preserves_other_startup(tmp_path):
    board = tmp_path/'boards/px4/sitl/default.px4board'
    board.parent.mkdir(parents=True)
    board.write_text('CONFIG_MODULES_UXRCE_DDS_CLIENT=y\nCONFIG_MODULES_MAVLINK=y\n')
    frames = tmp_path/'ROMFS/px4fmu_common/init.d-posix/airframes'
    frames.mkdir(parents=True)
    (frames/'4001_gz_x500').touch()
    (frames/'CMakeLists.txt').write_text('px4_add_romfs_files(\n\t# [22000, 22999] Reserve for custom models\n)\n')
    rc = frames.parent/'rcS'
    rc.write_text('navigator start\n# Try to start the UXRCE-DDS Client with UDP transport if module exists\nuxrce_dds_client start -t udp -p $uxrce_dds_port $uxrce_dds_ns\ngimbal start\n')
    prepare.prepare_sitl(tmp_path)
    before = [p.read_bytes() for p in (board, rc, frames/'CMakeLists.txt', frames/prepare.AIRFRAME)]
    prepare.prepare_sitl(tmp_path)
    assert before == [p.read_bytes() for p in (board, rc, frames/'CMakeLists.txt', frames/prepare.AIRFRAME)]
    assert 'CONFIG_MODULES_UXRCE_DDS_CLIENT=n' in board.read_text()
    assert 'CONFIG_MODULES_MAVLINK=y' in board.read_text()
    assert 'uxrce_dds_client start' not in rc.read_text()
    assert 'navigator start' in rc.read_text() and 'gimbal start' in rc.read_text()
