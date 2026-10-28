#!/usr/bin/env python3
"""Prepare the project airframe and MAVLink-only startup for PX4 1.17.0 SITL."""
import argparse
from pathlib import Path
import subprocess

PX4_SOURCE_COMMIT = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'
AIRFRAME = '22001_gz_one_drone'
ROOT = Path(__file__).resolve().parents[1]


def prepare_sitl(px4):
    # The project uses MAVROS only; omit the DDS client from this SITL build.
    board = px4 / 'boards/px4/sitl/default.px4board'
    config = board.read_text()
    if 'CONFIG_MODULES_UXRCE_DDS_CLIENT=y' in config:
        board.write_text(config.replace('CONFIG_MODULES_UXRCE_DDS_CLIENT=y',
                                        'CONFIG_MODULES_UXRCE_DDS_CLIENT=n'))
    frames = px4 / 'ROMFS/px4fmu_common/init.d-posix/airframes'
    if not (frames / '4001_gz_x500').is_file():
        raise RuntimeError('PX4 1.17.0 Gazebo x500 airframe is missing')
    profile = (ROOT / 'config/px4_sitl' / AIRFRAME).read_bytes()
    target = frames / AIRFRAME
    if not target.is_file() or target.read_bytes() != profile:
        target.write_bytes(profile)
        target.chmod(0o755)
    cmake = frames / 'CMakeLists.txt'
    text = cmake.read_text()
    if AIRFRAME not in text:
        anchor = '\t# [22000, 22999] Reserve for custom models'
        if text.count(anchor) != 1:
            raise RuntimeError(f'unsupported airframe registration layout: {cmake}')
        cmake.write_text(text.replace(anchor, f'\t{AIRFRAME}\n\n' + anchor))

    rc = px4 / 'ROMFS/px4fmu_common/init.d-posix/rcS'
    text = rc.read_text()
    marker = '# ONE_DRONE_MAVLINK_ONLY'
    if marker not in text:
        start = '# Try to start the UXRCE-DDS Client with UDP transport if module exists'
        end = 'uxrce_dds_client start -t udp -p $uxrce_dds_port $uxrce_dds_ns'
        if text.count(start) != 1 or text.count(end) != 1:
            raise RuntimeError(f'unsupported PX4 1.17.0 startup layout: {rc}')
        a, b = text.index(start), text.index(end) + len(end)
        text = text[:a] + marker + ': FCU communication is owned by MAVROS / MAVLink.' + text[b:]
    # These assignments are outside the startup block in upstream rcS.
    # They refer to parameters absent when the client is not compiled.
    text = '\n'.join(line for line in text.split('\n') if not (
        line.strip().startswith(('param set UXRCE_DDS_', 'param set-default UXRCE_DDS_'))))
    if rc.read_text() != text:
        rc.write_text(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4', help='official PX4 v1.17.0 checkout; omit on an onboard computer')
    args = parser.parse_args()
    if args.px4:
        px4 = Path(args.px4).expanduser().resolve()
        if not px4.is_dir():
            raise RuntimeError(f'PX4 source directory not found: {px4}; clone official tag v1.17.0 first')
        commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=px4, text=True).strip()
        if commit != PX4_SOURCE_COMMIT:
            raise RuntimeError(f'requires PX4 1.17.0 at {PX4_SOURCE_COMMIT}, found {commit}')
        prepare_sitl(px4)
    header = ROOT / 'src/localization/open_vins/ov_msckf/src/core/VioManager.h'
    if not header.is_file() or 'RM27_STATIC_VIO' not in header.read_text():
        raise RuntimeError('bundled OpenVINS source/static initialization patch is missing')
    print('OpenVINS verified; PX4 1.17.0 uses the project airframe and MAVROS. Rebuild after preparation.')


if __name__ == '__main__':
    main()
