#!/usr/bin/env python3
"""Verify bundled OpenVINS and apply optional MicoAir PX4 1.14.3 SITL patches."""
import argparse
from pathlib import Path
import subprocess

PX4_SOURCE_COMMIT = '08310a5e8ac64d02edb41523460e7dc267298deb'


def write_checked(path,old,new,marker,equivalent=None):
    text=path.read_text(encoding='utf-8')
    if marker in text:return
    if equivalent is not None and text.count(equivalent)==1:
        old=equivalent
    if text.count(old)!=1:raise RuntimeError(f'unsupported source version/anchor: {path}')
    backup=path.with_name(path.name+'.rm27-backup')
    if not backup.exists():backup.write_text(text,encoding='utf-8')
    path.write_text(text.replace(old,new),encoding='utf-8',newline='\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4',help='MicoAir PX4 1.14.3 source; omit on an onboard computer')
    a=p.parse_args()
    if a.px4:
        px4=Path(a.px4).resolve()
        if not px4.is_dir():
            raise RuntimeError(f'PX4 source directory not found: {px4}; clone the MicoAir 1.14.3 source first')
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=px4,text=True).strip()
        if commit!=PX4_SOURCE_COMMIT:
            raise RuntimeError(f'requires MicoAir PX4 1.14.3 source at {PX4_SOURCE_COMMIT}, found {commit}')
        source=px4/'src/modules/uxrce_dds_client/uxrce_dds_client.cpp'
        old='\t// latest round trip time (RTT)'
        new='''#if defined(__PX4_POSIX)
    // RM27_SIM_CLOCK: /clock is the clock on BOTH sides in algorithm SITL.
    if (getenv("RM27_SIM_CLOCK") && strcmp(getenv("RM27_SIM_CLOCK"), "1") == 0) {
        session->time_offset = 0;
        return;
    }
#endif
'''+old
        write_checked(source,old,new,'// RM27_SIM_CLOCK:')
        rc=px4/'ROMFS/px4fmu_common/init.d-posix/px4-rc.params'
        text=rc.read_text(encoding='utf-8')
        marker='# RM27_VISION_ONLY'
        if marker not in text:
            backup=rc.with_name(rc.name+'.rm27-backup')
            if not backup.exists():backup.write_text(text,encoding='utf-8')
            text+='''
# RM27_VISION_ONLY: applies only when explicitly started by start_algorithm_sim.sh.
if [ "$RM27_SIM_CLOCK" = "1" ]; then
    param set EKF2_GPS_CTRL 0
    param set EKF2_EV_CTRL 15
    param set EKF2_HGT_REF 3
    param set EKF2_MAG_TYPE 5
    param set EKF2_EV_POS_X 0
    param set EKF2_EV_POS_Y 0
    param set EKF2_EV_POS_Z 0
    param set MPC_XY_VEL_MAX 0.5
    param set MPC_XY_CRUISE 0.5
    param set COM_OF_LOSS_T 0.5
    param set COM_OBL_RC_ACT 4
    param set COM_POSCTL_NAVL 1
fi
'''
            rc.write_text(text,encoding='utf-8',newline='\n')
    header=Path(__file__).resolve().parents[1]/'src/localization/open_vins/ov_msckf/src/core/VioManager.h'
    if not header.is_file() or 'RM27_STATIC_VIO' not in header.read_text(encoding='utf-8'):
        raise RuntimeError('bundled OpenVINS source/static initialization patch is missing')
    print('Bundled OpenVINS verified. Rebuild this workspace; rebuild PX4 if patched.')



if __name__=='__main__':main()
