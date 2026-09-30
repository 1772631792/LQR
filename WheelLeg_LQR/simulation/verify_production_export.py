"""Verify a package can stand alone and matches the reference controller."""
import ctypes as ct
import shutil
import tempfile
from pathlib import Path
import numpy as np
from simulation.production_export import export_package
from simulation.firmware_native import Firmware
from simulation.mujoco_model import angles


class Config(ct.Structure):
    _fields_=[(name,ct.c_float) for name in ('sample_time_s','thigh_length_m','calf_length_m','joint_distance_m','wheel_distance_m','body_mass_kg','max_acceleration_m_s2','airborne_force_n','wheel_torque_limit_nm','joint_torque_limit_nm','min_leg_length_m','max_leg_length_m')]+[('lqr_coefficients',(ct.c_float*4)*12),('mpc_coefficients',(ct.c_float*4)*12)]
class PidPort(ct.Structure):
    _fields_=[('instance',ct.c_void_p),('calculate',ct.c_void_p),('reset',ct.c_void_p)]
class Context(ct.Structure):
    _fields_=[('config',Config),('leg',(ct.c_float*24)*2),('speed_integral',ct.c_float*2),('target_velocity',ct.c_float),('target_distance',ct.c_float),('target_yaw',ct.c_float),('burst_start',ct.c_float),('pid',PidPort*8),('jump_locked',ct.c_uint32),('burst',ct.c_uint32),('initialized',ct.c_uint32)]
class Input(ct.Structure):
    _fields_=[(name,ct.c_float) for name in ('pitch_rad','pitch_rate_rad_s','roll_rad','roll_rate_rad_s','yaw_rad','yaw_rate_rad_s','distance_m','velocity_m_s')]+[('joint_angle_rad',ct.c_float*4),('joint_rate_rad_s',ct.c_float*4),('normal_force_n',ct.c_float*2),('time_s',ct.c_float)]
class Command(ct.Structure):
    _fields_=[('velocity_m_s',ct.c_float),('yaw_rate_rad_s',ct.c_float),('leg_length_m',ct.c_float),('mode_flags',ct.c_uint32)]
class Output(ct.Structure):
    _fields_=[('wheel_torque_nm',ct.c_float*2),('joint_torque_nm',ct.c_float*4),('status_flags',ct.c_uint32)]


def main():
    directory=Path(tempfile.mkdtemp(prefix='wlc_export_verify_'))
    try:
        manifest=export_package(directory/'package','verification')
        package=directory/'package';compiler=manifest['compiler'];dll=package/'build'/'wlc_verify.dll'
        serial_manifest=export_package(directory/'package_serial','verification-serial',parameters={'leg_topology':1})
        serial_package=directory/'package_serial'
        assert manifest['leg_variant']=='five_bar' and serial_manifest['leg_variant']=='serial_leg'
        five_kin=(package/'src'/'wlc_kinematics.h').read_text(encoding='utf-8')
        serial_kin=(serial_package/'src'/'wlc_kinematics.h').read_text(encoding='utf-8')
        assert 'five-bar' in five_kin and 'serial' not in five_kin and 'q12' not in five_kin
        assert 'serial-leg' in serial_kin and 'five-bar' not in serial_kin and 's32' not in serial_kin
        assert 'leg_topology' not in (package/'src'/'wlc_internal.c').read_text(encoding='utf-8')
        assert 'leg_topology' not in (serial_package/'src'/'wlc_internal.c').read_text(encoding='utf-8')
        guide=(package/'INTEGRATION_GUIDE_CN.md').read_text(encoding='utf-8')
        assert 'wheel_leg_step()' in guide and 'WheelLeg_UserAfterControl' in guide and 'Motor_SetTorqueNm' in guide and 'wheel_leg_bind_pid' in guide
        user_source=package/'src'/'wheel_leg_user.c';user_source.write_text(user_source.read_text(encoding='utf-8')+'\n/* PRESERVE_USER_PID */\n',encoding='utf-8')
        export_package(package,'verification');assert 'PRESERVE_USER_PID' in user_source.read_text(encoding='utf-8')
        import subprocess
        subprocess.run([compiler,'-std=c11','-O2','-shared','-I'+str(package/'include'),'-I'+str(package/'generated'),str(package/'src'/'wlc_internal.c'),'-o',str(dll)],check=True)
        portable=ct.CDLL(str(dll));config=Config();context=Context();portable.Wlc_DefaultConfig(ct.byref(config));assert portable.Wlc_Init(ct.byref(context),ct.byref(config))==0
        firmware=Firmware();phi=angles(.18);maximum=0.
        try:
            for tick in range(2000):
                time=tick*.001
                state=np.array([.02*np.sin(time),.01*np.cos(time),.005*np.sin(2*time),0,.03*np.sin(time),.02,0,.1,phi[0],phi[3],0,0,80,phi[0],phi[3],0,0,80],dtype=np.float64)
                command=np.array([.4,.1,.18,0,0,time],dtype=np.float64);reference=firmware.update(state,command).copy()
                item=Input();item.pitch_rad,item.pitch_rate_rad_s,item.roll_rad,item.roll_rate_rad_s,item.yaw_rad,item.yaw_rate_rad_s,item.distance_m,item.velocity_m_s=map(float,state[:8])
                item.joint_angle_rad[:]=[phi[0],phi[3],phi[0],phi[3]];item.joint_rate_rad_s[:]=[0]*4;item.normal_force_n[:]=[80,80];item.time_s=time
                output=Output();assert portable.Wlc_Step(ct.byref(context),ct.byref(item),ct.byref(Command(.4,.1,.18,0)),ct.byref(output),None)==0
                actual=np.array([*output.wheel_torque_nm,*output.joint_torque_nm]);maximum=max(maximum,float(np.max(abs(actual-reference))))
            assert maximum<1e-4,maximum
        finally:firmware.close()
        print('PASS: self-contained compile, dependency audit, self-test, reference equivalence; max error',maximum)
    finally:shutil.rmtree(directory,ignore_errors=True)


if __name__=='__main__':main()
