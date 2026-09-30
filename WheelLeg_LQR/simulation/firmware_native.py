"""ctypes adapter to the actual reference source kernels."""
import ctypes as ct
import tempfile
import shutil
from pathlib import Path
import numpy as np
from simulation.chassis_sim import ROOT


class Firmware:
    def __init__(self,dt=.001,leg_topology='five_bar',thigh_length=.135,calf_length=.24,joint_distance=.12,
                 wheel_torque_limit=8.,joint_torque_limit=35.):
        if not (ROOT/'controller'/'firmware.dll').exists():raise RuntimeError('请先 Ctrl+Shift+B 编译 firmware.dll')
        self.directory=Path(tempfile.mkdtemp(prefix='wheelleg_fw_'))
        shutil.copy2(ROOT/'controller'/'firmware.dll',self.directory/'firmware.dll')
        self.dll=ct.CDLL(str(self.directory/'firmware.dll'))
        ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
        self.dll.FW_Reset.argtypes=[ct.c_double];self.dll.FW_Reset.restype=None
        self.dll.FW_Update.argtypes=[ptr,ptr,ptr,ptr];self.dll.FW_Update.restype=ct.c_int
        self.dll.FW_Gains.argtypes=[ct.c_double,ptr,ptr];self.dll.FW_Gains.restype=None
        self.dll.FW_SetGain.argtypes=[ptr];self.dll.FW_SetGain.restype=None
        self.dll.FW_Coefficients.argtypes=[ptr,ptr];self.dll.FW_Coefficients.restype=None
        self.dll.FW_SetCoefficients.argtypes=[ptr,ptr];self.dll.FW_SetCoefficients.restype=None
        self.dll.FW_SetLegTopology.argtypes=[ct.c_int];self.dll.FW_SetLegTopology.restype=None
        self.dll.FW_SetLegGeometry.argtypes=[ct.c_double,ct.c_double,ct.c_double];self.dll.FW_SetLegGeometry.restype=None
        self.dll.FW_SetTorqueLimits.argtypes=[ct.c_double,ct.c_double];self.dll.FW_SetTorqueLimits.restype=None
        self.output=np.zeros(6);self.log=np.zeros(20);self.dll.FW_Reset(dt)
        self.dll.FW_SetLegGeometry(thigh_length,calf_length,joint_distance)
        self.dll.FW_SetTorqueLimits(wheel_torque_limit,joint_torque_limit)
        self.dll.FW_SetLegTopology(1 if leg_topology=='serial' else 0)

    def close(self):
        if getattr(self,'dll',None) is not None:
            import _ctypes
            _ctypes.FreeLibrary(self.dll._handle)
            self.dll=None
            shutil.rmtree(self.directory)

    def __del__(self):
        self.close()

    def update(self,state,command):
        if self.dll.FW_Update(state,command,self.output,self.log):raise RuntimeError('腿部运动学无效，请检查腿型、角度零位和机械限位。')
        return self.output

    def gains(self,height):
        lqr,mpc=np.zeros((2,6)),np.zeros((2,6));self.dll.FW_Gains(height,lqr,mpc);return lqr,mpc
