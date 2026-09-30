"""ctypes ABI only: all online control arithmetic is in controller/lqr.c."""
import ctypes as ct
import math
import os
from pathlib import Path
import shutil
from config.robot_params import PARAMS
from simulation.design_data import ROOT, load_design


class RobotState(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in ('pos', 'vel', 'pitch', 'pitch_rate')]


class LQRController(ct.Structure):
    _fields_ = [('K', ct.c_float * 4), ('output_limit', ct.c_float)]


class CController:
    def __init__(self):
        data = load_design()
        dll_path = ROOT / 'controller' / 'controller.dll'
        if not dll_path.exists():
            raise RuntimeError('Missing controller.dll: run mingw32-make first.')
        self._dll_directories = []
        if os.name == 'nt':
            self._dll_directories.append(os.add_dll_directory(str(dll_path.parent)))
            compiler = shutil.which('gcc')
            if compiler:
                self._dll_directories.append(os.add_dll_directory(str(Path(compiler).parent)))
        try:
            self.dll = ct.CDLL(str(dll_path))
        except OSError as exc:
            raise RuntimeError('Cannot load DLL: check Python/GCC architecture and MinGW runtime path.') from exc
        dll = self.dll
        dll.LQR_Init.argtypes = [ct.POINTER(LQRController)]
        dll.LQR_Init.restype = None
        dll.LQR_Update.argtypes = [ct.POINTER(LQRController), ct.POINTER(RobotState), ct.POINTER(RobotState)]
        dll.LQR_Update.restype = ct.c_float
        for name, return_type in [('LQR_GetSampleTime', ct.c_float), ('LQR_GetDesignId', ct.c_char_p),
                                  ('LQR_GetStateSize', ct.c_uint), ('LQR_GetControllerSize', ct.c_uint)]:
            function = getattr(dll, name)
            function.argtypes = []
            function.restype = return_type
        if dll.LQR_GetStateSize() != ct.sizeof(RobotState) or dll.LQR_GetControllerSize() != ct.sizeof(LQRController):
            raise RuntimeError('C/Python ABI size mismatch.')
        if not math.isclose(dll.LQR_GetSampleTime(), PARAMS.sample_time, rel_tol=1e-6):
            raise RuntimeError('C/Python sample time mismatch: rebuild DLL.')
        if dll.LQR_GetDesignId().decode('ascii') != data['design_id']:
            raise RuntimeError('DLL uses an old gain design: run mingw32-make to rebuild.')
        self.ctrl = LQRController()
        self.state = RobotState()
        self.reference = RobotState()
        dll.LQR_Init(ct.byref(self.ctrl))

    def update(self, state, reference):
        for i, (name, _) in enumerate(RobotState._fields_):
            setattr(self.state, name, float(state[i]))
            setattr(self.reference, name, float(reference[i]))
        return float(self.dll.LQR_Update(ct.byref(self.ctrl), ct.byref(self.state), ct.byref(self.reference)))
