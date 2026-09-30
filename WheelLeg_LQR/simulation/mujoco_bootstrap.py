"""Load a consistent app-local MSVC runtime before global MuJoCo on this Windows host."""
import os
import ctypes
from pathlib import Path
HANDLES=[]
if os.name=='nt':
    runtime=Path(__file__).resolve().parents[1]/'config'/'windows_runtime'
    if runtime.exists():
        HANDLES.append(os.add_dll_directory(str(runtime)))
        for name in ('vcruntime140.dll','vcruntime140_1.dll','msvcp140.dll','msvcp140_1.dll','msvcp140_2.dll','msvcp140_atomic_wait.dll'):
            HANDLES.append(ctypes.WinDLL(str(runtime/name)))
import mujoco
