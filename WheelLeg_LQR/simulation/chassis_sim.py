"""Reduced variable-height chassis physics. All controller design/update runs in C."""
import ctypes as ct
import json
import math
from dataclasses import dataclass, asdict, replace
from pathlib import Path
import os
import shutil
import tempfile
import threading
import time
import numpy as np
from config.robot_params import PARAMS
from simulation.model import continuous_matrices

ROOT = Path(__file__).resolve().parents[1]
RUN_LOCK = threading.Lock()  # C has one static MCU instance per loaded DLL.
STATE_NAMES = ['pos','vel','pitch','pitch_rate','height','height_rate','roll','roll_rate','yaw','yaw_rate']
OUTPUT_NAMES = ['drive_N','left_support_N','right_support_N','yaw_Nm','left_wheel_Nm','right_wheel_Nm',
                'left_hip_A_Nm','left_hip_E_Nm','right_hip_A_Nm','right_hip_E_Nm']


@dataclass
class Settings:
    duration: float = 10.0
    control_dt: float = 0.001
    physics_dt: float = 0.001
    initial_pitch: float = 10.0  # degrees, GUI convenience only
    initial_roll: float = 2.0
    initial_height: float = 0.20
    target_height: float = 0.23
    target_position: float = 0.0
    target_yaw: float = 15.0
    command_time: float = 2.0
    impulse: float = 0.5
    impulse_time: float = 4.0
    q: tuple = (20.0,2.0,300.0,5.0)
    r: float = 0.1
    length_pid: tuple = (700.0,100.0,35.0)
    roll_pid: tuple = (12.0,1.0,1.5)
    yaw_pid: tuple = (3.0,0.1,0.8)
    wheel_limit: float = 1.5
    joint_limit: float = 12.0

    def validate(self):
        for value in asdict(self).values():
            if not np.all(np.isfinite(value)):
                raise ValueError('All parameters must be finite.')
        if not 0.0001 <= self.control_dt <= 0.01 or not 0 < self.physics_dt <= self.control_dt:
            raise ValueError('Control period: 0.0001..0.01 s; integration step: >0 and <= control period.')
        for ratio in (self.control_dt/self.physics_dt, self.duration/self.control_dt):
            if ratio < 1 or not math.isclose(ratio,round(ratio),rel_tol=0,abs_tol=1e-7):
                raise ValueError('Duration must be a multiple of control period; control period a multiple of integration step.')
        if self.duration/self.physics_dt > 5_000_000 or self.duration/self.control_dt > 1_000_000:
            raise ValueError('Maximum 5 million integration steps / 1 million logged control samples per run.')
        if not 0.17 <= self.initial_height <= 0.23 or not 0.17 <= self.target_height <= 0.23:
            raise ValueError('Initial/target height must be within 0.17..0.23 m for this model.')
        if abs(self.initial_pitch)>15 or abs(self.initial_roll)>4:
            raise ValueError('This reduced model accepts initial pitch <=15 deg and roll <=4 deg.')
        for event in (self.command_time,self.impulse_time):
            if event<0 or not math.isclose(event/self.control_dt,round(event/self.control_dt),abs_tol=1e-7):
                raise ValueError('Event times must be nonnegative multiples of the control period.')
        if len(self.q)!=4 or min(self.q)<=0 or self.r<=0:
            raise ValueError('Four positive Q weights and positive R required.')
        for gains in (self.length_pid,self.roll_pid,self.yaw_pid):
            if len(gains)!=3 or min(gains)<0: raise ValueError('PID gains must be three nonnegative values.')
        if min(self.wheel_limit,self.joint_limit)<=0: raise ValueError('Torque limits must be positive.')


class NativeChassis:
    def __init__(self):
        dll_path = ROOT/'controller'/'controller.dll'
        if not dll_path.exists(): raise RuntimeError('Build controller.dll with Ctrl+Shift+B first.')
        # Load a per-run copy: user can rebuild the source DLL while the GUI remains open.
        self.temp = tempfile.TemporaryDirectory(prefix='wheelleg_mcu_')
        target = Path(self.temp.name)/'controller.dll'
        shutil.copy2(dll_path,target)
        self.directories=[]
        if os.name=='nt' and shutil.which('gcc'):
            self.directories.append(os.add_dll_directory(str(Path(shutil.which('gcc')).parent)))
        self.dll=ct.CDLL(str(target))
        ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
        self.dll.LQR_Design.argtypes=[ptr,ptr,ptr,ct.c_double,ct.c_double,ptr,ptr,ptr,ptr]
        self.dll.LQR_Design.restype=ct.c_int
        self.dll.Chassis_Init.argtypes=[ptr,ptr,ct.c_double]
        self.dll.Chassis_Init.restype=ct.c_int
        self.dll.Chassis_Update.argtypes=[ptr,ptr,ptr]
        self.dll.Chassis_Update.restype=ct.c_int

    def design(self,height,settings):
        a,b=continuous_matrices(replace(PARAMS,com_height=height+0.05))
        ad,bd,p,k=np.empty((4,4)),np.empty(4),np.empty((4,4)),np.empty(4)
        q=np.array(settings.q,dtype=np.float64)
        count=self.dll.LQR_Design(a,b,q,settings.r,settings.control_dt,ad,bd,p,k)
        if count<0: raise RuntimeError(f'C Riccati design failed: status {count}.')
        return dict(height=height,A=a.tolist(),B=b.tolist(),Ad=ad.tolist(),Bd=bd.tolist(),
                    P=p.tolist(),K=k.tolist(),iterations=count)

    def initialize(self,settings):
        designs=[self.design(h,settings) for h in (0.16,0.20,0.24)]
        gains=np.array([row['K'] for row in designs],dtype=np.float64)
        tuning=np.array([*settings.length_pid,*settings.roll_pid,*settings.yaw_pid,settings.wheel_limit,settings.joint_limit],dtype=np.float64)
        if self.dll.Chassis_Init(gains,tuning,settings.control_dt): raise RuntimeError('C chassis initialization failed.')
        return designs

    def update(self,state,reference,output):
        status=self.dll.Chassis_Update(state,reference,output)
        if status: raise RuntimeError(f'C controller failed: status {status}.')

    def close(self):
        # No callback or worker may use the handle after this explicit unload.
        if getattr(self,'dll',None) is not None:
            import _ctypes
            if os.name=='nt': _ctypes.FreeLibrary(self.dll._handle)
            else: _ctypes.dlclose(self.dll._handle)
            self.dll=None
        for handle in self.directories: handle.close()
        self.temp.cleanup()


def derivative(state,output):
    """q=(p,theta,l): variable-length pendulum from T,V; l=h+0.05.
    Roll/yaw use reduced inertial equations. Fixed contact, massless link rods.
    This is not a full spatial rigid-body/contact model.
    """
    _,velocity,theta,omega,height,hdot,roll,roll_rate,yaw,yaw_rate=state
    length=height+0.05
    m=5.0
    sine,cosine=math.sin(theta),math.cos(theta)
    mass=np.array([[6.0,m*length*cosine,m*sine],
                   [m*length*cosine,0.10+m*length*length,0],
                   [m*sine,0,m]])
    rhs=np.array([output[0]-0.1*velocity+m*length*sine*omega*omega-2*m*cosine*hdot*omega,
                  m*9.81*length*sine-2*m*length*hdot*omega,
                  output[1]+output[2]-m*9.81*cosine+m*length*omega*omega-2*hdot])
    acc=np.linalg.solve(mass,rhs)
    return np.array([velocity,acc[0],omega,acc[1],hdot,acc[2],roll_rate,
                     (0.18*(output[1]-output[2])-0.2*roll_rate)/0.12,
                     yaw_rate,(output[3]-0.15*yaw_rate)/0.20])


def simulate(settings,progress=None,cancel=None):
    settings.validate()
    start=time.perf_counter()
    with RUN_LOCK:
        native=NativeChassis()
        try:
            designs=native.initialize(settings)
            steps=round(settings.duration/settings.control_dt)
            substeps=round(settings.control_dt/settings.physics_dt)
            dt=settings.physics_dt
            states=np.zeros((steps+1,10));outputs=np.zeros((steps,10));refs=np.zeros((steps+1,4))
            state=np.array([0,0,np.deg2rad(settings.initial_pitch),0,settings.initial_height,0,
                            np.deg2rad(settings.initial_roll),0,0,0],dtype=np.float64)
            reference=np.array([0,settings.initial_height,0,0],dtype=np.float64)
            applied=np.empty(10)
            impulse_tick=round(settings.impulse_time/settings.control_dt)
            command_tick=round(settings.command_time/settings.control_dt)
            for tick in range(steps):
                if cancel is not None and cancel.is_set(): raise InterruptedError('Simulation cancelled.')
                if tick==command_tick:
                    reference[:]=[settings.target_position,settings.target_height,0,np.deg2rad(settings.target_yaw)]
                if tick==impulse_tick and settings.impulse:
                    length=state[4]+0.05;sine,cosine=np.sin(state[2]),np.cos(state[2])
                    mass=np.array([[6,5*length*cosine,5*sine],[5*length*cosine,.1+5*length*length,0],[5*sine,0,5]])
                    state[[1,3,5]]+=np.linalg.solve(mass,[settings.impulse,0,0])
                states[tick]=state;refs[tick]=reference
                native.update(state,reference,applied)
                outputs[tick]=applied
                for _ in range(substeps):
                    k1=derivative(state,applied);k2=derivative(state+dt*k1/2,applied)
                    k3=derivative(state+dt*k2/2,applied);k4=derivative(state+dt*k3,applied)
                    state+=dt*(k1+2*k2+2*k3+k4)/6
                if not np.all(np.isfinite(state)) or abs(state[2])>np.deg2rad(35) or abs(state[6])>0.15 or not 0.145<state[4]<0.265:
                    raise RuntimeError(f'Model left balance/leg domain at t={(tick+1)*settings.control_dt:.4f}s; check gains and torque limits.')
                if progress and tick%max(1,steps//100)==0: progress(tick/steps)
            states[-1]=state;refs[-1]=reference
        finally:
            native.close()
    return dict(time=np.arange(steps+1)*settings.control_dt,states=states,outputs=outputs,references=refs,
                settings=asdict(settings),designs=designs,elapsed=time.perf_counter()-start)


def save_run(result,directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    np.savez(directory/'chassis.npz',time=result['time'],states=result['states'],outputs=result['outputs'],references=result['references'],
             metadata=json.dumps({k:result[k] for k in ('settings','designs','elapsed')}))
    padded=np.vstack([result['outputs'],np.full(10,np.nan)])
    np.savetxt(directory/'chassis.csv',np.column_stack([result['time'],result['states'],padded,result['references']]),delimiter=',',
               header=','.join(['time_s',*STATE_NAMES,*OUTPUT_NAMES,'ref_position','ref_height','ref_roll','ref_yaw']),comments='')
    (directory/'design_and_settings.json').write_text(json.dumps({k:result[k] for k in ('settings','designs','elapsed')},indent=2)+'\n',encoding='utf-8')
