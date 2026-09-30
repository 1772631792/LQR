"""MuJoCo plant -> reference C controller -> six actual motor torques."""
import time
import numpy as np
from simulation.mujoco_model import create,angles,serial_angles
from simulation.mujoco_bootstrap import mujoco
from simulation.firmware_native import Firmware


class Arena:
    def __init__(self,height=.18,terrain=True,substeps=1,min_leg_height=.15,max_leg_height=.30,stability_assist=True,
                 leg_topology='five_bar',thigh_length=.135,calf_length=.24,joint_distance=.12,wheel_distance=.52,
                 wheel_torque_limit=8.,joint_torque_limit=35.):
        if substeps not in (1,2,4):raise ValueError('substeps must be 1, 2 or 4')
        self.model,self.data=create(height,terrain,min_leg_height,max_leg_height,leg_topology,thigh_length,calf_length,joint_distance,wheel_distance,wheel_torque_limit,joint_torque_limit);self.control_dt=.001;self.substeps=substeps
        self.model.opt.timestep=self.control_dt/substeps;self.firmware=Firmware(self.control_dt,leg_topology,thigh_length,calf_length,joint_distance,wheel_torque_limit,joint_torque_limit)
        self.leg_topology=leg_topology
        self.start_angles=serial_angles(height,thigh_length,calf_length) if leg_topology=='serial' else angles(height,thigh_length,calf_length,joint_distance)
        self.distance=0.;self.height=height
        self.min_leg_height=min_leg_height;self.max_leg_height=max_leg_height;self.stability_assist=stability_assist
        self.body=self.model.body('chassis').id
        self.qidx={name:self.model.joint(name).qposadr[0] for name in ('LA','LE','RA','RE')}
        self.vidx={name:self.model.joint(name).dofadr[0] for name in self.qidx}
        self.tires=[self.model.geom(name+'tire').id for name in ('L','R')]
        self.sensor=np.zeros(18);self.command=np.zeros(6);self.command[2]=height
        self.normal=np.zeros(2);self.contact_buffer=np.zeros(6)
        self.elapsed_physics=0.;self.steps=0

    def observe(self):
        d=self.data;matrix=d.xmat[self.body].reshape(3,3)
        pitch=np.arctan2(-matrix[2,0],np.hypot(matrix[0,0],matrix[1,0]))
        roll=np.arctan2(matrix[2,1],matrix[2,2]);yaw=np.arctan2(matrix[1,0],matrix[0,0])
        # Rotational free-joint qvel is body-local; map pitch to source's sign convention.
        self.sensor[:8]=[-pitch,-d.qvel[4],roll,d.qvel[3],yaw,d.qvel[5],self.distance,
                         np.dot(d.qvel[:3],[np.cos(yaw),np.sin(yaw),0])]
        self.normal[:]=0
        for i in range(d.ncon):
            contact=d.contact[i]
            for side,gid in enumerate(self.tires):
                if gid in (contact.geom1,contact.geom2):
                    mujoco.mj_contactForce(self.model,d,i,self.contact_buffer)
                    self.normal[side]+=abs(contact.frame[2])*self.contact_buffer[0]
        for side,prefix in enumerate(('L','R')):
            second=1 if self.leg_topology=='serial' else 3
            sign=-1.0 if self.leg_topology=='serial' else 1.0
            self.sensor[8+5*side:13+5*side]=[self.start_angles[0]+sign*d.qpos[self.qidx[prefix+'A']],
                self.start_angles[second]+sign*d.qpos[self.qidx[prefix+'E']],sign*d.qvel[self.vidx[prefix+'A']],sign*d.qvel[self.vidx[prefix+'E']],self.normal[side]]
        return self.sensor

    def step(self,speed=0,yaw_rate=0,height=None,jump=False,zero=False):
        start=time.perf_counter()
        self.observe()
        requested_height=height if height is not None else self.height
        requested_height=float(np.clip(requested_height,self.min_leg_height,self.max_leg_height))
        if self.stability_assist:
            # Ease aggressive commands before a lean becomes unrecoverable.  At
            # speed, sharp steering is the dominant rollover trigger.
            tilt=max(abs(self.sensor[0]),abs(self.sensor[2]))
            tilt_scale=float(np.clip((np.radians(42)-tilt)/np.radians(24),0,1))
            steering_scale=max(.3,1-abs(self.sensor[7])/2.5)
            speed*=tilt_scale;yaw_rate*=tilt_scale*steering_scale
        self.command[:]=[speed,yaw_rate,requested_height,jump,zero,self.data.time]
        output=self.firmware.update(self.sensor,self.command)
        self.data.ctrl[:]=output[[0,2,3,1,4,5]]
        if self.leg_topology=='serial':self.data.ctrl[[1,2,4,5]]*=-1.0
        for _ in range(self.substeps):mujoco.mj_step(self.model,self.data)
        self.distance+=self.sensor[7]*self.control_dt
        self.elapsed_physics+=time.perf_counter()-start;self.steps+=1
        return self.sensor

    def snapshot(self):
        return dict(time=float(self.data.time),state=self.sensor.copy(),telemetry=self.firmware.log.copy(),
                    motors=self.firmware.output.copy(),position=self.data.xpos[self.body].copy(),
                    rtf=self.steps*self.control_dt/max(self.elapsed_physics,1e-9))


if __name__=='__main__':
    arena=Arena(terrain=False)
    for i in range(5000):
        arena.step()
        if i%500==0:print(i,arena.sensor[:8],arena.firmware.log[[0,1,8,9]],arena.firmware.output)
    print('RTF',arena.snapshot()['rtf'])
