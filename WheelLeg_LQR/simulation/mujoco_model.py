"""Actual free-body, articulated five-bar mechanism with wheel contacts and terrain."""
import math
import numpy as np
from simulation.mujoco_bootstrap import mujoco


def angles(height=.18,thigh_length=.135,calf_length=.24,joint_distance=.12):
    # Firmware x-forward/y-down convention, outward knee branch.
    r=thigh_length;R=calf_length;a=joint_distance/2;d=math.hypot(a,height);along=(r*r-R*R+d*d)/(2*d);v=math.sqrt(r*r-along*along)
    bx=-a+along*a/d-v*height/d;bz=-along*height/d-v*a/d
    phi1=math.atan2(-bz,bx+a);phi4=math.atan2(-bz,-bx-a)
    phi2=math.atan2(height+bz,-bx);phi3=math.atan2(height+bz,bx)
    return phi1,phi2,phi3,phi4


def quat(angle):return f'{math.cos(angle/2):.10g} 0 {math.sin(angle/2):.10g} 0'


def _wrapped_delta(value, reference):
    return (value-reference+math.pi)%(2*math.pi)-math.pi


def leg_joint_ranges(height=.18, min_height=.15, max_height=.30,
                     thigh_length=.135,calf_length=.24,joint_distance=.12):
    """Return joint-coordinate limits for the normal vertical five-bar branch.

    MuJoCo joint coordinates are offsets from the XML pose.  The public limit
    parameters are effective leg lengths, which are much easier to tune than
    eight unrelated joint angles.
    """
    if not all(math.isfinite(v) for v in (height,min_height,max_height)):
        raise ValueError('leg heights must be finite')
    if not .14<=min_height<max_height<=.33:
        raise ValueError('leg limits require 0.14 <= minimum < maximum <= 0.33 m')
    if not min_height<=height<=max_height:
        raise ValueError('initial leg height must be inside the configured limits')
    base=angles(height,thigh_length,calf_length,joint_distance)
    samples=[angles(float(v),thigh_length,calf_length,joint_distance) for v in np.linspace(min_height,max_height,101)]
    result={}
    # A/E are active hips.  The knee coordinate is lower-link angle minus its
    # parent upper-link angle, not the lower link's world angle.
    coordinates={
        'A':lambda p:p[0], 'Aknee':lambda p:p[1]-p[0],
        'E':lambda p:p[3], 'Eknee':lambda p:p[2]-p[3],
    }
    base_coordinates={key:fn(base) for key,fn in coordinates.items()}
    for key,fn in coordinates.items():
        values=[_wrapped_delta(fn(p),base_coordinates[key]) for p in samples]
        # A small allowance prevents chatter when the requested height is at an endpoint.
        result[key]=(min(values)-math.radians(1),max(values)+math.radians(1))
    return result


def five_bar_xml(height=.18,terrain=True,min_leg_height=.15,max_leg_height=.30,
                 thigh_length=.135,calf_length=.24,joint_distance=.12,wheel_distance=.52,
                 wheel_torque_limit=8.,joint_torque_limit=35.):
    p1,p2,p3,p4=angles(height,thigh_length,calf_length,joint_distance)
    limits=leg_joint_ranges(height,min_leg_height,max_leg_height,thigh_length,calf_length,joint_distance)
    legs=[];eq=[];act=[]
    for side,y,color in [('L',wheel_distance/2,'0.06 0.62 0.78 1'),('R',-wheel_distance/2,'0.95 0.48 0.14 1')]:
        for key,hip,upper,lower in [('A',-joint_distance/2,p1,p2),('E',joint_distance/2,p4,p3)]:
            hip_range=' '.join(f'{v:.10g}' for v in limits[key])
            knee_range=' '.join(f'{v:.10g}' for v in limits[key+'knee'])
            wheel=''
            if key=='A':
                wheel=f'''<body name="{side}wheel" pos=".24 0 0" quat="{quat(-lower)}">
                  <joint name="{side}wheel_j" axis="0 1 0" damping=".002" armature=".0002"/>
                  <inertial pos="0 0 0" mass=".865" diaginertia=".0017 .002433 .0017"/>
                  <geom name="{side}tire" type="cylinder" size=".075 .025" quat=".70710678 .70710678 0 0" friction="1.2 .015 .001" rgba=".10 .13 .18 1" contype="1" conaffinity="1"/>
                  <geom type="cylinder" size=".05 .0255" quat=".70710678 .70710678 0 0" rgba=".66 .74 .82 1" contype="0" conaffinity="0"/>
                  <geom type="capsule" fromto="-.055 .027 0 .055 .027 0" size=".006" rgba="{color}" contype="0" conaffinity="0"/>
                </body>'''
            legs.append(f'''<body name="{side}{key}upper" pos="{hip} {y} 0" quat="{quat(upper)}">
              <joint name="{side}{key}" axis="0 1 0" damping=".03" armature=".001" limited="true" range="{hip_range}" margin=".01"/>
              <geom type="capsule" fromto="0 0 0 {thigh_length:.10g} 0 0" size=".012" mass=".1437" rgba="{color}"/>
              <body name="{side}{key}lower" pos="{thigh_length:.10g} 0 0" quat="{quat(lower-upper)}">
                <joint name="{side}{key}knee" axis="0 1 0" damping=".008" limited="true" range="{knee_range}" margin=".01"/>
                <geom type="capsule" fromto="0 0 0 {calf_length:.10g} 0 0" size=".009" mass=".1827" rgba="{color}"/>
                <site name="{side}{key}tip" pos="{calf_length:.10g} 0 0" size=".003"/>{wheel}
              </body>
            </body>''')
        eq.append(f'<connect site1="{side}Atip" site2="{side}Etip" solref=".004 1" solimp=".99 .999 .0001"/>')
        for name in (f'{side}wheel_j',f'{side}A',f'{side}E'):
            limit=wheel_torque_limit if name.endswith('wheel_j') else joint_torque_limit
            act.append(f'<motor name="{name}_motor" joint="{name}" gear="1" ctrllimited="true" ctrlrange="{-limit:.10g} {limit:.10g}"/>')
    terrain_xml=''
    if terrain:
        # Ramp mesh has a level top platform and a sharp drop; separate 4 cm step lane.
        terrain_xml='''<geom name="launch_ramp" type="mesh" mesh="ramp" pos="0 0 0" rgba=".25 .38 .49 1" contype="1" conaffinity="1"/>
        <geom name="ramp_platform" type="box" pos="2.1 0 .16" size=".5 .7 .16" rgba=".30 .45 .57 1" contype="1" conaffinity="1"/>
        <geom name="single_step" type="box" pos=".8 2.3 .02" size=".8 .7 .02" rgba=".68 .48 .20 1" contype="1" conaffinity="1"/>
        <geom name="step_marker" type="box" pos=".0 2.3 .041" size=".01 .7 .001" rgba="1 .8 .18 1" contype="0" conaffinity="0"/>
        <geom name="ramp_marker" type="box" pos="2.58 0 .322" size=".015 .7 .002" rgba="1 .7 .1 1" contype="0" conaffinity="0"/>'''
    return f'''<mujoco model="RoboMaster five-bar training arena">
      <compiler angle="radian"/>
      <option timestep=".001" integrator="implicitfast" solver="Newton" iterations="30" tolerance="1e-9" gravity="0 0 -9.81"/>
      <visual><global offwidth="1280" offheight="720"/><quality shadowsize="2048"/><map znear=".015" zfar="60"/></visual>
      <default><joint limited="false"/><geom contype="0" conaffinity="0" condim="3" solref=".006 1"/></default>
      <asset>
        <texture type="skybox" builtin="gradient" rgb1=".25 .40 .58" rgb2=".85 .91 .96" width="512" height="3072"/>
        <texture name="groundtex" type="2d" builtin="checker" rgb1=".25 .31 .34" rgb2=".31 .37 .40" width="512" height="512"/>
        <material name="groundmat" texture="groundtex" texrepeat="24 24" reflectance=".02"/>
        <mesh name="ramp" vertex="0 -.7 0  0 .7 0  1.6 -.7 0  1.6 .7 0  1.6 -.7 .32  1.6 .7 .32" face="0 2 1 1 2 3 0 4 2 1 3 5 0 1 5 0 5 4 2 4 5 2 5 3"/>
      </asset>
      <worldbody>
        <light pos="-3 -4 7" dir=".2 .3 -1" castshadow="true"/>
        <light pos="1 3 4" dir="-.2 -.3 -1" diffuse=".4 .4 .4" specular=".1 .1 .1" castshadow="false"/>
        <geom name="ground" type="plane" size="12 10 .1" material="groundmat" contype="1" conaffinity="1"/>
        <body name="terrain"><geom type="box" pos="0 0 -.08" size="12 10 .05" rgba=".2 .25 .3 1"/>{terrain_xml}</body>
        <body name="chassis" pos="-2 0 {height+.075}">
          <freejoint name="base"/>
          <inertial pos="0 0 -.016" mass="7.645" diaginertia=".25 .16 .30"/>
          <geom name="shell" type="box" pos="0 0 .02" size=".17 .205 .06" rgba=".22 .30 .42 1" contype="1" conaffinity="1"/>
          <geom type="box" pos=".175 0 .04" size=".008 .13 .025" rgba=".12 .75 .9 1"/>
          <geom type="box" pos="-.02 0 .095" size=".10 .14 .025" rgba=".6 .7 .8 1"/>
          <site name="imu" size=".01"/>
          {''.join(legs)}
        </body>
      </worldbody>
      <equality>{''.join(eq)}</equality>
      <actuator>{''.join(act)}</actuator>
    </mujoco>'''


def serial_angles(height=.18,upper=.135,lower=.24):
    """Hip and knee angles for a vertical two-link serial leg.

    Angles use the portable controller convention: zero points down, knee angle
    is relative to the upper link and the forward-bent branch is positive.
    """
    cosine=(height*height-upper*upper-lower*lower)/(2*upper*lower)
    if not -1.0<=cosine<=1.0:raise ValueError('串联腿长度超出两连杆可达范围')
    knee=math.acos(cosine)
    hip=-math.atan2(lower*math.sin(knee),upper+lower*math.cos(knee))
    return hip,knee


def serial_joint_ranges(height=.18,min_height=.15,max_height=.30,upper=.135,lower=.24):
    if not abs(upper-lower)<min_height<max_height<upper+lower:
        raise ValueError('串联腿限位必须位于 |大腿-小腿| 与 大腿+小腿 之间')
    base=np.array(serial_angles(height,upper,lower))
    samples=np.array([serial_angles(float(v),upper,lower) for v in np.linspace(min_height,max_height,101)])-base
    allowance=math.radians(1)
    return {'hip':(samples[:,0].min()-allowance,samples[:,0].max()+allowance),
            'knee':(samples[:,1].min()-allowance,samples[:,1].max()+allowance)}


def serial_xml(height=.18,terrain=True,min_leg_height=.15,max_leg_height=.30,
               thigh_length=.135,calf_length=.24,wheel_distance=.52,wheel_torque_limit=8.,joint_torque_limit=35.):
    hip,knee=serial_angles(height,thigh_length,calf_length)
    limits=serial_joint_ranges(height,min_leg_height,max_leg_height,thigh_length,calf_length)
    legs=[];act=[]
    for side,y,color in [('L',wheel_distance/2,'0.06 0.62 0.78 1'),('R',-wheel_distance/2,'0.95 0.48 0.14 1')]:
        legs.append(f'''<body name="{side}Aupper" pos="0 {y:.10g} 0" quat="{quat(math.pi/2-hip)}">
          <joint name="{side}A" axis="0 1 0" damping=".03" armature=".001" limited="true" range="{limits['hip'][0]:.10g} {limits['hip'][1]:.10g}" margin=".01"/>
          <geom type="capsule" fromto="0 0 0 {thigh_length:.10g} 0 0" size=".014" mass=".22" rgba="{color}"/>
          <body name="{side}Elower" pos="{thigh_length:.10g} 0 0" quat="{quat(-knee)}">
            <joint name="{side}E" axis="0 1 0" damping=".018" armature=".001" limited="true" range="{limits['knee'][0]:.10g} {limits['knee'][1]:.10g}" margin=".01"/>
            <geom type="capsule" fromto="0 0 0 {calf_length:.10g} 0 0" size=".011" mass=".28" rgba="{color}"/>
            <body name="{side}wheel" pos="{calf_length:.10g} 0 0" quat="{quat(-(math.pi/2-hip-knee))}">
              <joint name="{side}wheel_j" axis="0 1 0" damping=".002" armature=".0002"/>
              <inertial pos="0 0 0" mass=".865" diaginertia=".0017 .002433 .0017"/>
              <geom name="{side}tire" type="cylinder" size=".075 .025" quat=".70710678 .70710678 0 0" friction="1.2 .015 .001" rgba=".10 .13 .18 1" contype="1" conaffinity="1"/>
              <geom type="cylinder" size=".05 .0255" quat=".70710678 .70710678 0 0" rgba=".66 .74 .82 1" contype="0" conaffinity="0"/>
            </body>
          </body>
        </body>''')
        for name in (f'{side}wheel_j',f'{side}A',f'{side}E'):
            limit=wheel_torque_limit if name.endswith('wheel_j') else joint_torque_limit
            act.append(f'<motor name="{name}_motor" joint="{name}" gear="1" ctrllimited="true" ctrlrange="{-limit:.10g} {limit:.10g}"/>')
    terrain_xml=''
    if terrain:
        terrain_xml='''<geom name="launch_ramp" type="mesh" mesh="ramp" pos="0 0 0" rgba=".25 .38 .49 1" contype="1" conaffinity="1"/>
        <geom name="ramp_platform" type="box" pos="2.1 0 .16" size=".5 .7 .16" rgba=".30 .45 .57 1" contype="1" conaffinity="1"/>
        <geom name="single_step" type="box" pos=".8 2.3 .02" size=".8 .7 .02" rgba=".68 .48 .20 1" contype="1" conaffinity="1"/>'''
    return f'''<mujoco model="RoboMaster serial-leg training arena">
      <compiler angle="radian"/><option timestep=".001" integrator="implicitfast" solver="Newton" iterations="30" tolerance="1e-9" gravity="0 0 -9.81"/>
      <visual><global offwidth="1280" offheight="720"/><quality shadowsize="2048"/><map znear=".015" zfar="60"/></visual>
      <default><joint limited="false"/><geom contype="0" conaffinity="0" condim="3" solref=".006 1"/></default>
      <asset><texture type="skybox" builtin="gradient" rgb1=".25 .40 .58" rgb2=".85 .91 .96" width="512" height="3072"/>
      <texture name="groundtex" type="2d" builtin="checker" rgb1=".25 .31 .34" rgb2=".31 .37 .40" width="512" height="512"/><material name="groundmat" texture="groundtex" texrepeat="24 24" reflectance=".02"/>
      <mesh name="ramp" vertex="0 -.7 0  0 .7 0  1.6 -.7 0  1.6 .7 0  1.6 -.7 .32  1.6 .7 .32" face="0 2 1 1 2 3 0 4 2 1 3 5 0 1 5 0 5 4 2 4 5 2 5 3"/></asset>
      <worldbody><light pos="-3 -4 7" dir=".2 .3 -1" castshadow="true"/><geom name="ground" type="plane" size="12 10 .1" material="groundmat" contype="1" conaffinity="1"/>
      <body name="terrain"><geom type="box" pos="0 0 -.08" size="12 10 .05" rgba=".2 .25 .3 1"/>{terrain_xml}</body>
      <body name="chassis" pos="-2 0 {height+.075}"><freejoint name="base"/><inertial pos="0 0 -.016" mass="7.645" diaginertia=".25 .16 .30"/>
      <geom name="shell" type="box" pos="0 0 .02" size=".17 .205 .06" rgba=".22 .30 .42 1" contype="1" conaffinity="1"/><geom type="box" pos="-.02 0 .095" size=".10 .14 .025" rgba=".6 .7 .8 1"/><site name="imu" size=".01"/>{''.join(legs)}</body></worldbody>
      <actuator>{''.join(act)}</actuator></mujoco>'''


def xml(height=.18,terrain=True,min_leg_height=.15,max_leg_height=.30,leg_topology='five_bar',
        thigh_length=.135,calf_length=.24,joint_distance=.12,wheel_distance=.52,wheel_torque_limit=8.,joint_torque_limit=35.):
    if leg_topology=='serial':
        return serial_xml(height,terrain,min_leg_height,max_leg_height,thigh_length,calf_length,wheel_distance,wheel_torque_limit,joint_torque_limit)
    return five_bar_xml(height,terrain,min_leg_height,max_leg_height,thigh_length,calf_length,joint_distance,wheel_distance,wheel_torque_limit,joint_torque_limit)


def create(height=.18,terrain=True,min_leg_height=.15,max_leg_height=.30,leg_topology='five_bar',
           thigh_length=.135,calf_length=.24,joint_distance=.12,wheel_distance=.52,wheel_torque_limit=8.,joint_torque_limit=35.):
    model=mujoco.MjModel.from_xml_string(xml(height,terrain,min_leg_height,max_leg_height,leg_topology,
                                             thigh_length,calf_length,joint_distance,wheel_distance,wheel_torque_limit,joint_torque_limit))
    # All terrain geoms collide; decorative parts and rods remain noncolliding.
    for name in ('launch_ramp','ramp_platform','single_step'):
        index=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name)
        if index>=0:model.geom_contype[index]=model.geom_conaffinity[index]=1
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    return model,data
