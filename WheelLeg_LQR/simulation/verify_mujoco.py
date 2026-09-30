"""Physical contact regression and native-render throughput; no controller in Python."""
import json
import time
import numpy as np
from PIL import Image
from simulation.mujoco_engine import Arena,mujoco
from simulation.mujoco_model import leg_joint_ranges
from simulation.chassis_sim import ROOT


def experiment(name,seconds,speed=0,y=0,height=.18):
    arena=Arena(height=height);arena.data.qpos[1]=y;mujoco.mj_forward(arena.model,arena.data)
    peak=0.;air=0;contacts=set();closure=0
    try:
        for i in range(round(seconds/.001)):
            arena.step(speed if i>1000 else 0,height=height)
            peak=max(peak,abs(arena.sensor[0]));air+=int(all(arena.firmware.log[[7,15]]))
            for side in ('L','R'):
                closure=max(closure,float(np.linalg.norm(arena.data.site(side+'Atip').xpos-arena.data.site(side+'Etip').xpos)))
            for c in arena.data.contact:
                for g in (c.geom1,c.geom2):contacts.add(mujoco.mj_id2name(arena.model,mujoco.mjtObj.mjOBJ_GEOM,g))
            assert np.isfinite(arena.data.qpos).all()
            assert max(abs(arena.firmware.output[:2]))<=8 and max(abs(arena.firmware.output[2:]))<=35
        assert peak<.25,(name,peak)
        assert closure<.006,(name,closure)
        if speed:assert arena.data.qpos[0]>3,(name,arena.data.qpos[:3])
        if name=='ramp':assert air>0 and {'launch_ramp','ramp_platform'}<=contacts and arena.data.qpos[2]<.4
        if name=='step':assert 'single_step' in contacts
        result=dict(name=name,peak_pitch_deg=float(np.degrees(peak)),airborne_ticks=air,max_closure_error_m=closure,
                    final_position=arena.data.qpos[:3].tolist(),physics_rtf=arena.snapshot()['rtf'],contacts=sorted(str(v) for v in contacts))
        print(result);return result
    finally:arena.firmware.close()


def main():
    output=ROOT/'outputs'/'mujoco';output.mkdir(parents=True,exist_ok=True)
    # Every articulated leg joint is mechanically limited, and changing the
    # public length range changes the generated angular stops.
    wide=leg_joint_ranges(.18,.15,.30);narrow=leg_joint_ranges(.18,.17,.27)
    assert all(wide[key][0]<narrow[key][0]<narrow[key][1]<wide[key][1] for key in wide)
    limited=Arena(terrain=False,min_leg_height=.17,max_leg_height=.27)
    try:
        for name in ('LA','LAknee','LE','LEknee','RA','RAknee','RE','REknee'):
            assert limited.model.jnt_limited[limited.model.joint(name).id]
        for _ in range(1500):limited.step(height=.33)
        assert np.isfinite(limited.data.qpos).all()
    finally:limited.firmware.close()
    # Serial-leg mode is a real hip/knee open chain and uses the same virtual
    # leg controller interface and six motor outputs as the generated C core.
    serial=Arena(terrain=False,leg_topology='serial')
    try:
        for _ in range(3000):serial.step(height=.18)
        assert np.isfinite(serial.data.qpos).all()
        assert abs(serial.sensor[0])<.10 and serial.data.qpos[2]>.18
        assert max(abs(serial.firmware.output[:2]))<=8 and max(abs(serial.firmware.output[2:]))<=35
    finally:serial.firmware.close()
    results=[experiment('stand',3),experiment('ramp',12,.7,height=.26),experiment('step',10,.7,y=2.3,height=.26)]
    a=Arena();renderer=mujoco.Renderer(a.model,height=450,width=800)
    try:
        camera=mujoco.MjvCamera();camera.azimuth=45;camera.elevation=-30;camera.distance=3
        start=time.perf_counter()
        for frame in range(120):
            for _ in range(33):a.step()
            camera.lookat[:]=a.data.xpos[a.body]
            renderer.update_scene(a.data,camera);image=renderer.render()
        elapsed=time.perf_counter()-start
        metrics=dict(sim_seconds=a.data.time,wall_seconds=elapsed,rendered_rtf=a.data.time/elapsed,fps=120/elapsed)
        Image.fromarray(image).save(output/'render.png')
        print('Physics + C + 800x450 render:',metrics)
        # Throughput is reported, not asserted against machine-dependent frame rate.
        (output/'verification.json').write_text(json.dumps(dict(experiments=results,render=metrics),indent=2),encoding='utf-8')
    finally:renderer.close();a.firmware.close()


if __name__=='__main__':main()
