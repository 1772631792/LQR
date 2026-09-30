"""Exercise actual Tk input, worker, recording, replay and matrix clipboard."""
import ctypes
import time
import tkinter as tk
from types import SimpleNamespace
import numpy as np
from PIL import ImageGrab
from simulation.host_app import HostApp,matrix_initializer
from simulation.chassis_sim import ROOT


def main():
    root=tk.Tk();app=HostApp(root);panel=app.arena_panel
    def pump(seconds):
        deadline=time.perf_counter()+seconds
        while time.perf_counter()<deadline:root.update();time.sleep(.005)
    def finish(timeout=30):
        deadline=time.perf_counter()+timeout
        while panel.worker.is_alive() and time.perf_counter()<deadline:root.update();time.sleep(.005)
        assert not panel.worker.is_alive();pump(.2)
    try:
        app.tabs.select(app.pages['MuJoCo 实景']);root.update()
        assert panel.min_leg_height.get()=='0.15' and panel.max_leg_height.get()=='0.30'
        assert panel.stability_assist.get()
        panel.scene.set('平地');panel.start_live();pump(1)
        panel.key_down(SimpleNamespace(keysym='w',state=0));pump(2)
        panel.key_up(SimpleNamespace(keysym='w'));panel.key_down(SimpleNamespace(keysym='c',state=0));pump(1)
        panel.release();assert not panel.keys
        output=ROOT/'outputs'/'mujoco';output.mkdir(parents=True,exist_ok=True)
        ImageGrab.grab(window=ctypes.windll.user32.GetParent(root.winfo_id())).save(output/'upper_computer.png')
        panel.halt();finish();assert panel.run is not None
        assert panel.run['records'][-1,7]>.3
        print('Live GUI:',panel.run['records'][-1,0],'sim seconds; status:',panel.note.get())
        panel.export();assert (output/'record.csv').exists()
        panel.start_run('replay');pump(1);assert panel.worker.is_alive();panel.halt();finish()
        panel.scene.set('一级台阶');panel.duration.set('10');panel.start_run('batch');finish()
        assert panel.run['qpos'][-1,0]>3
        panel.design();deadline=time.perf_counter()+60
        while panel.design_busy and time.perf_counter()<deadline:root.update();time.sleep(.01)
        assert panel.report is not None
        panel.copy_matrix();assert root.clipboard_get()==matrix_initializer(panel.value())
        panel.use_generated.set(True);panel.duration.set('2');panel.start_run('batch');finish()
        assert np.max(np.abs(panel.run['records'][:,1]))<.2
        assert panel.run['initial_height']==.26
        panel.start_run('replay');pump(2.6);assert panel.worker.is_alive();panel.halt();finish()
        panel.substeps.set('4');panel.duration.set('.3');panel.start_run('batch');finish()
        assert abs(panel.run['records'][-1,0]-.3)<.002
        print('PASS: live input, release, high leg, record/export, replay, batch step, C design, matrix clipboard, regenerated gains run')
    finally:app.close()


if __name__=='__main__':main()
