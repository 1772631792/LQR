"""Exercise real Tk widgets and worker lifecycle without requiring user interaction."""
import time
import json
import sys
import shutil
import tempfile
from pathlib import Path
from types import SimpleNamespace
import tkinter as tk
import numpy as np
from simulation.host_app import HostApp,matrix_values,matrix_initializer
from simulation.parameter_guide import PARAMETERS,signal_value
from simulation.chassis_sim import ROOT


def main():
    root=tk.Tk();root.withdraw();app=HostApp(root);production_directory=Path(tempfile.mkdtemp(prefix='wlc_ui_verify_'))
    try:
        app.fields['duration'].set('5')
        app.compute()
        deadline=time.perf_counter()+30
        while app.result is None and time.perf_counter()<deadline:
            root.update();time.sleep(.01)
        assert app.result is not None,'Background calculation did not finish'
        for tab in app.pages.values():
            app.tabs.select(tab);root.update();app.render();root.update()
        assert '生产代码导出' in app.pages
        app.show_robot_settings();root.update()
        dialogs=[child for child in root.winfo_children() if isinstance(child,tk.Toplevel)]
        assert dialogs and '机器人设置' in dialogs[-1].title();dialogs[-1].destroy();root.update()
        app.apply_robot_settings(dict(app.robot_config,leg_topology='serial'))
        assert app.production_panel.robot_config['leg_topology']=='serial'
        assert '串联腿' in app.production_panel.topology.get()
        assert '串联腿' in app.nav_buttons['二维机构'].cget('text')
        assert '髋/膝' in app.guide.parameter('left_hip')[0]
        app.tabs.select(app.pages['二维机构']);app.render();root.update()
        assert all('serial leg' in ax.get_title() for ax in app.axes2)
        app.tabs.select(app.pages['三维底盘']);app.render();root.update()
        assert 'Serial-leg chassis' in app.axes3.get_title()
        app.tabs.select(app.pages['参数导览']);root.update()
        guide_text=[app.guide.canvas.itemcget(item,'text') for item in app.guide.canvas.find_all() if app.guide.canvas.type(item)=='text']
        assert any('串联腿参数导览' in text for text in guide_text)
        app.apply_robot_settings(dict(app.robot_config,leg_topology='five_bar'))
        assert '五连杆' in app.nav_buttons['二维机构'].cget('text')
        production=app.production_panel;production.output.set(str(production_directory/'wheelleg_control'))
        production.start();deadline=time.perf_counter()+30
        while production.worker.is_alive() and time.perf_counter()<deadline:root.update();time.sleep(.01)
        while production.state.get().startswith('正在') and time.perf_counter()<deadline:root.update();time.sleep(.01)
        assert not production.worker.is_alive()
        assert (production_directory/'wheelleg_control'/'manifest.json').exists()
        assert production.state.get()=='导出完成并通过独立编译/自测。'
        production.output.set(str(production_directory/'selected_control'));production.start_both();deadline=time.perf_counter()+30
        while production.worker.is_alive() and time.perf_counter()<deadline:root.update();time.sleep(.01)
        while production.state.get().startswith('正在') and time.perf_counter()<deadline:root.update();time.sleep(.01)
        assert (production_directory/'five_bar_control'/'manifest.json').exists()
        assert (production_directory/'serial_leg_control'/'manifest.json').exists()
        assert production.state.get()=='两套独立代码均已生成并通过自测。'
        app.seek('2.5');assert app.current==2.5
        app.toggle_play();deadline=time.perf_counter()+.2
        while time.perf_counter()<deadline:root.update();time.sleep(.01)
        assert app.current>2.5
        app.toggle_play();app.export_to(ROOT/'outputs'/'host_verified')
        # A delayed frame may cross more than one loop; preserve the remainder.
        app.speed.set('1');app.loop.set(True);app.playing=True
        app.anchor_time=4.8;app.anchor_wall=100.;app.advance_playback(110.5)
        assert abs(app.current-.3)<1e-10 and app.playing
        app.loop.set(False);app.anchor_time=4.8;app.anchor_wall=100.;app.advance_playback(100.5)
        assert app.current==5 and not app.playing
        # Clipboard contains only completed-run matrix data, not edited settings/logs.
        for key in ('A','B','Ad','Bd','Q','R','P','K'):
            app.matrix_key.set(key);app.refresh_matrix();app.copy_matrix()
            assert root.clipboard_get()==matrix_initializer(matrix_values(app.result,1,key))
        app.fields['r'].set('99');app.copy_all_matrices()
        copied=json.loads(root.clipboard_get())
        assert set(copied)=={'A','B','Ad','Bd','Q','R','P','K'} and copied['R']==[[.1]]
        app.fields['r'].set('.1')
        app.matrix_node.current(0);app.matrix_key.set('K');app.copy_matrix()
        assert root.clipboard_get()==matrix_initializer(matrix_values(app.result,0,'K'))
        # Exercise diagram hit testing, hover, click-to-pin, and curve navigation.
        app.tabs.select(app.pages['参数导览']);root.update();app.seek('2.5');root.update()
        for key in PARAMETERS:
            x,y=app.guide.label_centers[key];event=SimpleNamespace(x=x,y=y)
            assert app.guide.key_at(event)==key, f'Callout hit test failed: {key}'
            app.guide.motion(event);assert app.guide.hovered==key
            app.guide.click(event);assert app.guide.selected==key and app.guide.pinned
            app.guide.jump();tab=PARAMETERS[key][2]
            assert app.tabs.tab(app.tabs.select(),'text')==tab
            ax=app.figures[tab].axes[PARAMETERS[key][3]]
            assert ax.get_facecolor()[2]<.95
            app.tabs.select(app.pages['参数导览']);root.update()
        state=app.result['states'][2500]
        assert signal_value('left_height',state,None)==f'{state[4]+.18*state[6]:.4f} m'
        app.guide.leave();app.guide.unpin();app.rewind();assert app.current==0 and not app.playing
        if '--screenshots' in sys.argv:
            # Capture only this test application's HWND, never the full desktop.
            import ctypes
            from PIL import ImageGrab
            root.deiconify();root.update()
            hwnd=ctypes.windll.user32.GetParent(root.winfo_id())
            app.guide.selected='pitch';app.guide.pinned=True
            app.tabs.select(app.pages['参数导览']);app.seek('0');root.update()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'parameter_guide_ui.png')
            app.apply_robot_settings(dict(app.robot_config,leg_topology='serial'));app.tabs.select(app.pages['二维机构']);app.render();root.update()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'serial_leg_2d_ui.png')
            app.tabs.select(app.pages['参数导览']);app.render();root.update()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'serial_parameter_guide_ui.png')
            app.apply_robot_settings(dict(app.robot_config,leg_topology='five_bar'))
            app.tabs.select(app.pages['C算法与增益']);app.refresh_matrix();root.update()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'matrix_ui.png')
            app.tabs.select(app.pages['仿真设置']);root.update()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'settings_ui.png')
            app.tabs.select(app.pages['MuJoCo 实景']);root.update()
            assert not app.arena_panel.advanced.winfo_manager()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'mujoco_ui.png')
            app.arena_panel.toggle_advanced();root.update()
            assert app.arena_panel.advanced.winfo_manager()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'mujoco_advanced_ui.png')
            app.arena_panel.toggle_advanced();root.update()
            app.tabs.select(app.pages['生产代码导出']);root.update()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'production_ui.png')
            root.withdraw()
        previous=app.result
        app.fields['duration'].set('20');app.fields['physics_dt'].set('0.0001')
        app.compute();app.cancel.set()
        deadline=time.perf_counter()+10
        while app.worker.is_alive() and time.perf_counter()<deadline:root.update();time.sleep(.01)
        root.update();assert not app.worker.is_alive();assert app.result is previous
        print('PASS: Tk startup, worker, nine sidebar pages, seek/play/loop/end, matrix-only clipboard,')
        print('      11 diagram hover/pin/curve links, data/code export and cancellation.')
    finally:
        app.close();shutil.rmtree(production_directory,ignore_errors=True)


if __name__=='__main__':main()
