"""Tk owns widgets; one worker owns MuJoCo, its GL context and the C controller."""
import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
from PIL import Image, ImageTk
from simulation.chassis_sim import ROOT


class MuJoCoPanel(ttk.Frame):
    def __init__(self,parent,robot_config=None):
        super().__init__(parent)
        self.robot_config=robot_config or {}
        self.worker=None;self.closed=False;self.stop=threading.Event()
        self.commands=queue.SimpleQueue();self.events=queue.SimpleQueue();self.frames=queue.Queue(1)
        self.keys=set();self.active=False;self.high=False;self.fast=False
        self.camera=[45.,-24.,3.2];self.overview=False;self.drag=None;self.run=None;self.report=None;self.design_busy=False
        self.loop=tk.BooleanVar(value=True);self.use_generated=tk.BooleanVar(value=False)
        self.stability_assist=tk.BooleanVar(value=True)
        self.scene=tk.StringVar(value='RM地图');self.duration=tk.StringVar(value='12')
        self.substeps=tk.StringVar(value='1');self.resolution=tk.StringVar(value='960x540')
        self.min_leg_height=tk.StringVar(value=f"{self.robot_config.get('min_leg_length_m',.15):.2f}");self.max_leg_height=tk.StringVar(value=f"{self.robot_config.get('max_leg_length_m',.30):.2f}")
        self.low_leg_height=tk.StringVar(value='.18');self.high_leg_height=tk.StringVar(value='.26')
        self.normal_speed=tk.StringVar(value='2.0');self.fast_speed=tk.StringVar(value='3.5')
        self.yaw_speed=tk.StringVar(value='1.0');self.batch_speed=tk.StringVar(value='1.0')
        self.note=tk.StringVar(value='源代码控制器 · 1 kHz · 六电机 · 刚体接触动力学。点击进入操控模式。')
        heading=ttk.Frame(self);heading.pack(fill='x',pady=(2,8))
        ttk.Label(heading,text='MuJoCo 实景',style='Title.TLabel').pack(side='left')
        ttk.Label(heading,text='刚体接触 · C 控制器 · 1 kHz',style='Secondary.TLabel').pack(side='left',padx=12)
        self.toolbar=ttk.Frame(self,style='Surface.TFrame',padding=(10,8));self.toolbar.pack(fill='x',pady=(0,8))
        ttk.Button(self.toolbar,text='▶  进入操控',style='Primary.TButton',command=self.start_live).pack(side='left')
        ttk.Button(self.toolbar,text='停止',style='Danger.TButton',command=self.halt).pack(side='left',padx=(7,12))
        ttk.Label(self.toolbar,text='场景').pack(side='left')
        ttk.Combobox(self.toolbar,textvariable=self.scene,values=['RM地图','起伏路','中央高地','平地'],state='readonly',width=9).pack(side='left',padx=(5,10))
        ttk.Button(self.toolbar,text='预计算',style='Compact.TButton',command=lambda:self.start_run('batch')).pack(side='left')
        ttk.Button(self.toolbar,text='回放',style='Compact.TButton',command=lambda:self.start_run('replay')).pack(side='left',padx=5)
        ttk.Checkbutton(self.toolbar,text='循环',variable=self.loop,command=self.send).pack(side='left')
        ttk.Button(self.toolbar,text='全图 / 跟随',style='Compact.TButton',command=self.toggle_camera).pack(side='left',padx=5)
        ttk.Button(self.toolbar,text='导出',style='Compact.TButton',command=self.export).pack(side='right')
        self.advanced_button=ttk.Button(self.toolbar,text='调整参数  ▾',style='Compact.TButton',command=self.toggle_advanced)
        self.advanced_button.pack(side='right',padx=5)
        self.advanced_visible=False
        self.advanced=ttk.Frame(self,style='Surface.TFrame',padding=(12,9))
        self.advanced.columnconfigure(9,weight=1)
        ttk.Label(self.advanced,text='预计算').grid(row=0,column=0,sticky='w')
        ttk.Entry(self.advanced,textvariable=self.duration,width=5).grid(row=0,column=1,padx=(5,2))
        ttk.Label(self.advanced,text='s').grid(row=0,column=2,sticky='w')
        ttk.Label(self.advanced,text='物理子步').grid(row=0,column=3,padx=(18,0))
        ttk.Combobox(self.advanced,textvariable=self.substeps,values=['1','2','4'],state='readonly',width=3).grid(row=0,column=4,padx=5)
        ttk.Label(self.advanced,text='画面').grid(row=0,column=5,padx=(15,0))
        ttk.Combobox(self.advanced,textvariable=self.resolution,values=['800x450','960x540','1280x720'],state='readonly',width=9).grid(row=0,column=6,padx=5)
        self.topology_text=tk.StringVar();ttk.Label(self.advanced,textvariable=self.topology_text,style='Secondary.TLabel').grid(row=0,column=7,padx=(16,0),sticky='w')
        ttk.Label(self.advanced,text='腿长限位').grid(row=1,column=0,sticky='w',pady=(8,0))
        ttk.Entry(self.advanced,textvariable=self.min_leg_height,width=6).grid(row=1,column=1,padx=(5,2),pady=(8,0))
        ttk.Label(self.advanced,text='—').grid(row=1,column=2,pady=(8,0))
        ttk.Entry(self.advanced,textvariable=self.max_leg_height,width=6).grid(row=1,column=3,padx=5,pady=(8,0),sticky='w')
        ttk.Label(self.advanced,text='m').grid(row=1,column=4,sticky='w',pady=(8,0))
        ttk.Checkbutton(self.advanced,text='防翻车辅助',variable=self.stability_assist).grid(row=1,column=5,columnspan=2,padx=(15,0),pady=(8,0),sticky='w')
        ttk.Label(self.advanced,text='W/S 前后 · A/D 转向 · C 高低腿 · G 快慢 · Shift 越阶 · Ctrl+C 零力 · 右键旋转 · 滚轮缩放',style='Secondary.TLabel').grid(row=2,column=0,columnspan=10,sticky='w',pady=(8,0))
        self.apply_robot_config(self.robot_config)
        book=ttk.Notebook(self);book.pack(fill='both',expand=True)
        view=ttk.Frame(book);book.add(view,text='第三人称地图')
        self.screen=tk.Label(view,bg='#17232e',fg='#c8dce8',text='RMUC 2026 理论训练场\n\n启动后显示 MuJoCo 物理渲染画面',takefocus=True,width=1,height=1)
        self.screen.pack(fill='both',expand=True)
        self.screen.bind('<Button-1>',lambda e:self.screen.focus_set())
        self.screen.bind('<KeyPress>',self.key_down);self.screen.bind('<KeyRelease>',self.key_up)
        self.screen.bind('<FocusOut>',lambda e:self.release())
        self.screen.bind('<ButtonPress-3>',self.drag_start);self.screen.bind('<B3-Motion>',self.drag_camera)
        self.screen.bind('<MouseWheel>',self.zoom)
        controls=ttk.Frame(book,padding=16);book.add(controls,text='操控参数')
        self.make_drive_settings(controls)
        self.plots=ttk.Frame(book);book.add(self.plots,text='本次物理记录曲线')
        ttk.Label(self.plots,text='停止操控或预计算完成后显示。与前面简化模型的曲线独立，全部来自 MuJoCo + 参考 C 代码。').pack(anchor='w')
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        self.figure=Figure(figsize=(10,5),dpi=90);self.plot_canvas=FigureCanvasTkAgg(self.figure,master=self.plots)
        self.plot_canvas.get_tk_widget().pack(fill='both',expand=True)
        design=ttk.Frame(book,padding=8);book.add(design,text='替代 MATLAB / 六状态矩阵')
        ttk.Label(design,text='每腿状态 [θ, θ̇, p, v, pitch, pitch_rate]，输出 [轮力矩, 虚拟髋力矩]。C 迭代求解 DARE，Python 离线建模和验证。').pack(anchor='w')
        edit=ttk.Frame(design);edit.pack(fill='x',pady=8)
        self.q=tk.StringVar(value='480 240 200 600 2000 50');self.r=tk.StringVar(value='2.5 0.25')
        ttk.Label(edit,text='Q 对角').pack(side='left');ttk.Entry(edit,textvariable=self.q,width=32).pack(side='left')
        ttk.Label(edit,text='R 对角').pack(side='left',padx=(10,0));ttk.Entry(edit,textvariable=self.r,width=12).pack(side='left')
        self.design_button=ttk.Button(edit,text='C 重算 30 个腿长节点',command=self.design);self.design_button.pack(side='left',padx=8)
        ttk.Checkbutton(design,text='下一次启动使用重算的增益（未勾选时使用原始固件参数）',variable=self.use_generated).pack(anchor='w')
        select=ttk.Frame(design);select.pack(fill='x',pady=8)
        self.node=tk.StringVar(value='0.18');self.matrix=tk.StringVar(value='K')
        self.design_note=tk.StringVar(value='完成后显示所选腿长的闭环稳定性与 C 迭代次数。')
        a=ttk.Combobox(select,textvariable=self.node,values=[f'{v/100:.2f}' for v in range(10,40)],width=7,state='readonly');a.pack(side='left');a.bind('<<ComboboxSelected>>',lambda e:self.preview())
        b=ttk.Combobox(select,textvariable=self.matrix,values=['A','B','Ad','Bd','Q','R','Qd','Rd','Nd','P','K','eigenvalues','lqr_coefficients','mpc_coefficients'],width=18,state='readonly');b.pack(side='left');b.bind('<<ComboboxSelected>>',lambda e:self.preview())
        ttk.Button(select,text='只复制当前矩阵',command=self.copy_matrix).pack(side='left',padx=8)
        ttk.Label(design,textvariable=self.design_note).pack(anchor='w',pady=4)
        self.text=tk.Text(design,font=('Consolas',10),wrap='none');self.text.pack(fill='both',expand=True)
        self.text.insert('end','点击重算生成 design.json 与 regenerated_gains.h。\n使用连续代价精确离散化（含交叉项 N），与参考 GenerateGains.m 对齐。');self.text.configure(state='disabled')
        ttk.Label(self,textvariable=self.note,style='Secondary.TLabel').pack(anchor='w',pady=5)
        self.after(30,self.poll)

    def make_drive_settings(self,page):
        ttk.Label(page,text='MuJoCo 操控参数',style='PageTitle.TLabel').pack(anchor='w')
        ttk.Label(page,text='这些参数在下一次进入操控或预计算时生效；修改机器人设置中的机械尺寸不会被这里覆盖。',
                  style='Secondary.TLabel').pack(anchor='w',pady=(3,14))
        grid=ttk.Frame(page,style='Surface.TFrame',padding=14);grid.pack(fill='x')
        fields=[
            ('最短有效腿长','min_leg_height','m','机械关节限位下界'),
            ('最长有效腿长','max_leg_height','m','机械关节限位上界'),
            ('低腿目标','low_leg_height','m','进入操控时的默认腿长'),
            ('高腿目标','high_leg_height','m','按 C 切换后的腿长'),
            ('普通速度','normal_speed','m/s','W/S，默认比旧版更快'),
            ('高速速度','fast_speed','m/s','按 G 切换高速档'),
            ('转向角速度','yaw_speed','rad/s','A/D 的航向速度'),
            ('预计算速度','batch_speed','m/s','预计算直行使用'),
        ]
        for row,(label,attribute,unit,help_text) in enumerate(fields):
            ttk.Label(grid,text=label).grid(row=row,column=0,sticky='w',padx=(0,12),pady=6)
            ttk.Entry(grid,textvariable=getattr(self,attribute),width=12).grid(row=row,column=1,sticky='w',pady=6)
            ttk.Label(grid,text=unit,style='Secondary.TLabel').grid(row=row,column=2,sticky='w',padx=(7,18),pady=6)
            ttk.Label(grid,text=help_text,style='Secondary.TLabel').grid(row=row,column=3,sticky='w',pady=6)
        ttk.Button(grid,text='恢复操控默认值',style='Compact.TButton',command=self.reset_drive_settings).grid(
            row=len(fields),column=0,columnspan=2,sticky='w',pady=(12,0))
        ttk.Label(page,text='快捷键：W/S 前后 · A/D 转向 · C 高低腿 · G 普通/高速 · Shift 越障动作 · R 重置',
                  style='Secondary.TLabel').pack(anchor='w',pady=14)

    def reset_drive_settings(self):
        defaults={'low_leg_height':'.18','high_leg_height':'.26','normal_speed':'2.0',
                  'fast_speed':'3.5','yaw_speed':'1.0','batch_speed':'1.0'}
        self.min_leg_height.set(f"{self.robot_config.get('min_leg_length_m',.15):.2f}")
        self.max_leg_height.set(f"{self.robot_config.get('max_leg_length_m',.30):.2f}")
        for name,value in defaults.items():getattr(self,name).set(value)
        self.note.set('操控参数已恢复默认值；下次进入操控时生效。')

    def send(self):
        self.commands.put(dict(keys=set(self.keys),high=self.high,fast=self.fast,camera=list(self.camera),loop=self.loop.get(),overview=self.overview))

    def toggle_advanced(self):
        self.advanced_visible=not self.advanced_visible
        if self.advanced_visible:
            self.advanced.pack(fill='x',after=self.toolbar,pady=(0,8))
            self.advanced_button.configure(text='收起参数  ▴')
        else:
            self.advanced.pack_forget()
            self.advanced_button.configure(text='调整参数  ▾')

    def toggle_camera(self):self.overview=not self.overview;self.send()

    def release(self):
        self.keys.clear();self.send()

    def key_down(self,event):
        if not self.active:return
        key=event.keysym.lower()
        if key not in self.keys:
            if key=='c' and not event.state&4:self.high=not self.high
            if key=='g':self.fast=not self.fast
            if key=='r':self.start_live();return 'break'
            if key=='escape':self.halt();return 'break'
        self.keys.add(key)
        if event.state&4:self.keys.add('control_l')
        self.send();return 'break'

    def key_up(self,event):
        key=event.keysym.lower();self.keys.discard(key)
        if key.startswith('control'):self.keys.difference_update({'control_l','control_r'})
        self.send();return 'break'

    def drag_start(self,event):self.drag=(event.x,event.y);self.screen.focus_set()
    def drag_camera(self,event):
        if self.drag:
            self.camera[0]+=(event.x-self.drag[0])*.4
            self.camera[1]=float(np.clip(self.camera[1]+(event.y-self.drag[1])*.3,-80,-5))
            self.drag=(event.x,event.y);self.send()
    def zoom(self,event):
        self.camera[2]=float(np.clip(self.camera[2]*(.9 if event.delta>0 else 1.1),.6,12));self.send()

    def halt(self):self.active=False;self.release();self.stop.set()
    def apply_robot_config(self,config):
        self.robot_config=config
        self.min_leg_height.set(f"{config.get('min_leg_length_m',.15):.2f}");self.max_leg_height.set(f"{config.get('max_leg_length_m',.30):.2f}")
        if hasattr(self,'topology_text'):
            name='串联腿' if config.get('leg_topology')=='serial' else '五连杆'
            self.topology_text.set(f'{name} · 下次启动生效')
            if hasattr(self,'screen') and not str(self.screen.cget('image')):
                self.screen.configure(text=f'MuJoCo {name}试验场\n\n启动后显示物理渲染画面')
    def start_live(self):self.start_run('live')
    def start_run(self,mode):
        if self.worker and self.worker.is_alive():
            self.halt();self.after(80,lambda:self.start_run(mode) if not self.closed else None);return
        if mode=='replay' and self.run is None:self.note.set('请先完成一次操控或预计算。');return
        try:
            duration=float(self.duration.get())
            if not np.isfinite(duration) or not .1<=duration<=120:raise ValueError('时间范围为 0.1–120 秒')
            min_leg_height=float(self.min_leg_height.get());max_leg_height=float(self.max_leg_height.get())
            if not .09<=min_leg_height<max_leg_height<=.39:raise ValueError('腿部限位需满足 0.09 <= 最短 < 最长 <= 0.39 m')
            low_leg_height=float(self.low_leg_height.get());high_leg_height=float(self.high_leg_height.get())
            normal_speed=float(self.normal_speed.get());fast_speed=float(self.fast_speed.get())
            yaw_speed=float(self.yaw_speed.get());batch_speed=float(self.batch_speed.get())
            if not min_leg_height<=low_leg_height<high_leg_height<=max_leg_height:raise ValueError('低腿/高腿目标必须位于机械腿长限位内，且低腿 < 高腿')
            if not .1<=normal_speed<fast_speed<=8:raise ValueError('速度需满足 0.1 <= 普通速度 < 高速速度 <= 8 m/s')
            if not .1<=yaw_speed<=4:raise ValueError('转向角速度范围为 0.1–4 rad/s')
            if not .1<=batch_speed<=4:raise ValueError('预计算速度范围为 0.1–4 m/s')
            initial_height=self.run['initial_height'] if mode=='replay' and self.run is not None else (high_leg_height if mode=='batch' else low_leg_height)
            if not min_leg_height<=initial_height<=max_leg_height:raise ValueError(f'初始腿长 {initial_height:.2f} m 必须位于机械限位内')
            if self.use_generated.get() and self.report is None:raise ValueError('请先重算参数')
        except ValueError as exc:messagebox.showerror('设置错误',str(exc));return
        self.stop=threading.Event();self.active=mode=='live';self.keys.clear();self.high=False
        # Drain old input, so a held key can never survive a reset.
        while not self.commands.empty():self.commands.get()
        options=dict(mode=mode,duration=duration,scene=self.scene.get(),report=self.report if self.use_generated.get() else None,run=self.run,
                     substeps=int(self.substeps.get()),resolution=tuple(map(int,self.resolution.get().split('x'))),
                     min_leg_height=min_leg_height,max_leg_height=max_leg_height,stability_assist=self.stability_assist.get(),
                     low_leg_height=low_leg_height,high_leg_height=high_leg_height,normal_speed=normal_speed,
                     fast_speed=fast_speed,yaw_speed=yaw_speed,batch_speed=batch_speed,
                     leg_topology=self.robot_config.get('leg_topology','five_bar'),thigh_length=self.robot_config.get('thigh_length_m',.135),
                     calf_length=self.robot_config.get('calf_length_m',.24),joint_distance=self.robot_config.get('joint_distance_m',.12),
                     wheel_distance=self.robot_config.get('wheel_distance_m',.52),wheel_torque_limit=self.robot_config.get('wheel_torque_limit_nm',8.),
                     joint_torque_limit=self.robot_config.get('joint_torque_limit_nm',35.))
        self.send();self.worker=threading.Thread(target=self.simulate,args=(options,self.stop),daemon=True);self.worker.start()
        self.screen.focus_set();self.note.set('正在初始化 MuJoCo / OpenGL…')

    def simulate(self,options,stop):
        arena=renderer=None;records=[];positions=[]
        try:
            from simulation.mujoco_engine import Arena,mujoco
            from simulation.rmuc2026_map import spawn_pose
            initial_height=options['run']['initial_height'] if options['mode']=='replay' else (options['high_leg_height'] if options['mode']=='batch' else options['low_leg_height'])
            arena=Arena(height=initial_height,terrain=True,substeps=options['substeps'],min_leg_height=options['min_leg_height'],
                        max_leg_height=options['max_leg_height'],stability_assist=options['stability_assist'],leg_topology=options['leg_topology'],
                        thigh_length=options['thigh_length'],calf_length=options['calf_length'],joint_distance=options['joint_distance'],wheel_distance=options['wheel_distance'],
                        wheel_torque_limit=options['wheel_torque_limit'],joint_torque_limit=options['joint_torque_limit'])
            spawn_x,spawn_y,surface_z,yaw=spawn_pose(options['scene'])
            arena.data.qpos[0:3]=[spawn_x,spawn_y,initial_height+.075+surface_z]
            arena.data.qpos[3:7]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
            mujoco.mj_forward(arena.model,arena.data)
            if options['report']:
                report=options['report'];arena.firmware.dll.FW_SetCoefficients(np.array(report['lqr_coefficients']),np.array(report['mpc_coefficients']))
            width,height=options['resolution'];renderer=mujoco.Renderer(arena.model,height=height,width=width)
            camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
            controls=dict(keys=set(),high=False,fast=False,camera=[45,-24,3.2],loop=True,overview=False)
            start=time.perf_counter();next_frame=start;frames=0;mode=options['mode'];run=options['run'];replay_start=start
            while not stop.is_set():
                while not self.commands.empty():controls=self.commands.get()
                now=time.perf_counter();keys=controls['keys']
                if mode=='replay':
                    elapsed=now-replay_start;duration=run['records'][-1,0]-run['records'][0,0]
                    if elapsed>duration:
                        if controls['loop']:replay_start=now;elapsed=0
                        else:break
                    idx=min(len(run['qpos'])-1,np.searchsorted(run['records'][:,0],elapsed+run['records'][0,0]))
                    arena.data.qpos[:]=run['qpos'][idx];arena.data.time=run['records'][idx,0];mujoco.mj_forward(arena.model,arena.data)
                else:
                    steps=50 if mode=='batch' else min(50,max(0,int(((now-start)-arena.data.time)/.001)))
                    for _ in range(steps):
                        if stop.is_set():break
                        if mode=='batch':speed=options['batch_speed'] if arena.data.time>1 else 0;yaw=0;height=options['high_leg_height'];jump=zero=False
                        else:
                            speed=(('w' in keys)-('s' in keys))*(options['fast_speed'] if controls['fast'] else options['normal_speed'])
                            yaw=(('a' in keys)-('d' in keys))*options['yaw_speed'];height=options['high_leg_height'] if controls['high'] else options['low_leg_height']
                            jump=bool({'shift_l','shift_r'}&keys);zero='c' in keys and bool({'control_l','control_r'}&keys)
                        arena.step(speed,yaw,height,jump,zero)
                        if arena.steps%20==0:
                            records.append(np.r_[arena.data.time,arena.sensor,arena.firmware.log,arena.firmware.output])
                            positions.append(arena.data.qpos.copy())
                        if not np.isfinite(arena.data.qpos).all() or abs(arena.sensor[0])>1.2 or abs(arena.sensor[2])>1.2:
                            raise RuntimeError('底盘已倾覆，记录已保留。按 R 或进入操控重置。')
                        if arena.data.time>= (options['duration'] if mode=='batch' else 120):stop.set();break
                if now>=next_frame or stop.is_set():
                    rotation=arena.data.xmat[arena.body].reshape(3,3)
                    heading=np.arctan2(rotation[1,0],rotation[0,0])
                    camera.lookat[:]=arena.data.xpos[arena.body]+[.5*np.cos(heading),.5*np.sin(heading),.05]
                    camera.azimuth,camera.elevation,camera.distance=controls['camera']
                    camera.azimuth+=np.degrees(heading)
                    if controls['overview']:
                        camera.lookat[:]=[0,0,.15];camera.azimuth=90;camera.elevation=-58;camera.distance=19
                    renderer.update_scene(arena.data,camera);pixels=renderer.render().copy();frames+=1
                    elapsed=max(time.perf_counter()-start,.001)
                    hud=f"{mode} | t={arena.data.time:.2f}s | physics {arena.snapshot()['rtf']:.1f}x | wall {arena.data.time/elapsed:.2f}x | {frames/elapsed:.0f} FPS"
                    if mode!='replay':hud+=f" | pitch {np.degrees(arena.sensor[0]):+.1f}° | leg {arena.firmware.log[0]:.3f}/{arena.firmware.log[8]:.3f}m"
                    if self.frames.full():self.frames.get_nowait()
                    self.frames.put_nowait((pixels,hud));next_frame=max(next_frame+1/30,time.perf_counter())
                if mode!='batch':stop.wait(.002)
            if mode=='replay':self.events.put(('status','回放结束。'))
        except Exception as exc:self.events.put(('error',str(exc)))
        finally:
            if records:self.events.put(('run',dict(records=np.array(records),qpos=np.array(positions),scene=options['scene'],dt=.001,
                                                  initial_height=initial_height,substeps=options['substeps'],min_leg_height=options['min_leg_height'],
                                                  max_leg_height=options['max_leg_height'],stability_assist=options['stability_assist'],
                                                  leg_topology=options['leg_topology'],thigh_length_m=options['thigh_length'],calf_length_m=options['calf_length'],
                                                  gain_source='regenerated' if options['report'] else 'original')))
            if renderer is not None:renderer.close()
            if arena is not None:arena.firmware.close()
            self.events.put(('done',None))

    def poll(self):
        if self.closed:return
        try:
            pixels,hud=self.frames.get_nowait();picture=Image.fromarray(pixels)
            size=(max(1,self.screen.winfo_width()),max(1,self.screen.winfo_height()))
            picture.thumbnail(size,Image.Resampling.BILINEAR)
            self.photo=ImageTk.PhotoImage(picture);self.screen.configure(image=self.photo,text='');self.note.set(hud)
        except queue.Empty:pass
        while not self.events.empty():
            kind,value=self.events.get()
            if kind=='run':self.run=value;self.draw_curves()
            elif kind=='design':self.report=value;self.preview();self.note.set('30 个节点已验证稳定并导出。可勾选重算参数用于下一次物理仿真。')
            elif kind=='design_done':self.design_busy=False;self.design_button.configure(state='normal')
            elif kind=='done':self.active=False
            elif kind in ('error','status'):self.note.set(value)
        self.after(30,self.poll)

    def draw_curves(self):
        a=self.run['records'];t=a[:,0];self.figure.clear();axes=self.figure.subplots(2,4)
        for ax,cols,labels,title,unit in zip(axes.flat,[[1,2],[3,5],[7,8],[19,27],[20,28],[22,30],[39,40],[41,42,43,44]],
                [['pitch','pitch rate'],['roll','yaw'],['distance','velocity'],['L','R'],['L','R'],['L','R'],['L','R'],['L back','L front','R back','R front']],
                ['Pitch','Roll / Yaw','Travel','Leg length','Virtual theta','Theta rate','Wheel torque','Joint torque'],
                ['rad / rad/s','rad','m / m/s','m','rad','rad/s','Nm','Nm']):
            for col,label in zip(cols,labels):ax.plot(t,a[:,col],label=label,lw=1)
            ax.set(title=title,xlabel='time (s)',ylabel=unit);ax.grid(alpha=.2);ax.legend(fontsize=7)
        self.figure.tight_layout();self.plot_canvas.draw_idle()

    def export(self):
        if self.run is None:self.note.set('请先停止操控，或完成预计算。');return
        output=ROOT/'outputs'/'mujoco';output.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(output/'record.npz',**self.run)
        headers=['time','pitch','pitch_rate','roll','roll_rate','yaw','yaw_rate','distance','velocity']
        for side in ('L','R'):headers.extend(f'{side}_{x}' for x in ('phi1','phi4','rate1','rate4','normal'))
        for side in ('L','R'):headers.extend(f'{side}_{x}' for x in ('length','theta','length_rate','theta_rate','force','hip_torque','normal_control','airborne'))
        headers+=['target_velocity','target_distance','target_yaw','stair_state','wheel_L','wheel_R','back_L','front_L','back_R','front_R']
        np.savetxt(output/'record.csv',self.run['records'],delimiter=',',header=','.join(headers),comments='')
        self.figure.savefig(output/'curves.png',dpi=160);self.note.set('已导出：'+str(output))

    def design(self):
        if self.design_busy:return
        try:
            q=np.array([float(v) for v in self.q.get().split()]);r=np.array([float(v) for v in self.r.get().split()])
            if q.shape!=(6,) or r.shape!=(2,) or not np.isfinite(np.r_[q,r]).all() or (q<=0).any() or (r<=0).any():raise ValueError('Q 需要 6 个、R 需要 2 个正数')
        except ValueError as exc:messagebox.showerror('参数错误',str(exc));return
        self.design_busy=True;self.design_button.configure(state='disabled');self.note.set('C 正在求解各腿长节点的 Riccati 方程…')
        def work():
            try:
                from simulation.source_design import generate
                self.events.put(('design',generate(q_values=q,r_values=r)))
            except Exception as exc:self.events.put(('error',str(exc)))
            finally:self.events.put(('design_done',None))
        threading.Thread(target=work,daemon=True).start()

    def value(self):
        if self.report is None:return None
        key=self.matrix.get()
        return self.report[key] if key in self.report else self.report['records'][round(float(self.node.get())*100)-10][key]
    def preview(self):
        value=self.value()
        if value is None:return
        from simulation.host_app import matrix_initializer
        self.text.configure(state='normal');self.text.delete('1.0','end');self.text.insert('end',matrix_initializer(value));self.text.configure(state='disabled')
        node=self.report['records'][round(float(self.node.get())*100)-10]
        self.design_note.set(f"h={node['height']:.2f} m | C 迭代 {node['iterations']} 次 | ρ(Ad−BdK)={node['spectral_radius']:.9f} < 1 | eigenvalues 两列为实部、虚部")
    def copy_matrix(self):
        value=self.value()
        if value is None:return
        from simulation.host_app import matrix_initializer
        self.clipboard_clear();self.clipboard_append(matrix_initializer(value));self.note.set('已复制当前矩阵，无说明文字。')
    def close(self):
        self.closed=True;self.halt()
        if self.worker and self.worker.is_alive():self.worker.join(timeout=3)
