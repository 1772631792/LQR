"""Tk upper-computer application. Worker computes, main thread owns all widgets."""
import json
import queue
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk,messagebox,filedialog
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
from simulation.chassis_sim import Settings,simulate,save_run,ROOT
from simulation.chassis_view import curves,view2d,view3d
from simulation.parameter_guide import ParameterGuide,PARAMETERS
from simulation.robot_settings import RobotSettingsDialog,DEFAULT_ROBOT_CONFIG
from simulation.ui_theme import apply_theme,page_header,COLORS


def matrix_values(result,node,key):
    """Read the completed run, never unsaved edits in the settings page."""
    if key=='Q':return np.diag(result['settings']['q'])
    if key=='R':return np.array([[result['settings']['r']]])
    value=np.asarray(result['designs'][node][key])
    if key in ('B','Bd'):return value.reshape(4,1)
    if key=='K':return value.reshape(1,4)
    return value


def matrix_initializer(value):
    return '{\n'+',\n'.join('    {'+', '.join(format(float(v),'.12g') for v in row)+'}' for row in value)+'\n}'


class HostApp:
    def __init__(self,root):
        self.root=root;root.title('WheelLeg Studio');root.geometry('1380x900');root.minsize(1120,740)
        self.result=None;self.worker=None;self.cancel=threading.Event();self.messages=queue.Queue()
        self.playing=False;self.current=0.;self.internal_seek=False;self.closed=False;self.cursors={}
        self.fields={};self.canvases={};self.figures={}
        self.robot_config=dict(DEFAULT_ROBOT_CONFIG)
        apply_theme(root)
        bar=ttk.Frame(root,style='Toolbar.TFrame',padding=(18,11));bar.pack(fill='x')
        ttk.Label(bar,text='WheelLeg Studio',style='Title.TLabel').pack(side='left',padx=(0,24))
        self.start=ttk.Button(bar,text='▶  运行仿真',style='Primary.TButton',command=self.compute);self.start.pack(side='left')
        self.stop=ttk.Button(bar,text='停止',style='Danger.TButton',command=self.cancel.set,state='disabled');self.stop.pack(side='left',padx=8)
        ttk.Button(bar,text='导出结果',command=self.export).pack(side='left')
        self.progress=ttk.Progressbar(bar,length=150,maximum=100);self.progress.pack(side='right',padx=(12,0))
        ttk.Button(bar,text='⚙  设置',command=self.show_robot_settings).pack(side='right')
        workspace=ttk.Frame(root);workspace.pack(fill='both',expand=True)
        self.sidebar=ttk.Frame(workspace,style='Sidebar.TFrame',width=198,padding=(12,15));self.sidebar.pack(side='left',fill='y');self.sidebar.pack_propagate(False)
        self.tabs=ttk.Notebook(workspace,style='Hidden.TNotebook');self.tabs.pack(side='left',fill='both',expand=True,padx=(0,12),pady=(12,0))
        self.pages={}
        for name in ('仿真设置','C算法与增益','平衡曲线','腿长与电机','二维机构','三维底盘','参数导览','MuJoCo 实景','生产代码导出'):
            page=ttk.Frame(self.tabs,padding=8);self.tabs.add(page,text=name);self.pages[name]=page
        self.make_sidebar()
        self.make_settings()
        self.make_matrix_panel()
        for name in ('平衡曲线','腿长与电机','二维机构','三维底盘'):
            fig=Figure(figsize=(11,6),dpi=100)
            canvas=FigureCanvasTkAgg(fig,master=self.pages[name]);canvas.get_tk_widget().pack(fill='both',expand=True)
            NavigationToolbar2Tk(canvas,self.pages[name])
            self.figures[name]=fig;self.canvases[name]=canvas
        self.axes2=self.figures['二维机构'].subplots(1,2)
        self.axes3=self.figures['三维底盘'].add_subplot(111,projection='3d');self.axes3.view_init(22,-55)
        self.guide=ParameterGuide(self.pages['参数导览'],self.show_curve,self.robot_config)
        self.guide.pack(fill='both',expand=True)
        from simulation.mujoco_panel import MuJoCoPanel
        self.arena_panel=MuJoCoPanel(self.pages['MuJoCo 实景'],self.robot_config)
        self.arena_panel.pack(fill='both',expand=True)
        from simulation.production_export_panel import ProductionExportPanel
        self.production_panel=ProductionExportPanel(self.pages['生产代码导出'],self.arena_panel,self.robot_config)
        self.production_panel.pack(fill='both',expand=True)
        playback=ttk.Frame(root,style='Toolbar.TFrame',padding=(18,9));playback.pack(fill='x');self.playback=playback
        self.play_button=ttk.Button(playback,text='▶  播放 / 暂停',command=self.toggle_play);self.play_button.pack(side='left')
        ttk.Button(playback,text='↶  起点',command=self.rewind).pack(side='left',padx=8)
        self.loop=tk.BooleanVar(value=True)
        ttk.Checkbutton(playback,text='循环',style='Surface.TCheckbutton',variable=self.loop).pack(side='left',padx=5)
        self.slider=ttk.Scale(playback,from_=0,to=10,command=self.seek);self.slider.pack(side='left',fill='x',expand=True,padx=10)
        self.time_text=tk.StringVar(value='0.000 / 0.000 s');ttk.Label(playback,textvariable=self.time_text,width=22).pack(side='left')
        ttk.Label(playback,text='倍速').pack(side='left');self.speed=tk.StringVar(value='1')
        ttk.Combobox(playback,textvariable=self.speed,values=['0.25','0.5','1','2','4'],state='readonly',width=5).pack(side='left',padx=4)
        self.speed.trace_add('write',lambda *_:self.reset_clock())
        self.status=tk.StringVar(value='就绪 · F5 运行 · Esc 停止 · Ctrl+E 导出 · Ctrl+, 设置')
        self.status_label=ttk.Label(root,textvariable=self.status,style='Status.TLabel');self.status_label.pack(fill='x')
        self.tabs.bind('<<NotebookTabChanged>>',lambda _:self.render())
        root.bind('<F5>',lambda _:self.compute())
        root.bind('<Escape>',lambda _:self.cancel.set())
        root.bind('<Control-e>',lambda _:self.export())
        root.bind('<Control-comma>',lambda _:self.show_robot_settings())
        root.protocol('WM_DELETE_WINDOW',self.close)
        root.after(80,self.poll)

    def make_sidebar(self):
        self.nav_buttons={}
        groups=[('设计',('仿真设置','C算法与增益')),
                ('分析',('平衡曲线','腿长与电机','参数导览')),
                ('模型',('二维机构','三维底盘')),
                ('实景与部署',('MuJoCo 实景','生产代码导出'))]
        for heading,names in groups:
            ttk.Label(self.sidebar,text=heading.upper(),style='SidebarHeading.TLabel').pack(fill='x',padx=7,pady=(9,4))
            for name in names:
                button=ttk.Button(self.sidebar,text=name,style='Nav.TButton',command=lambda n=name:self.select_page(n))
                button.pack(fill='x',pady=1);self.nav_buttons[name]=button
        ttk.Frame(self.sidebar,style='Sidebar.TFrame').pack(fill='both',expand=True)
        ttk.Label(self.sidebar,text='C 控制器 · MuJoCo\n生成代码 · 外部 PID',style='Sidebar.TLabel',foreground='#86868b',justify='left').pack(anchor='w',padx=8,pady=8)
        self.update_sidebar('仿真设置')

    def select_page(self,name):
        self.tabs.select(self.pages[name]);self.update_sidebar(name);self.render()

    def update_sidebar(self,name=None):
        if name is None:name=self.tabs.tab(self.tabs.select(),'text')
        for key,button in self.nav_buttons.items():button.configure(style='NavSelected.TButton' if key==name else 'Nav.TButton')

    def show_robot_settings(self):
        RobotSettingsDialog(self.root,self.robot_config,self.apply_robot_settings)

    def apply_robot_settings(self,config):
        self.robot_config.clear();self.robot_config.update(config)
        self.arena_panel.apply_robot_config(config);self.production_panel.apply_robot_config(config);self.guide.set_robot_config(config)
        name='串联腿' if config['leg_topology']=='serial' else '五连杆'
        self.nav_buttons['参数导览'].configure(text=name+'参数导览')
        self.nav_buttons['二维机构'].configure(text='二维'+name)
        self.control_summary.set(self.topology_control_text())
        self.model_summary.set(self.topology_model_text())
        if self.result is not None:
            for page,legs in [('平衡曲线',False),('腿长与电机',True)]:
                self.cursors[page]=curves(self.figures[page],self.result,legs,self.robot_config);self.canvases[page].draw_idle()
            self.render()
        self.status.set(f'已切换为{name}；二维、三维、导览、MuJoCo 和代码生成已同步。')

    def make_matrix_panel(self):
        page=self.pages['C算法与增益']
        page_header(page,'控制器设计','查看离散模型、Riccati 解和各腿长节点的反馈增益。')
        controls=ttk.Frame(page);controls.pack(fill='x',pady=(0,10))
        ttk.Label(controls,text='腿长节点').pack(side='left',padx=5)
        self.matrix_node=ttk.Combobox(controls,values=['0.16 m','0.20 m','0.24 m'],state='readonly',width=10)
        self.matrix_node.current(1);self.matrix_node.pack(side='left')
        ttk.Label(controls,text='矩阵').pack(side='left',padx=(15,5))
        self.matrix_key=ttk.Combobox(controls,values=['A','B','Ad','Bd','Q','R','P','K'],state='readonly',width=6)
        self.matrix_key.set('K');self.matrix_key.pack(side='left')
        self.copy_button=ttk.Button(controls,text='复制此矩阵（C 数组）',command=self.copy_matrix,state='disabled');self.copy_button.pack(side='left',padx=10)
        self.copy_all_button=ttk.Button(controls,text='复制此节点全部矩阵（JSON）',command=self.copy_all_matrices,state='disabled');self.copy_all_button.pack(side='left')
        self.matrix_node.bind('<<ComboboxSelected>>',lambda _:self.refresh_matrix())
        self.matrix_key.bind('<<ComboboxSelected>>',lambda _:self.refresh_matrix())
        self.matrix_caption=tk.StringVar(value='完成计算后可复制。只包含矩阵，不包含说明、日志或其他控制参数。')
        ttk.Label(page,textvariable=self.matrix_caption).pack(anchor='w',pady=5)
        preview=ttk.LabelFrame(page,text='所选矩阵 · 只读预览',padding=5);preview.pack(fill='x')
        self.matrix_text=tk.Text(preview,height=7,font=('Consolas',11),wrap='none',state='disabled')
        scroll=ttk.Scrollbar(preview,orient='horizontal',command=self.matrix_text.xview)
        self.matrix_text.configure(xscrollcommand=scroll.set);self.matrix_text.pack(fill='x');scroll.pack(fill='x')
        details=ttk.LabelFrame(page,text='本次计算详情（不会进入矩阵剪贴板）',padding=5);details.pack(fill='both',expand=True,pady=(10,0))
        self.design_text=tk.Text(details,font=('Consolas',10),wrap='none')
        sy=ttk.Scrollbar(details,command=self.design_text.yview);sy.pack(side='right',fill='y')
        sx=ttk.Scrollbar(details,orient='horizontal',command=self.design_text.xview);sx.pack(side='bottom',fill='x')
        self.design_text.configure(yscrollcommand=sy.set,xscrollcommand=sx.set)
        self.design_text.pack(fill='both',expand=True)
        self.design_text.insert('end','计算后显示 C 求解结果；Python 不求解控制增益。')
        self.design_text.configure(state='disabled')

    def refresh_matrix(self):
        if self.result is None:return
        node=self.matrix_node.current();key=self.matrix_key.get();value=matrix_values(self.result,node,key)
        self.matrix_text.configure(state='normal');self.matrix_text.delete('1.0','end')
        self.matrix_text.insert('end',matrix_initializer(value));self.matrix_text.configure(state='disabled')
        self.matrix_caption.set(f'已完成实验 · h={self.result["designs"][node]["height"]:.2f} m · {key}：{value.shape[0]}×{value.shape[1]} · 复制不包含变量名/说明。')
        self.copy_button.configure(state='normal');self.copy_all_button.configure(state='normal')

    def copy_matrix(self):
        if self.result is None:return
        text=matrix_initializer(matrix_values(self.result,self.matrix_node.current(),self.matrix_key.get()))
        self.root.clipboard_clear();self.root.clipboard_append(text)
        self.status.set('已复制 '+self.matrix_key.get()+' 的纯 C 数组初始化值（无变量名或日志）。')

    def copy_all_matrices(self):
        if self.result is None:return
        matrices={key:matrix_values(self.result,self.matrix_node.current(),key).tolist() for key in ('A','B','Ad','Bd','Q','R','P','K')}
        self.root.clipboard_clear();self.root.clipboard_append(json.dumps(matrices,indent=2))
        self.status.set('已复制此腿长节点的 8 个矩阵（仅矩阵 JSON，无其他参数）。')

    def show_curve(self,tab,index,key):
        if self.result is None:
            self.status.set('此参数对应 '+tab+'；请先计算一次仿真查看实际曲线。');return
        for name in ('平衡曲线','腿长与电机'):
            for i,ax in enumerate(self.figures[name].axes):
                ax.set_facecolor('#fff1dc' if name==tab and i==index else 'white')
        self.tabs.select(self.pages[tab]);self.render()
        self.status.set(f'{self.guide.parameter(key)[0]} → {tab} 第 {index+1} 张子图（浅橙色高亮）。')

    def entry(self,group,label,key,value):
        row=len(group.grid_slaves())//2
        ttk.Label(group,text=label).grid(row=row,column=0,sticky='w',padx=8,pady=5)
        var=tk.StringVar(value=str(value));self.fields[key]=var
        ttk.Entry(group,textvariable=var,width=19).grid(row=row,column=1,sticky='ew',padx=8,pady=5)

    def make_settings(self):
        page=self.pages['仿真设置'];defaults=Settings()
        header=ttk.Frame(page);header.grid(row=0,column=0,columnspan=3,sticky='ew',pady=(2,12))
        ttk.Label(header,text='仿真实验',style='PageTitle.TLabel').pack(anchor='w')
        ttk.Label(header,text='设置工况、控制器和执行器参数，然后从顶部运行。',style='Secondary.TLabel').pack(anchor='w',pady=(3,0))
        groups=[]
        for column,title in enumerate(('时间与工况（秒、米、度）','C LQR / 执行器','C PID 参数（Kp Ki Kd）')):
            group=ttk.LabelFrame(page,text=title,padding=12);group.grid(row=1,column=column,sticky='new',padx=6,pady=4)
            page.columnconfigure(column,weight=1);groups.append(group)
        for label,key in [('仿真总时长','duration'),('C 控制周期 Ts','control_dt'),('物理积分步长','physics_dt'),
                          ('初始俯仰角','initial_pitch'),('初始横滚角','initial_roll'),('初始腿长','initial_height'),
                          ('目标腿长','target_height'),('目标位置','target_position'),('目标航向角','target_yaw'),('指令阶跃时刻','command_time')]:
            self.entry(groups[0],label,key,getattr(defaults,key))
        for label,key in [('Q：位置 速度 俯仰 角速度','q'),('R：输入代价','r'),('轮电机限幅 Nm','wheel_limit'),('关节电机限幅 Nm','joint_limit'),
                          ('扰动冲量 N·s','impulse'),('扰动时刻 s','impulse_time')]:
            value=getattr(defaults,key);self.entry(groups[1],label,key,' '.join(map(str,value)) if isinstance(value,tuple) else value)
        self.entry(groups[1],'播放帧率 FPS','fps',20)
        for label,key in [('左右腿长 PID','length_pid'),('横滚角 PID','roll_pid'),('航向角 PID','yaw_pid')]:
            self.entry(groups[2],label,key,' '.join(map(str,getattr(defaults,key))))
        self.control_summary=tk.StringVar(value=self.topology_control_text())
        ttk.Label(groups[2],textvariable=self.control_summary,justify='left').grid(row=3,column=0,columnspan=2,padx=8,pady=20,sticky='w')
        self.model_summary=tk.StringVar(value=self.topology_model_text())
        ttk.Label(page,textvariable=self.model_summary,wraplength=1150,justify='left',style='Secondary.TLabel').grid(row=2,column=0,columnspan=3,sticky='w',padx=12,pady=16)

    def topology_control_text(self):
        mapping='串联腿二连杆雅可比映射' if self.robot_config.get('leg_topology')=='serial' else '五连杆 VMC'
        return f'C 执行：\n• 矩阵指数离散化 + Riccati\n• 腿长增益插值 + LQR\n• 腿长 / 横滚 / 航向 PID\n• {mapping} + 力矩限幅\n\n俯仰由 LQR 控制，\n不额外叠加俯仰 PID。'

    def topology_model_text(self):
        topology='二连杆串联腿虚拟力映射' if self.robot_config.get('leg_topology')=='serial' else '五连杆虚拟力映射'
        return ('积分步长可小于控制周期；两者需整数倍。播放帧率只影响显示。计算在后台运行，完成后可暂停、拖动、倍速播放。\n'
                f'模型：变腿长倒立摆 + 横滚/航向惯量 + {topology}；固定地面接触、轻质连杆，尚无跳跃/碰撞/轮电机机体反作用力矩。')

    def read_settings(self):
        kwargs={}
        for key,var in self.fields.items():
            if key=='fps': continue
            text=var.get().replace(',',' ')
            kwargs[key]=tuple(map(float,text.split())) if key in ('q','length_pid','roll_pid','yaw_pid') else float(text)
        fps=float(self.fields['fps'].get())
        if not np.isfinite(fps) or not 1<=fps<=60: raise ValueError('播放帧率需在 1～60 FPS。')
        settings=Settings(**kwargs);settings.validate();return settings

    def compute(self):
        if self.worker and self.worker.is_alive(): return
        try: settings=self.read_settings()
        except (ValueError,TypeError) as exc: messagebox.showerror('参数错误',str(exc));return
        self.playing=False;self.cancel.clear();self.progress['value']=0
        self.start['state']='disabled';self.stop['state']='normal';self.status.set('正在后台计算：C 求增益并运行闭环……')
        def worker():
            try:
                result=simulate(settings,lambda progress:self.messages.put(('progress',progress)),self.cancel)
                self.messages.put(('done',result))
            except InterruptedError: self.messages.put(('cancelled',None))
            except Exception as exc: self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=worker,daemon=True);self.worker.start()

    def accept_result(self,result):
        self.playing=False;self.result=result;self.current=0
        self.internal_seek=True;self.slider.configure(to=result['time'][-1]);self.slider.set(0);self.internal_seek=False
        self.xlimits=(float(result['states'][:,0].min()-.35),float(result['states'][:,0].max()+.35))
        for name,legs in [('平衡曲线',False),('腿长与电机',True)]:
            self.cursors[name]=curves(self.figures[name],result,legs,self.robot_config);self.canvases[name].draw_idle()
        self.design_text.configure(state='normal');self.design_text.delete('1.0','end')
        text='C 算法：ZOH 矩阵指数 → DARE 迭代 → K；控制周期内按实际腿长插值。\n'
        text+='Python 仅构建物理 A/B、积分和绘图。以下增益和矩阵来自 C DLL。\n'
        text+=f'仿真 {result["time"][-1]:g} s；计算耗时 {result["elapsed"]:.3f} s\n'
        text+=json.dumps({'settings':result['settings'],'C_designs':result['designs']},indent=2)
        self.design_text.insert('end',text)
        self.design_text.configure(state='disabled');self.refresh_matrix()
        self.guide.set_sample(result['states'][0],result['outputs'][0],0.)
        self.status.set(f'计算完成：{result["time"][-1]:g} 秒仿真，耗时 {result["elapsed"]:.3f} 秒。点击播放或拖动时间轴。')
        self.tabs.select(self.pages['参数导览']);self.render()

    def poll(self):
        if self.closed:return
        try:
            while True:
                kind,value=self.messages.get_nowait()
                if kind=='progress':self.progress['value']=100*value
                else:
                    self.start['state']='normal';self.stop['state']='disabled'
                    if kind=='done':self.progress['value']=100;self.accept_result(value)
                    elif kind=='error':self.status.set('计算失败：'+value);messagebox.showerror('仿真失败',value)
                    else:self.status.set('计算已取消，保留上一组完成的结果。')
        except queue.Empty:pass
        if self.playing and self.result is not None:
            self.advance_playback(time.perf_counter())
            self.internal_seek=True;self.slider.set(self.current);self.internal_seek=False;self.render()
        try: delay=round(1000/max(1,min(60,float(self.fields['fps'].get()))))
        except (ValueError,OverflowError):delay=50
        self.root.after(delay,self.poll)

    def advance_playback(self,now):
        end=float(self.result['time'][-1])
        position=self.anchor_time+(now-self.anchor_wall)*float(self.speed.get())
        if position>=end:
            if self.loop.get():
                position%=end;self.anchor_time=position;self.anchor_wall=now
            else:
                position=end;self.playing=False
        self.current=position

    def reset_clock(self):
        self.anchor_time=self.current;self.anchor_wall=time.perf_counter()

    def toggle_play(self):
        if self.result is None:return
        if self.current>=self.result['time'][-1]:self.current=0
        self.playing=not self.playing;self.reset_clock()

    def seek(self,value):
        if self.internal_seek or self.result is None:return
        self.current=float(value);self.reset_clock();self.render()

    def rewind(self):
        self.playing=False;self.seek('0');self.slider.set(0)

    def render(self):
        name=self.tabs.tab(self.tabs.select(),'text')
        if hasattr(self,'nav_buttons'):self.update_sidebar(name)
        if name not in ('平衡曲线','腿长与电机','二维机构','三维底盘','参数导览'):self.playback.pack_forget()
        elif not self.playback.winfo_manager():self.playback.pack(fill='x',before=self.status_label)
        if self.result is None:return
        index=min(np.searchsorted(self.result['time'],self.current),len(self.result['time'])-1)
        self.time_text.set(f'{self.current:.3f} / {self.result["time"][-1]:.3f} s')
        if name in self.cursors:
            for cursor in self.cursors[name]:cursor.set_xdata([self.current,self.current])
        elif name=='二维机构':view2d(self.axes2,self.result['states'][index],self.xlimits,self.robot_config);self.figures[name].tight_layout()
        elif name=='三维底盘':view3d(self.axes3,self.result['states'][index],self.xlimits,self.robot_config)
        elif name=='参数导览':
            output=self.result['outputs'][index] if index<len(self.result['outputs']) else None
            self.guide.set_sample(self.result['states'][index],output,self.result['time'][index])
        if name in self.canvases:self.canvases[name].draw_idle()

    def export_to(self,directory):
        save_run(self.result,directory)
        for name,filename in [('平衡曲线','balance.png'),('腿长与电机','legs_motors.png')]:self.figures[name].savefig(Path(directory)/filename,dpi=150)
        index=min(np.searchsorted(self.result['time'],self.current),len(self.result['time'])-1)
        view2d(self.axes2,self.result['states'][index],self.xlimits,self.robot_config);self.figures['二维机构'].tight_layout()
        view3d(self.axes3,self.result['states'][index],self.xlimits,self.robot_config)
        for name,filename in [('二维机构','robot_2d.png'),('三维底盘','robot_3d.png')]:self.figures[name].savefig(Path(directory)/filename,dpi=150)

    def export(self):
        if self.result is None:messagebox.showinfo('尚无结果','请先完成一次仿真。');return
        directory=filedialog.askdirectory(title='选择导出文件夹')
        if directory:
            try:self.export_to(directory);self.status.set('已导出 CSV / NPZ / C参数 / 四张图片：'+directory)
            except OSError as exc:messagebox.showerror('导出失败',str(exc))

    def close(self):
        self.closed=True;self.cancel.set();self.arena_panel.close();self.production_panel.close();self.root.destroy()


def launch(settings=None):
    root=tk.Tk();app=HostApp(root)
    app.tabs.select(app.pages['MuJoCo 实景'])
    if settings is not None:
        from dataclasses import asdict
        for key,value in asdict(settings).items():
            app.fields[key].set(' '.join(map(str,value)) if isinstance(value,tuple) else str(value))
    root.mainloop()
