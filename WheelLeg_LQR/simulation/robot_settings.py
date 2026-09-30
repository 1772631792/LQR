"""Shared robot/leg configuration dialog for simulation and code export."""
import math
import tkinter as tk
from tkinter import ttk,messagebox
from simulation.ui_theme import COLORS


DEFAULT_ROBOT_CONFIG={
    'leg_topology':'five_bar','thigh_length_m':.135,'calf_length_m':.24,
    'joint_distance_m':.12,'wheel_distance_m':.52,
    'min_leg_length_m':.15,'max_leg_length_m':.30,
    'wheel_torque_limit_nm':8.,'joint_torque_limit_nm':35.,
}


class RobotSettingsDialog(tk.Toplevel):
    def __init__(self,parent,config,on_apply):
        super().__init__(parent);self.title('机器人设置 · 仿真与代码生成共用');self.resizable(False,False);self.configure(bg=COLORS['window'])
        self.transient(parent);self.on_apply=on_apply
        self.vars={key:tk.StringVar(value=str(value)) for key,value in config.items()}
        self.vars['leg_topology'].set('串联腿（二关节开链）' if config['leg_topology']=='serial' else '五连杆（闭链）')
        body=ttk.Frame(self,padding=16);body.pack(fill='both',expand=True)
        ttk.Label(body,text='腿型配置',font=('Microsoft YaHei UI',14,'bold')).grid(row=0,column=0,columnspan=2,sticky='w',pady=(0,10))
        fields=[('腿型','leg_topology'),('大腿长度 m','thigh_length_m'),('小腿长度 m','calf_length_m'),
                ('五连杆髋间距 m','joint_distance_m'),('左右轮距 m','wheel_distance_m'),
                ('最短有效腿长 m','min_leg_length_m'),('最长有效腿长 m','max_leg_length_m'),
                ('轮电机力矩限幅 Nm','wheel_torque_limit_nm'),('关节电机力矩限幅 Nm','joint_torque_limit_nm')]
        for row,(label,key) in enumerate(fields,1):
            ttk.Label(body,text=label).grid(row=row,column=0,sticky='e',padx=(0,8),pady=4)
            if key=='leg_topology':
                widget=ttk.Combobox(body,textvariable=self.vars[key],state='readonly',width=27,
                                    values=['五连杆（闭链）','串联腿（二关节开链）'])
            else:widget=ttk.Entry(body,textvariable=self.vars[key],width=29)
            widget.grid(row=row,column=1,sticky='ew',pady=4)
        self.help=tk.StringVar();ttk.Label(body,textvariable=self.help,wraplength=470,justify='left').grid(row=10,column=0,columnspan=2,sticky='w',pady=(10,6))
        actions=ttk.Frame(body);actions.grid(row=11,column=0,columnspan=2,sticky='e')
        ttk.Button(actions,text='取消',command=self.destroy).pack(side='right')
        ttk.Button(actions,text='应用到仿真和代码生成',style='Primary.TButton',command=self.apply).pack(side='right',padx=8)
        self.vars['leg_topology'].trace_add('write',lambda *_:self.update_help());self.update_help()
        self.protocol('WM_DELETE_WINDOW',self.destroy);self.grab_set();self.focus_set()

    def update_help(self):
        serial=self.vars['leg_topology'].get().startswith('串联')
        self.help.set('串联腿模式：输入为每侧髋角、膝角，使用二连杆正运动学和雅可比 VMC；参数会同时写入 MuJoCo 与导出的 C 代码。'
                      if serial else '五连杆模式：保持当前双髋闭链机构与原参考固件控制路径。')

    def apply(self):
        try:
            result={'leg_topology':'serial' if self.vars['leg_topology'].get().startswith('串联') else 'five_bar'}
            for key,var in self.vars.items():
                if key!='leg_topology':result[key]=float(var.get())
            if not all(math.isfinite(v) and v>0 for k,v in result.items() if k!='leg_topology'):raise ValueError('所有尺寸和限幅必须是有限正数。')
            if not .09<=result['min_leg_length_m']<result['max_leg_length_m']<=.39:raise ValueError('腿长需满足 0.09 ≤ 最短 < 最长 ≤ 0.39 m。')
            if result['leg_topology']=='serial':
                low=abs(result['thigh_length_m']-result['calf_length_m']);high=result['thigh_length_m']+result['calf_length_m']
                if not low<result['min_leg_length_m']<result['max_leg_length_m']<high:
                    raise ValueError(f'串联腿可达范围为 {low:.3f}–{high:.3f} m，机械限位必须严格位于其中。')
            elif not .14<=result['min_leg_length_m']<result['max_leg_length_m']<=.33:
                raise ValueError('当前五连杆几何限位需满足 0.14 ≤ 最短 < 最长 ≤ 0.33 m。')
            self.on_apply(result);self.destroy()
        except ValueError as exc:messagebox.showerror('设置错误',str(exc),parent=self)
