"""Interactive oblique schematic: UI-only mapping from mechanism to logged signals."""
import tkinter as tk
from tkinter import ttk
import numpy as np
from simulation.robot_view import geometry as five_bar_geometry
from simulation.serial_leg_view import geometry as serial_geometry

# (Chinese name, symbol, destination tab, subplot index, explanation)
PARAMETERS = {
    'pos': ('水平位置', 'p', '平衡曲线', 0, '轮轴沿前进方向的位置。单位 m，向前为正；它是 LQR 第 1 个状态。'),
    'vel': ('前进速度', 'v = dp/dt', '平衡曲线', 1, '水平位置的变化率。单位 m/s；它是 LQR 第 2 个状态。'),
    'pitch': ('机体俯仰角', 'theta / pitch', '平衡曲线', 2, '机体相对竖直方向的前后倾角，前倾为正。内部 rad，曲线显示 deg；LQR 第 3 个状态。不是髋关节角。'),
    'roll': ('机体横滚角', 'roll', '平衡曲线', 3, '机体左右倾斜角，左侧抬高为正。横滚 PID 调整左右支撑力差；它不属于四状态 LQR。'),
    'yaw': ('机体航向角', 'yaw', '平衡曲线', 4, '从上方看，车头在水平面内的转角。左转为正；航向 PID 通过左右轮差动实现转向。'),
    'left_height': ('左腿有效长度', 'hL = h + 0.18*roll', '腿长与电机', 0, '左侧五连杆：髋中点到轮轴的有效伸缩长度，单位 m。不是某根杆的长度。左右按机器人自身朝向定义。'),
    'right_height': ('右腿有效长度', 'hR = h - 0.18*roll', '腿长与电机', 1, '右侧五连杆的有效伸缩长度，单位 m。小横滚角模型中由平均腿长 h 和 roll 得到。'),
    'support': ('左右腿支撑力', 'F_L / F_R', '腿长与电机', 2, '腿长 PID、横滚 PID 和重力前馈共同产生的轴向支撑力，单位 N；关节限幅后是实际施加值。'),
    'wheels': ('左右轮电机转矩', 'tau_L / tau_R', '腿长与电机', 3, '左右轮转矩，单位 N·m。共同分量来自 LQR，差动分量用于航向控制。图中深色圆环为轮。'),
    'left_hip': ('左腿髋关节电机', 'tau_LA / tau_LE', '腿长与电机', 4, '左腿 A/E 为主动关节，B/D 为被动膝关节，C 是轮轴。曲线显示 VMC 输出转矩 N·m。qA/qE 是几何关节角，不是机体 pitch。'),
    'right_hip': ('右腿髋关节电机', 'tau_RA / tau_RE', '腿长与电机', 5, '右腿 A/E 主动关节的 VMC 转矩，单位 N·m。关节角 qA/qE 以局部 +x 为零、逆时针为正，当前作为几何量展示。'),
}


def signal_value(key, state, output):
    if key in ('pos','vel'): return f'{state[0 if key=="pos" else 1]:+.4f} '+('m' if key=='pos' else 'm/s')
    if key in ('pitch','roll','yaw'):
        index={'pitch':2,'roll':6,'yaw':8}[key]
        return f'{np.rad2deg(state[index]):+.3f}°  ({state[index]:+.5f} rad)'
    if key in ('left_height','right_height'):
        h=state[4]+(1 if key=='left_height' else -1)*.18*state[6]
        return f'{h:.4f} m'
    if output is None: return '此帧无控制输出'
    indices={'support':(1,2),'wheels':(4,5),'left_hip':(6,7),'right_hip':(8,9)}[key]
    return ' / '.join(f'{output[i]:+.3f}' for i in indices)+(' N' if key=='support' else ' N·m')


class ParameterGuide(ttk.Frame):
    def __init__(self,parent,on_curve,robot_config=None):
        super().__init__(parent)
        self.robot_config=robot_config or {'leg_topology':'five_bar'}
        self.on_curve=on_curve;self.selected='pitch';self.hovered=None;self.pinned=False
        self.state=np.array([0,0,np.deg2rad(10),0,.2,0,0,0,0,0.]);self.output=None;self.timestamp=None
        self.canvas=tk.Canvas(self,bg='#f4f7fb',highlightthickness=0,width=850,height=560)
        self.canvas.pack(side='left',fill='both',expand=True)
        panel=ttk.Frame(self,padding=15,width=260);panel.pack(side='right',fill='y');panel.pack_propagate(False)
        ttk.Label(panel,text='参数说明',font=('Microsoft YaHei UI',14,'bold')).pack(anchor='w',pady=(0,12))
        self.title=tk.StringVar();self.value=tk.StringVar();self.detail=tk.StringVar();self.destination=tk.StringVar()
        ttk.Label(panel,textvariable=self.title,font=('Microsoft YaHei UI',12,'bold'),wraplength=225).pack(anchor='w',pady=6)
        ttk.Label(panel,textvariable=self.value,font=('Consolas',11),wraplength=225,foreground='#146c94').pack(anchor='w',pady=10)
        ttk.Label(panel,textvariable=self.detail,wraplength=225,justify='left').pack(anchor='w',pady=10)
        ttk.Label(panel,textvariable=self.destination,wraplength=225,foreground='#596b80').pack(anchor='w',pady=12)
        ttk.Button(panel,text='查看对应曲线 →',command=self.jump).pack(fill='x',pady=5)
        ttk.Button(panel,text='取消固定选择',command=self.unpin).pack(fill='x',pady=5)
        ttk.Label(panel,text='悬停：预览参数\n单击：固定选择\n双击：跳到对应曲线\n\n引出线与虚线是参数标注。\n斜视图固定航向以便认清左右；\n航向箭头仅示意转向方向，\n实际角度请看当前值/三维页。',wraplength=225,foreground='#596b80').pack(anchor='w',pady=20)
        self.tooltip=tk.Label(self.canvas,bg='#16304c',fg='white',padx=9,pady=6,justify='left',font=('Microsoft YaHei UI',10))
        self.canvas.bind('<Configure>',lambda _:self.draw())
        self.canvas.bind('<Motion>',self.motion);self.canvas.bind('<Leave>',lambda _:self.leave())
        self.canvas.bind('<Button-1>',self.click);self.canvas.bind('<Double-Button-1>',self.double_click)
        self.update_detail();self.draw()

    def parameter(self,key):
        if self.robot_config.get('leg_topology')!='serial':return PARAMETERS[key]
        replacements={
            'left_height':('左串联腿有效长度','hL = h + 0.18*roll','腿长与电机',0,'左侧二连杆串联腿从髋关节到轮轴的有效长度，单位 m。'),
            'right_height':('右串联腿有效长度','hR = h - 0.18*roll','腿长与电机',1,'右侧二连杆串联腿从髋关节到轮轴的有效长度，单位 m。'),
            'left_hip':('左腿髋/膝关节电机','tau_LH / tau_LK','腿长与电机',4,'左串联腿 H/K 为髋、膝主动关节。曲线显示接口输出的两个关节转矩，便于替换为自己的 PID。'),
            'right_hip':('右腿髋/膝关节电机','tau_RH / tau_RK','腿长与电机',5,'右串联腿 H/K 为髋、膝主动关节；qHip/qKnee 使用串联腿控制器的角度约定。'),
        }
        return replacements.get(key,PARAMETERS[key])

    def leg_geometry(self,height):
        if self.robot_config.get('leg_topology')=='serial':
            return serial_geometry(0,self.state[2],height,self.robot_config.get('thigh_length_m',.135),self.robot_config.get('calf_length_m',.24))
        return five_bar_geometry(0,self.state[2],height)

    def set_robot_config(self,config):
        self.robot_config=config
        self.update_detail();self.draw()

    def set_sample(self,state,output=None,timestamp=None):
        self.state=np.asarray(state);self.output=output;self.timestamp=timestamp
        self.update_detail();self.draw()
        if self.hovered:self.tooltip.configure(text=self.tooltip_text(self.hovered))

    def tooltip_text(self,key):
        name,symbol,tab,_,_=self.parameter(key)
        return f'{name} · {symbol}\n{signal_value(key,self.state,self.output)}\n曲线：{tab}（双击跳转）'

    def active(self): return self.selected if self.pinned else (self.hovered or self.selected)

    def update_detail(self):
        key=self.active();name,symbol,tab,index,description=self.parameter(key)
        self.title.set(name+'\n'+symbol)
        value=signal_value(key,self.state,self.output)
        if key in ('left_hip','right_hip'):
            h=self.state[4]+(1 if key=='left_hip' else -1)*.18*self.state[6]
            _,_,_,angles=self.leg_geometry(h)
            labels=('qHip','qKnee') if self.robot_config.get('leg_topology')=='serial' else ('qA','qE')
            value+=f'\n{labels[0]}={np.rad2deg(angles[0]):.2f}°\n{labels[1]}={np.rad2deg(angles[1]):.2f}°'
        self.value.set(value if self.timestamp is not None else '示意姿态（尚未计算）\n'+value)
        self.detail.set(description);self.destination.set(f'对应：{tab}\n第 {index+1} 张子图\n'+('已固定选择' if self.pinned else '悬停预览'))

    def key_at(self,event):
        items=self.canvas.find_overlapping(event.x-2,event.y-2,event.x+2,event.y+2)
        for item in reversed(items):
            for tag in self.canvas.gettags(item):
                if tag.startswith('param:'): return tag[6:]
        return None

    def motion(self,event):
        key=self.key_at(event)
        if key!=self.hovered:
            self.hovered=key;self.update_detail();self.draw()
        self.canvas.configure(cursor='hand2' if key else '')
        if key:
            self.tooltip.configure(text=self.tooltip_text(key))
            self.tooltip.place(x=min(event.x+14,max(0,self.canvas.winfo_width()-315)),y=min(event.y+18,max(0,self.canvas.winfo_height()-95)))
        else:self.tooltip.place_forget()

    def leave(self):
        self.hovered=None;self.tooltip.place_forget();self.update_detail();self.draw()

    def click(self,event):
        key=self.key_at(event)
        if key:self.selected=key;self.pinned=True;self.update_detail();self.draw()

    def double_click(self,event):
        self.click(event)
        if self.key_at(event):self.jump()

    def unpin(self):self.pinned=False;self.update_detail();self.draw()

    def jump(self):
        key=self.active();_,_,tab,index,_=self.parameter(key);self.on_curve(tab,index,key)

    def draw(self):
        c=self.canvas;c.delete('all');w=max(c.winfo_width(),720);h=max(c.winfo_height(),480)
        scale=min(w/1.28,h/.76);cx=w*.49;cy=h*.76
        def project(points,y):
            points=np.asarray(points)
            return np.column_stack([cx+scale*(points[:,0]+.53*y),cy-scale*(points[:,1]-.23*y)])
        def tags(key):return ('param:'+key,)
        def color(key,normal):return '#b64074' if self.active()==key else normal
        def line(points,key,normal,width=4,**kwargs):
            return c.create_line(*np.asarray(points).ravel(),fill=color(key,normal),width=width,tags=tags(key),**kwargs)
        # Body and the selected leg renderer use current pitch/roll/height.
        # yaw is an explanatory ground arrow so left/right labels stay readable.
        anchors={};s=self.state
        for side,y,key in [('右',-.18,'right_height'),('左',.18,'left_height')]:
            pts,body,com,angles=self.leg_geometry(s[4]+y*s[6]);q=project(pts,y)
            hipkey='left_hip' if y>0 else 'right_hip'
            serial=self.robot_config.get('leg_topology')=='serial'
            anchors[key]=(q[0]+q[2])/2 if serial else (q[0]+q[4]+2*q[2])/4
            anchors[hipkey]=(q[0]+q[1])/2 if serial else (q[0]+q[4])/2
            segments=([0,1,2],) if serial else ([0,1,2],[4,3,2])
            for ids in segments:line(q[list(ids)],key,'#168aad' if y>0 else '#eb8c3d',5)
            if not serial:line(q[[0,4]],hipkey,'#526477',5)
            for i,point in enumerate(q):
                target=hipkey if i in ((0,1) if serial else (0,4)) else key
                c.create_oval(point[0]-4,point[1]-4,point[0]+4,point[1]+4,fill=color(target,'#526477'),outline='',tags=tags(target))
                if i!=2:c.create_text(point[0]+9,point[1]-12,text=('HKW' if serial else 'ABCDE')[i],fill='#34475b',font=('Consolas',10,'bold'),tags=tags(target))
            wheel=project(np.column_stack([.06*np.cos(np.linspace(0,2*np.pi,48)),.06+.06*np.sin(np.linspace(0,2*np.pi,48))]),y)
            c.create_polygon(*wheel.ravel(),fill='#dbe5ef',outline=color('wheels','#283c51'),width=6,tags=tags('wheels'))
            line([q[2],q[2]+[scale*.045,0]],'wheels','#526477',3)
            line([project([[0,s[4]+y*s[6]+.06]],y)[0],q[2]],key,'#6b849d',1,dash=(5,4),arrow='both')
            c.create_text(q[2][0],q[2][1]+scale*.08,text=side+'轮 / '+side+'腿',font=('Microsoft YaHei UI',10,'bold'),fill='#34475b',tags=tags(key))
        vertices=[]
        for y in (-.13,.13):
            _,body,_,_=self.leg_geometry(s[4]+y*s[6]);vertices.extend(project(body,y))
        vertices=np.array(vertices)
        for face in ([0,1,2,3],[3,2,6,7],[1,5,6,2]):
            c.create_polygon(*vertices[face].ravel(),fill='#dce8f2',outline=color('pitch','#526477'),width=2,tags=tags('pitch')+('chassis',))
        c.tag_lower('chassis')
        center=vertices.mean(axis=0);anchors['pitch']=center;anchors['roll']=(vertices[2]+vertices[6])/2
        line([center,center+[0,-scale*.12]],'pitch','#7d91a6',1,dash=(4,3))
        theta=s[2];axis=np.array([np.sin(theta),-np.cos(theta)])*scale*.12
        line([center,center+axis],'pitch','#b64074',2,arrow='last')
        if abs(theta)>.005:
            arc=np.linspace(0,theta,25);points=center+scale*.08*np.column_stack([np.sin(arc),-np.cos(arc)])
            line(points,'pitch','#b64074',2,arrow='last')
        ground_y=cy+scale*.075
        line([[cx-scale*.22,ground_y],[cx+scale*.22,ground_y]],'pos','#71849a',1,arrow='both')
        anchors['pos']=np.array([cx-scale*.10,ground_y]);anchors['vel']=np.array([cx+scale*.21,ground_y])
        line([[cx+scale*.13,ground_y-15],[cx+scale*.26,ground_y-15]],'vel','#146c94',3,arrow='last')
        angles=np.linspace(-.3,.9,25);yawarc=np.column_stack([cx+scale*.12*np.cos(angles),ground_y+scale*.045*np.sin(angles)])
        line(yawarc,'yaw','#6c6eb2',2,arrow='last');anchors['yaw']=yawarc[-1]
        anchors['wheels']=project([[0,.06]],-.18)[0];anchors['support']=project([[0,.13]],.18)[0]
        title='串联腿参数导览' if self.robot_config.get('leg_topology')=='serial' else '五连杆参数导览'
        c.create_text(w/2,25,text=title+' · 固定斜视',font=('Microsoft YaHei UI',15,'bold'),fill='#233a51')
        c.create_text(w/2,51,text='悬停看说明  ·  单击固定  ·  双击定位曲线',font=('Microsoft YaHei UI',10),fill='#6c7d90')
        self.label_centers={}
        left=['pitch','roll','right_hip','right_height','wheels','pos']
        right=['left_hip','left_height','support','yaw','vel']
        for keys,x,anchor in [(left,18,'w'),(right,w-18,'e')]:
            for i,key in enumerate(keys):
                yy=100+i*(h-155)/max(len(keys)-1,1)
                target=anchors[key];endx=x+145 if anchor=='w' else x-145
                line([target,[endx,yy],[x+(4 if anchor=='w' else -4),yy]],key,'#a4b2c1',1)
                label=self.parameter(key)[0]
                item=c.create_text(x,yy-9,text=label,anchor=anchor,font=('Microsoft YaHei UI',10,'bold'),fill=color(key,'#294762'),tags=tags(key))
                c.create_text(x,yy+9,text=self.parameter(key)[1],anchor=anchor,font=('Consolas',9),fill='#71849a',tags=tags(key))
                bounds=c.bbox(item);self.label_centers[key]=((bounds[0]+bounds[2])/2,(bounds[1]+bounds[3])/2)
