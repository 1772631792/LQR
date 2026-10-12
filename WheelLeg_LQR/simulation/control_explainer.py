"""Code-aligned instructional drawings and equations for the serial-leg controller."""
import math
import tkinter as tk
from tkinter import ttk


CONTROL_TEXT = '''你到底在控制什么？
目标是让机身保持平衡、按目标速度前进、保持目标腿长并按目标航向转向。
最终送给执行器的是六路力矩（N·m）：左轮、右轮、左髋、左膝、右髋、右膝。
髋角和膝角是测量值；当前平衡控制不是把每个关节锁定到某个固定角度。

① 编码器 → 虚拟腿：先算出“腿现在是什么姿态”
以下严格采用 controller/firmware_bridge.c 的 Serial2Leg 约定：
局部 x 向前，局部 y 向下；qH=0 表示大腿朝机身下方；qK 为小腿相对大腿延长线的转角。
a = 大腿长，b = 小腿长；qH、qK、pitch 都用 rad。
x = a·sin(qH) + b·sin(qH + qK)
y = a·cos(qH) + b·cos(qH + qK)
L = sqrt(x² + y²)                         ← 髋到轮轴的虚拟腿长
phi0 = atan2(y, x)                         ← 虚拟腿在局部平面里的方向
theta = phi0 − π/2 − pitch                ← 控制器的虚拟腿摆角
theta_dot = phi0_dot − pitch_dot
因此：theta、pitch、qH、qK 是四个不同的角！theta 的正负按上式定义。

② 六状态 LQR → “轮怎么推、虚拟腿怎么摆”
MuJoCo 每侧腿使用的状态顺序：
X = [theta, theta_dot, s, v, pitch, pitch_dot]ᵀ     （6×1）
Xref = [0, 0, s_ref, v_ref, 0, 0]ᵀ
e = Xref − X
u_LQR = K(L)·e = [T_wheel, T_virtual]ᵀ           （K 为 2×6）
T_wheel   = K11·e1 + K12·e2 + … + K16·e6
T_virtual = K21·e1 + K22·e2 + … + K26·e6
T_virtual 是作用于整个虚拟腿的摆动广义力矩，不等于髋电机最终力矩。
默认固件还混合 MPC：u = (1−alpha)·u_LQR + alpha·u_MPC，两输出权重分别设置。
当前默认 alpha_wheel=0，alpha_virtual=0.3；所以轮输出是纯 LQR，摆腿输出为 70% LQR + 30% MPC。
离地时会切换输出规则；以上描述正常接地的平衡主通道。

③ 腿长 / 横滚 / 航向 → 独立控制通道
L_ref − L → 腿长位置环 → 伸缩速度目标 → 速度 PI + 重力前馈 → F_leg（N）
roll → 左右支撑力差，调节左右倾斜。
yaw_ref − yaw → 航向环 → 转向角速度环 → steer（N·m）
左轮目标力矩 = T_wheel,L − steer
右轮目标力矩 = T_wheel,R + steer
yaw 不在上述六状态 LQR 里；腿长 L 用于独立伸缩控制和选择 K(L)。
摆腿力矩还叠加左右腿协调、转向补偿，再进入下一步。

④ 虚拟力 → 髋、膝电机力矩：推导最后等于什么？
J = ∂[L, phi0] / ∂[qH, qK] = [[J11,J12],[J21,J22]]
对上面的二连杆公式求导，得到：
J11 = 0；J12 = −a·b·sin(qK) / L
J21 = −1；J22 = −b·(a·cos(qK) + b) / L²
[L_dot, phi0_dot]ᵀ = J·[qH_dot, qK_dot]ᵀ
由虚功：tauH·δqH + tauK·δqK = F_leg·δL + T_virtual·δphi0
所以 [tauH, tauK]ᵀ = Jᵀ·[F_leg, T_virtual]ᵀ
tauH = J11·F_leg + J21·T_virtual
tauK = J12·F_leg + J22·T_virtual
这才是每条串联腿两个电机的目标力矩；左右各算一次，再限幅，输出六路力矩。
这里的 J 是 [L,phi0] 的雅可比，不是 [x,y] 的雅可比，不能混用。

⑤ Q / R 到 K：离线设计算什么？
在平衡点附近线性化：X_dot = A·X + B·u
离散化：X[k+1] = Ad·X[k] + Bd·u[k]
离散代价 Σ(eᵀQd e + 2eᵀNd u + uᵀRd u) 对应 Riccati 方程：
S = Rd + BdᵀPBd
P = AdᵀPAd + Qd − (AdᵀPBd + Nd)·S⁻¹·(BdᵀPAd + Ndᵀ)
K = S⁻¹·(BdᵀPAd + Ndᵀ)
直接给离散 Q/R 且无交叉项时 Nd=0；连续代价精确离散化可能产生 Nd，六状态设计需保留它。
在线每个控制周期只需读取状态、计算 e，再算 K·e。
六状态 Q 对应上面的六个状态；R 对应轮转矩和虚拟摆腿力矩两路输出。
负载、质心、惯量变化会改变 A/B；Q/R 表示你对误差和控制力度的取舍。

当前软件中的两条链路必须分清
“简化计算预览 / C算法与增益”：四状态 [p,v,pitch,pitch_dot]，K 为 1×4，输出前后驱动力 F_drive（N）。
“MuJoCo 实景”：上面的六状态虚拟腿模型，K 为 2×6，输出两种力矩。六状态重算在 MuJoCo 的增益设计入口。
四状态参数页生成的 K 不会自动替换 MuJoCo 六状态 K。
现有参数导览播放的是简化实验记录；切换串联腿外观不会把这些记录变成六状态实景遥测。
串联腿的默认增益仍来自现有虚拟腿模型；更改实际质量/惯量后需要相应重建六状态动力学。
'''


def parallel_control_text():
    intro='''并联五连杆：控制目标与最终输出
目标同样是平衡、前进、腿长和转向；六路输出为左右轮、左后髋 A、左前髋 E、右后髋 A、右前髋 E 的力矩。
B、D 是被动膝关节，不接收电机力矩命令。

① 编码器 → 闭链 → 虚拟腿
局部坐标：A=(0,0)，E=(d,0)，O=(d/2,0)，x 向前、y 向下。
phi1 和 phi4 均从局部 +x 量起，分别是 AB 和 ED 的杆角。
B = [a cos(phi1), a sin(phi1)]
D = [d+a cos(phi4), a sin(phi4)]
解 |C−B|²=b² 与 |C−D|²=b²，并保持同一装配分支，得到轮轴 C。
x = xC−d/2；y = yC；L = sqrt(x²+y²)
phi0 = atan2(y,x)；theta = phi0−π/2−pitch
theta_dot = phi0_dot−pitch_dot
pitch 是机身前后倾角，yaw 是航向角，phi1/phi4 是两个髋角，theta 是虚拟腿摆角。

'''
    shared=CONTROL_TEXT[CONTROL_TEXT.index('② 六状态'):CONTROL_TEXT.index('④ 虚拟力')]
    mapping='''④ 虚拟力 → 后髋 A / 前髋 E 的电机力矩
先微分两个闭链约束求 Jxy = ∂[x,y]/∂[phi1,phi4]。
J = [[x/L, y/L], [−y/L², x/L²]] · Jxy
[L_dot,phi0_dot]ᵀ = J·[phi1_dot,phi4_dot]ᵀ
由虚功：[tauA,tauE]ᵀ = Jᵀ·[F_leg,T_virtual]ᵀ
tauA = J11·F_leg + J21·T_virtual
tauE = J12·F_leg + J22·T_virtual
左右各做一次，叠加轮转向控制后限幅，得到六路电机力矩。
这里不能使用串联腿的 J11=0、J21=−1；闭链雅可比取决于当前五连杆构型。

'''
    end=CONTROL_TEXT[CONTROL_TEXT.index('⑤ Q / R'):]
    end=end.replace('切换串联腿外观','切换腿型外观').replace('串联腿的默认增益','并联腿的默认增益')
    return intro+shared+mapping+end


class AngleExplainer(ttk.Frame):
    def __init__(self,parent,serial=True):
        super().__init__(parent)
        self.serial=serial
        ttk.Label(self,text='角度定义示意：拖动滑块，观察参考线与旋转方向（不是实景遥测）',padding=8).pack(fill='x')
        controls=ttk.Frame(self);controls.pack(fill='x',padx=10)
        self.values={}
        joints=([('qH','髋角 qH',-40,-80,30),('qK','膝角 qK',90,10,140)] if serial else
                [('qH','后髋 phi1',115,100,130),('qK','前髋 phi4',65,50,80)])
        for key,title,value,lo,hi in [('pitch','pitch 前倾',15,-30,30),('yaw','yaw 左转',30,-90,90)]+joints:
            box=ttk.Frame(controls);box.pack(side='left',fill='x',expand=True,padx=6)
            ttk.Label(box,text=title+'（°）').pack()
            var=tk.DoubleVar(value=value);self.values[key]=var
            ttk.Scale(box,from_=lo,to=hi,variable=var,command=lambda _:self.draw()).pack(fill='x')
        self.canvas=tk.Canvas(self,bg='#f4f7fb',height=530,highlightthickness=0)
        self.canvas.pack(fill='both',expand=True);self.canvas.bind('<Configure>',lambda _:self.draw())

    def draw(self):
        c=self.canvas;c.delete('all');w=max(c.winfo_width(),700);h=max(c.winfo_height(),440)
        k=min(w/1000,h/540);ox=(w-1000*k)/2
        def point(x,y):return ox+x*k,y*k
        def line(points,color='#526477',**kw):
            return c.create_line(*[v for p in points for v in point(*p)],fill=color,width=2,**kw)
        def label(x,y,text,color='#233a51',size=11):
            c.create_text(*point(x,y),text=text,fill=color,font=('Microsoft YaHei UI',size),justify='center')
        def arc(x,y,r,start,end,color):
            pts=[(x+r*math.cos(start+(end-start)*i/30),y+r*math.sin(start+(end-start)*i/30)) for i in range(31)]
            line(pts,color,arrow='last')
        p,yaw,q1,q2=[math.radians(self.values[t].get()) for t in ('pitch','yaw','qH','qK')]
        label(245,25,'串联：pitch / 髋角 / 膝角' if self.serial else '并联：pitch / 两个主动髋角',size=14)
        label(755,25,'俯视：yaw（绕竖直 Z 轴）',size=14)
        line([(500,15),(500,445)],'#cad4df')
        hip=(235,175)
        # Body forward and down basis, positive pitch = nose down.
        def local(x,y):return (hip[0]+x*math.cos(p)-y*math.sin(p),hip[1]+x*math.sin(p)+y*math.cos(p))
        line([local(-90,0),local(90,0)],'#168aad',arrow='last')
        line([(135,175),(350,175)],'#9cabbc',dash=(5,4))
        line([(235,175),(235,65)],'#9cabbc',dash=(5,4))
        line([hip,local(0,-100)],'#b64074',arrow='last')
        arc(*hip,65,-math.pi/2,-math.pi/2+p,'#b64074')
        label(365,115,f'pitch = {math.degrees(p):.0f}°\n机身前后倾斜','#b64074')
        label(100,90,'竖直参考线\npitch = 0')
        label(380,190,'车头 / +X →','#168aad')
        a,b=95,135
        if not self.serial:
            self.draw_five_bar(line,label,arc,local,hip,p,q1,q2)
        else:
            self.draw_serial(line,label,arc,local,hip,p,q1,q2)
        center=(755,245)
        line([center,(925,245)],'#9cabbc',arrow='last');label(895,277,'世界 +X / yaw=0')
        line([center,(755,90)],'#9cabbc',arrow='last');label(690,92,'世界 +Y（左）')
        def top(x,y):return (755+x*math.cos(yaw)-y*math.sin(yaw),245-x*math.sin(yaw)-y*math.cos(yaw))
        corners=[top(-65,-35),top(65,-35),top(65,35),top(-65,35),top(-65,-35)]
        line(corners,'#168aad');line([center,top(115,0)],'#168aad',arrow='last')
        arc(*center,85,0,-yaw,'#7958b1')
        label(830,365,f'yaw = {math.degrees(yaw):.0f}°\n世界 +X → 车头方向；左转为正','#7958b1')
        label(700,310,'⊙ +Z 向屏幕外')
        label(750,420,'pitch：绕左右轴 Y 前后倾\nyaw：绕竖直轴 Z 转向\nroll：绕车头轴 X 左右倾')

    def draw_serial(self,line,label,arc,local,hip,p,q1,q2):
        a,b=95,135
        knee=local(a*math.sin(q1),a*math.cos(q1))
        wheel=local(a*math.sin(q1)+b*math.sin(q1+q2),a*math.cos(q1)+b*math.cos(q1+q2))
        line([hip,knee,wheel],'#e48a36')
        line([hip,local(0,95)],'#9cabbc',dash=(4,3))
        down=math.pi/2+p
        arc(*hip,35,down,down-q1,'#168aad')
        line([knee,(knee[0]+65*math.sin(q1-p),knee[1]+65*math.cos(q1-p))],'#9cabbc',dash=(4,3))
        arc(*knee,32,down-q1,down-q1-q2,'#bd6b23')
        line([hip,wheel],'#71849a',dash=(5,4))
        for pos,text in [(hip,'H 髋'),(knee,'K 膝'),(wheel,'W 轮轴')]:
            label(pos[0]-38,pos[1]-12,text)
        label(100,235,f'qH = {math.degrees(q1):.0f}°\n机身向下线 → 大腿','#168aad')
        label(360,305,f'qK = {math.degrees(q2):.0f}°\n大腿延长线 → 小腿','#bd6b23')
        label(255,415,'虚线 H—W = 虚拟腿 L\n关节角相对机身；pitch 相对世界竖直')
        theta=math.atan2(a*math.cos(q1)+b*math.cos(q1+q2),a*math.sin(q1)+b*math.sin(q1+q2))-math.pi/2-p
        label(500,495,f'theta = atan2(y, x) − π/2 − pitch = {math.degrees(theta):.1f}°（本示意几何）\n虚拟腿摆角 theta 与机身 pitch 不同；符号采用控制器公式，见“推导 → 控制输出”。',size=12)

    def draw_five_bar(self,line,label,arc,local,hip,p,q1,q4):
        import numpy as np
        a,b,d=90.,145.,80.
        A=np.array([-d/2,0]);E=np.array([d/2,0])
        B=A+a*np.array([math.cos(q1),math.sin(q1)])
        D=E+a*np.array([math.cos(q4),math.sin(q4)])
        v=D-B;n=np.linalg.norm(v);mid=(B+D)/2
        C=mid+math.sqrt(b*b-n*n/4)*np.array([-v[1],v[0]])/n
        line([local(*A),local(*B),local(*C)],'#168aad')
        line([local(*E),local(*D),local(*C)],'#e48a36')
        line([hip,local(*C)],'#71849a',dash=(5,4))
        for v,name in [(A,'A 主动'),(B,'B 被动'),(C,'C 轮轴'),(D,'D 被动'),(E,'E 主动')]:
            pt=local(*v);label(pt[0]+(-34 if v[0]<0 else 34),pt[1]+12,name,size=10)
        label(235,148,'O 髋中点',size=10)
        for v,q,color in [(A,q1,'#168aad'),(E,q4,'#bd6b23')]:
            pt=local(*v);line([pt,local(*(v+[60,0]))],'#9cabbc',dash=(4,3));arc(*pt,28,p,p+q,color)
        label(90,300,f'phi1 = {math.degrees(q1):.0f}°\n局部 +x → AB','#168aad')
        label(395,310,f'phi4 = {math.degrees(q4):.0f}°\n局部 +x → ED','#bd6b23')
        label(255,435,'A—B—C—D—E—A 闭链；B/D 无驱动电机\n虚线 O—C = 虚拟腿 L；两个角均从局部 +x 量起',size=10)
        theta=math.atan2(C[1],C[0])-math.pi/2-p
        label(500,495,f'theta = atan2(yC, xC−d/2) − π/2 − pitch = {math.degrees(theta):.1f}°\n并联腿输出后髋 A / 前髋 E 力矩；不是串联腿的髋 / 膝力矩。',size=12)


def add_control_explanation(book,serial=True):
    angles=AngleExplainer(book,serial);book.add(angles,text='串联角度：pitch / yaw / 髋膝' if serial else '并联角度：pitch / yaw / 双髋')
    page=ttk.Frame(book);book.add(page,text=('串联' if serial else '并联')+'推导 → 控制输出')
    text=tk.Text(page,wrap='word',font=('Microsoft YaHei UI',11),padx=24,pady=16,bg='#f4f7fb',fg='#233a51',relief='flat',spacing3=6)
    scroll=ttk.Scrollbar(page,command=text.yview);scroll.pack(side='right',fill='y')
    text.configure(yscrollcommand=scroll.set);text.pack(fill='both',expand=True)
    text.insert('1.0',CONTROL_TEXT if serial else parallel_control_text());text.configure(state='disabled')
    return angles
