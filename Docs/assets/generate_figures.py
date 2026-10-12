"""Generate the explanatory figures used by the parallel-leg tutorial."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, FancyArrowPatch, FancyBboxPatch

OUT = Path(__file__).resolve().parent
plt.rcParams.update({
    'font.family': ['Microsoft YaHei', 'DejaVu Sans'],
    'mathtext.fontset': 'stix',
    'axes.unicode_minus': False,
    'figure.facecolor': 'white',
})

BLUE, ORANGE, NAVY, RED, GREY, GREEN = '#168aad', '#f08c38', '#294762', '#c23b69', '#71849a', '#2b8a3e'


def save(fig, name):
    fig.savefig(OUT / name, dpi=180, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def readable_symbols(serial=False):
    """Keep every symbol off its rod, with leader lines and a separate key."""
    fig,(ax,key)=plt.subplots(1,2,figsize=(13,8),gridspec_kw={'width_ratios':[1.25,1]})
    ax.set_aspect('equal');ax.axis('off');key.axis('off')
    ax.set_xlim(-1.6,2.8);ax.set_ylim(3.3,-1.0)
    def call(text,point,label,color=NAVY):
        ax.annotate(text,xy=point,xytext=label,fontsize=14,color=color,ha='center',va='center',
                    bbox=dict(boxstyle='round,pad=.22',fc='white',ec='none',alpha=1),
                    arrowprops=dict(arrowstyle='-',color=GREY,lw=1.2),zorder=10)
    def arc(center,r,start,end,color):
        t=np.linspace(start,end,60);points=center+r*np.column_stack([np.cos(t),np.sin(t)])
        ax.plot(*points.T,color=color,lw=2,zorder=5)
        ax.annotate('',xy=points[-1],xytext=points[-5],arrowprops=dict(arrowstyle='->',color=color,lw=2))
        return points[len(points)//2]
    if serial:
        H=np.array([.5,0]);q1,q2=np.radians([-30,65]);a,b=1.,1.35
        K=H+a*np.array([np.sin(q1),np.cos(q1)]);W=K+b*np.array([np.sin(q1+q2),np.cos(q1+q2)])
        ax.plot(*np.vstack([H,K,W]).T,'-o',lw=6,color=BLUE,zorder=2)
        ax.plot(*np.vstack([H,W]).T,'--',color=RED,lw=2,zorder=1)
        ax.plot([H[0],H[0]],[0,.72],':',color=GREY)
        ext=K+.65*np.array([np.sin(q1),np.cos(q1)])
        ax.plot(*np.vstack([K,ext]).T,':',color=GREY)
        call('H：主动髋',H,(-.8,-.45));call('K：主动膝',K,(-1.0,1.05));call('W：轮轴',W,(1.65,2.5))
        call('$l_1$ 大腿长',(H+K)/2,(-.85,.35));call('$l_2$ 小腿长',(K+W)/2,(-.75,2.05))
        call('$L$ 虚拟腿长',(H+W)/2,(1.9,1.28),RED)
        call('$q_1$ 髋角',arc(H,.33,np.pi/2,np.pi/2-q1,BLUE),(-.65,-.1),BLUE)
        call('$q_2$ 相对膝角',arc(K,.34,np.pi/2-q1,np.pi/2-q1-q2,ORANGE),(1.8,.72),ORANGE)
        phi=np.arctan2(*(W-H)[::-1]);call('$\phi_0$',arc(H,.55,0,phi,RED),(1.8,.17),RED)
        origin=H;title='串联腿：髋—膝—轮轴'
        notes=[('机构点','H、K：两个主动关节\nW：轮电机轴心'),('结构参数','l₁ = |HK|，l₂ = |KW|\n固定杆长，不是控制命令'),('关节角','q₁：机身向下参考线 → HK\nq₂：HK 延长线 → KW\n箭头给出图中有向转角'),('虚拟腿','L = |HW|\nφ₀：局部 +x → HW\n虚线不是实体杆'),('机身姿态','φ = pitch：机身前后倾\nψ = yaw：俯视航向\n见独立侧视 / 俯视图')]
    else:
        A=np.array([0.,0]);E=np.array([1.2,0]);O=(A+E)/2
        a,b=1.1,1.5;q1,q4=np.radians([108,72]);B=A+a*np.array([np.cos(q1),np.sin(q1)]);D=E+a*np.array([np.cos(q4),np.sin(q4)])
        C=(B+D)/2+np.array([0,np.sqrt(b*b-np.linalg.norm(D-B)**2/4)])
        ax.plot(*np.vstack([A,B,C]).T,'-o',lw=6,color=BLUE);ax.plot(*np.vstack([E,D,C]).T,'-o',lw=6,color=ORANGE)
        ax.plot(*np.vstack([A,E]).T,lw=5,color=NAVY);ax.plot(*np.vstack([O,C]).T,'--',lw=2,color=RED)
        for pos,name,dest in [(A,'A 主动',(-.65,-.45)),(E,'E 主动',(1.85,-.45)),(O,'O 中点',(.6,-.78)),(B,'B 被动',(-1.,1.05)),(D,'D 被动',(2.2,1.05)),(C,'C 轮轴',(.6,2.72))]:call(name,pos,dest)
        for pos,name,dest in [((A+B)/2,'$l_1$',(-1.1,.48)),((E+D)/2,'$l_1$',(2.3,.48)),((B+C)/2,'$l_2$',(-.9,1.85)),((D+C)/2,'$l_2$',(2.1,1.85))]:call(name,pos,dest)
        ax.annotate('',xy=(1.2,-.24),xytext=(0,-.24),arrowprops=dict(arrowstyle='<->',color=GREY));call('$d$',(.85,-.24),(1.05,-.68))
        for pos,q,color,lab,dest in [(A,q1,BLUE,'$\phi_1$',(-.8,.02)),(E,q4,ORANGE,'$\phi_4$',(2.1,.02))]:
            ax.plot([pos[0],pos[0]+.5],[0,0],':',color=GREY);call(lab,arc(pos,.28,0,q,color),dest,color)
        call('$\phi_0$',arc(O,.43,0,np.pi/2,RED),(1.02,.65),RED)
        call('$L$',(O+C)/2,(.12,1.35),RED)
        origin=O;title='并联五连杆：闭链 A—B—C—D—E—A'
        notes=[('机构点','A、E：两个主动髋\nB、D：被动膝，无驱动电机\nO：两髋中点；C：轮轴'),('结构参数','l₁ = |AB| = |ED|\nl₂ = |BC| = |DC|；d = |AE|'),('主动杆角','φ₁：局部 +x → AB\nφ₄：局部 +x → ED\n两个角都不从竖直量起'),('虚拟腿','L = |OC|\nφ₀：局部 +x → OC\n虚线不是实体杆'),('机身姿态','φ = pitch：机身前后倾\nψ = yaw：俯视航向\n见独立侧视 / 俯视图')]
    # Detached axes avoid hiding O, d or the physical hip beam.
    ax.annotate('',xy=(2.4,2.95),xytext=(1.65,2.95),arrowprops=dict(arrowstyle='->',color=NAVY))
    ax.annotate('',xy=(1.65,3.28),xytext=(1.65,2.95),arrowprops=dict(arrowstyle='->',color=NAVY))
    ax.text(2.4,2.86,'+x 前',ha='center',fontsize=11);ax.text(1.48,3.2,'+y 下',ha='right',fontsize=11)
    for i,(heading,body) in enumerate(notes):
        y=.96-i*.195;key.text(.03,y,heading,fontsize=15,color=NAVY,weight='bold',va='top')
        key.text(.03,y-.05,body,fontsize=12,color='#263442',va='top',linespacing=1.55)
    fig.suptitle(title,fontsize=19,color=NAVY)
    save(fig,'07_serial_symbols.png' if serial else '01_five_bar_symbols.png')


def mechanism():
    readable_symbols()


def closure():
    B=np.array([0.,0.]);D=np.array([2.2,.35]);r=1.55
    mid=(B+D)/2;v=D-B;perp=np.array([-v[1],v[0]])/np.linalg.norm(v);h=np.sqrt(r*r-(np.linalg.norm(v)/2)**2)
    C1=mid+h*perp;C2=mid-h*perp
    fig,ax=plt.subplots(figsize=(9,5.4));t=np.linspace(0,2*np.pi,300)
    for P,color in [(B,BLUE),(D,ORANGE)]:ax.plot(P[0]+r*np.cos(t),P[1]+r*np.sin(t),'--',color=color,alpha=.65,lw=2)
    ax.plot(*np.vstack([B,D]).T,'-',color=GREY,lw=2);ax.plot(*np.vstack([B,C1,D]).T,'-o',color=GREEN,lw=4)
    ax.plot(*C2,'o',color=RED);ax.text(*(C2+[.08,-.08]),'other branch $C_-$',color=RED,fontsize=12)
    for P,n in [(B,'B'),(D,'D'),(C1,'selected $C_+$')]:ax.text(*(P+[.06,.12]),n,fontsize=13,weight='bold')
    ax.text(.35,.9,'$|BC|=l_2$',color=BLUE,fontsize=14);ax.text(1.5,1.08,'$|DC|=l_2$',color=ORANGE,fontsize=14)
    ax.text(1.1,.06,'$\Delta=D-B$',ha='center',fontsize=14,color=NAVY)
    ax.set_aspect('equal');ax.set_xlim(-1.8,4);ax.set_ylim(-1.85,2.2);ax.axis('off')
    ax.set_title('Closure is the intersection of two circles — the branch must be chosen explicitly',fontsize=14)
    save(fig,'02_closure_branches.png')


def jacobian():
    A=np.array([0.,0.]);E=np.array([1.2,0.]);B=np.array([-.25,1.25]);D=np.array([1.55,1.18]);C=np.array([.62,2.25]);O=(A+E)/2
    fig,ax=plt.subplots(figsize=(9,6))
    ax.plot(*np.vstack([A,B,C]).T,'-o',lw=4,color=BLUE);ax.plot(*np.vstack([E,D,C]).T,'-o',lw=4,color=ORANGE);ax.plot(*np.vstack([A,E]).T,lw=4,color=NAVY)
    def arrow(p,v,text,color,label_offset=(.04,.02)):
        q=p+v;ax.add_patch(FancyArrowPatch(p,q,arrowstyle='-|>',mutation_scale=15,lw=2.2,color=color));ax.text(*(q+label_offset),text,color=color,fontsize=13)
    arrow(B,np.array([-.58,-.12]),'$v_B=l_1\dot\phi_1 e_\perp(\phi_1)$',BLUE)
    arrow(D,np.array([.52,.15]),'$v_D=l_1\dot\phi_4 e_\perp(\phi_4)$',ORANGE)
    e0=(C-O)/np.linalg.norm(C-O);ep=np.array([-e0[1],e0[0]])
    arrow(C,.52*e0,'$\dot L e(\phi_0)$',GREEN,(.06,.12));arrow(C,.52*ep,'$L\dot\phi_0 e_\perp(\phi_0)$',RED,(-.05,-.14))
    ax.plot(*np.vstack([O,C]).T,'--',color=RED,lw=2)
    ax.text(.6,-.33,'Differentiate the rigid-link constraints, then solve the 2×2 linear system',fontsize=13,color=NAVY,ha='center')
    ax.set_aspect('equal');ax.set_xlim(-1.1,2.35);ax.set_ylim(3.08,-.55);ax.axis('off')
    save(fig,'03_velocity_jacobian.png')


def vmc():
    fig,ax=plt.subplots(figsize=(10,4.5));ax.axis('off');ax.set_xlim(0,10);ax.set_ylim(0,4.5)
    boxes=[(0.35,1.45,2.15,1.5,'Virtual commands\n$F$ [N], $T$ [N·m]',BLUE),
           (3.0,1.45,2.35,1.5,'Jacobian transpose\n$\tau=J^T[F,T]^T$',RED),
           (5.85,1.45,1.75,1.5,'Joint torque\n$\tau_1,\tau_4$',ORANGE),
           (8.05,1.45,1.55,1.5,'Motor current\n$I=\tau/(K_t i\eta)$',GREEN)]
    for x,y,w,h,label,color in boxes:
        ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=.04,rounding_size=.08',fc=color+'18',ec=color,lw=2))
        ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=12,color=NAVY)
    for x1,x2 in [(2.5,3.0),(5.35,5.85),(7.6,8.05)]:ax.annotate('',xy=(x2,2.2),xytext=(x1,2.2),arrowprops=dict(arrowstyle='->',lw=2,color=GREY))
    ax.text(5,3.85,'Instantaneous power / virtual-work principle',ha='center',fontsize=15,weight='bold',color=NAVY)
    ax.text(5,.55,'$F\dot L+T\dot\phi_0=\tau_1\dot\phi_1+\tau_4\dot\phi_4$',ha='center',fontsize=17,color=RED)
    save(fig,'04_vmc_power_mapping.png')


def pipeline():
    fig,ax=plt.subplots(figsize=(12,5));ax.axis('off');ax.set_xlim(0,12);ax.set_ylim(0,5)
    labels=[('Sensors','angles, rates, IMU'),('Kinematics','$L,\phi_0,J$'),('Controller','LQR + PID'),('VMC','$J^T[F,T]$'),('Safety','limit + finite'),('Actuators','6 torques')]
    colors=[NAVY,BLUE,RED,ORANGE,GREEN,NAVY]
    xs=np.linspace(.2,10.1,len(labels))
    for i,((title,sub),color,x) in enumerate(zip(labels,colors,xs)):
        ax.add_patch(FancyBboxPatch((x,2),1.65,1.15,boxstyle='round,pad=.04,rounding_size=.08',fc=color+'18',ec=color,lw=2))
        ax.text(x+.825,2.68,title,ha='center',fontsize=12,weight='bold',color=NAVY);ax.text(x+.825,2.3,sub,ha='center',fontsize=10,color=GREY)
        if i<len(labels)-1:ax.annotate('',xy=(xs[i+1],2.58),xytext=(x+1.65,2.58),arrowprops=dict(arrowstyle='->',lw=2,color=GREY))
    ax.annotate('',xy=(.95,1.93),xytext=(10.9,1.15),arrowprops=dict(arrowstyle='->',lw=1.8,color=BLUE,connectionstyle='arc3,rad=-.16'))
    ax.text(6,.65,'1 kHz feedback: measure → compute → saturate → output',ha='center',fontsize=14,color=NAVY)
    ax.text(6,4.25,'Embedded control-cycle data flow',ha='center',fontsize=16,weight='bold',color=NAVY)
    save(fig,'05_control_pipeline.png')


def singularity():
    fig,axes=plt.subplots(1,2,figsize=(10,4.5))
    postures=[(np.array([0.,0.]),np.array([1.2,0.]),np.array([-.2,1.1]),np.array([1.45,1.05]),np.array([.62,2.05]),'well-conditioned','$|\sin(\phi_3-\phi_2)|$ is large',GREEN),
              (np.array([0.,0.]),np.array([1.2,0.]),np.array([.1,1.1]),np.array([1.1,1.1]),np.array([.6,1.15]),'near singular',r'$\phi_3-\phi_2\approx\pi$',RED)]
    for ax,(A,E,B,D,C,title,desc,color) in zip(axes,postures):
        ax.plot(*np.vstack([A,B,C]).T,'-o',lw=4,color=BLUE);ax.plot(*np.vstack([E,D,C]).T,'-o',lw=4,color=ORANGE);ax.plot(*np.vstack([A,E]).T,lw=4,color=NAVY)
        ax.set_title(title,color=color,weight='bold');ax.text(.6,3.0,desc,ha='center',color=color,fontsize=12)
        ax.set_aspect('equal');ax.set_xlim(-.5,1.7);ax.set_ylim(3.2,-.4);ax.axis('off')
    fig.suptitle('The Jacobian denominator is a geometric safety condition, not just a numerical detail',fontsize=14,color=NAVY)
    save(fig,'06_singularity.png')


def serial_symbols():
    readable_symbols(True)


def serial_inverse():
    fig,ax=plt.subplots(figsize=(8,6));H=np.array([0.,0.]);l1=1.;l2=.72
    t=np.linspace(0,2*np.pi,400)
    for r,style in [(l1+l2,'-'),(abs(l1-l2),'--')]:ax.plot(r*np.cos(t),r*np.sin(t),style,color=GREY,lw=2)
    W=np.array([.45,1.25]);L=np.linalg.norm(W);c=(L*L-l1*l1-l2*l2)/(2*l1*l2);q2=np.arccos(c)
    alpha=np.arctan2(W[0],W[1])
    for sign,color,label in [(1,BLUE,'forward-bent branch'),(-1,ORANGE,'backward-bent branch')]:
        knee=sign*q2;hip=alpha-np.arctan2(l2*np.sin(knee),l1+l2*np.cos(knee));K=np.array([l1*np.sin(hip),l1*np.cos(hip)])
        ax.plot(*np.vstack([H,K,W]).T,'-o',lw=5,color=color,label=label)
    ax.plot(*W,'o',ms=10,color=RED);ax.text(*(W+[.08,.04]),'target $(x,y)$',fontsize=13)
    ax.text(0,-1.95,'Reachable annulus: $|l_1-l_2| < L < l_1+l_2$',ha='center',fontsize=14,color=NAVY)
    ax.set_aspect('equal');ax.set_xlim(-2,2);ax.set_ylim(2,-2.1);ax.axis('off');ax.legend(loc='upper right',frameon=False)
    save(fig,'08_serial_inverse_branches.png')


def serial_jacobian():
    H=np.array([0.,0.]);K=np.array([.72,1.05]);W=np.array([.28,2.05]);fig,ax=plt.subplots(figsize=(8,6))
    ax.plot(*np.vstack([H,K]).T,'-o',lw=7,color=BLUE);ax.plot(*np.vstack([K,W]).T,'-o',lw=7,color=ORANGE);ax.plot(*np.vstack([H,W]).T,'--',lw=2,color=RED)
    def arrow(p,v,text,color,offset=(.04,.02)):
        q=p+v;ax.add_patch(FancyArrowPatch(p,q,arrowstyle='-|>',mutation_scale=15,lw=2.2,color=color));ax.text(*(q+offset),text,color=color,fontsize=12)
    r=W;er=r/np.linalg.norm(r);et=np.array([-er[1],er[0]])
    arrow(W,.5*er,'$\dot L e_r$',GREEN,(.05,.12));arrow(W,.5*et,'$L\dot\phi_0 e_t$',RED,(-.15,-.12))
    upper=K-H;lower=W-K
    arrow(W,.78*np.array([-upper[1],upper[0]])/np.linalg.norm(upper),'$\partial r/\partial q_1$',BLUE,(-.38,.2))
    arrow(W,.68*np.array([-lower[1],lower[0]])/np.linalg.norm(lower),'$\partial r/\partial q_2$',ORANGE,(-.32,-.18))
    ax.text(.45,-.38,'Cartesian derivatives → polar projection → virtual-leg Jacobian',ha='center',fontsize=14,color=NAVY)
    ax.set_aspect('equal');ax.set_xlim(-1.2,1.7);ax.set_ylim(2.8,-.6);ax.axis('off')
    save(fig,'09_serial_jacobian.png')


def serial_singularity():
    fig,axes=plt.subplots(1,3,figsize=(12,4.2));cases=[(.9,'bent',r'$|\sin q_2|$ is large',GREEN),(.08,'extended',r'$q_2\approx0$',RED),(np.pi-.08,'folded',r'$q_2\approx\pi$',RED)]
    l1=1.;l2=.72
    for ax,(q2,title,desc,color) in zip(axes,cases):
        q1=-.45*q2;H=np.array([0.,0.]);K=np.array([l1*np.sin(q1),l1*np.cos(q1)]);W=K+np.array([l2*np.sin(q1+q2),l2*np.cos(q1+q2)])
        ax.plot(*np.vstack([H,K]).T,'-o',lw=6,color=BLUE);ax.plot(*np.vstack([K,W]).T,'-o',lw=6,color=ORANGE)
        ax.set_title(title,color=color,weight='bold');ax.text(0,1.93,desc,ha='center',color=color,fontsize=12)
        ax.set_aspect('equal');ax.set_xlim(-1.25,1.25);ax.set_ylim(2.08,-.35);ax.axis('off')
    fig.suptitle('Serial-leg singularities occur when the two links are collinear',fontsize=14,color=NAVY)
    save(fig,'10_serial_singularity.png')


def embedded_data_pipeline():
    fig, ax = plt.subplots(figsize=(15, 8.2))
    ax.axis('off'); ax.set_xlim(0, 15); ax.set_ylim(0, 8.2)
    ax.text(7.5, 7.82, '下位机一周期：原始数据 → 状态量 → 控制量 → CAN',
            ha='center', fontsize=19, weight='bold', color=NAVY)

    columns = [
        (0.25, '1  原始输入', [
            'DM：位置 / 速度 / 力矩', 'DJI：编码器 / RPM / 电流',
            'BMI088：陀螺仪 / 加速度', '遥控器、裁判系统、电容']),
        (3.25, '2  入口处理', [
            'CAN 解包与多圈计数', '零位、方向、减速比',
            '四元数 EKF 与去重力', '目标整形、加速度限幅']),
        (6.25, '3  状态估计', [
            '五连杆 Link2Leg', '轮速刚体运动补偿',
            '速度-加速度 Kalman', '支持力 / 离地估计']),
        (9.25, '4  控制计算', [
            '6 维状态误差', '腿长调度 LQR / MPC',
            '航向、抗劈叉、腿长 PID', '重力与横向惯性补偿']),
        (12.25, '5  执行输出', [
            'VMC：虚拟力 → 关节力矩', '符号、倍率、限幅、急停',
            'DM / DJI 协议打包', 'CAN 发送与反馈看门狗']),
    ]
    colors = [BLUE, '#3973b7', GREEN, ORANGE, RED]
    for i, ((x, title, lines), color) in enumerate(zip(columns, colors)):
        ax.add_patch(FancyBboxPatch((x, 2.0), 2.48, 4.85,
                     boxstyle='round,pad=.06,rounding_size=.12',
                     fc=color+'12', ec=color, lw=2.2))
        ax.text(x+1.24, 6.45, title, ha='center', fontsize=13.5,
                weight='bold', color=color)
        for j, line in enumerate(lines):
            yy = 5.65 - j*.88
            ax.add_patch(FancyBboxPatch((x+.17, yy-.31), 2.14, .58,
                         boxstyle='round,pad=.025,rounding_size=.05',
                         fc='white', ec='#ccd5df', lw=1))
            ax.text(x+1.24, yy, line, ha='center', va='center',
                    fontsize=10.5, color=NAVY)
        if i < len(columns)-1:
            ax.annotate('', xy=(x+3.0, 4.45), xytext=(x+2.48, 4.45),
                        arrowprops=dict(arrowstyle='-|>', lw=2.2, color=GREY))

    ax.add_patch(FancyBboxPatch((3.1, .55), 8.8, .82,
                 boxstyle='round,pad=.04,rounding_size=.08',
                 fc=NAVY+'10', ec=NAVY, lw=1.6))
    ax.text(7.5, .96,
            '关键边界：LQR 只接收整理后的状态；MotorSetRef 只写参考值；电机任务才负责最终 CAN 帧',
            ha='center', va='center', fontsize=12.5, color=NAVY, weight='bold')
    save(fig, '11_embedded_data_to_can.png')


def theory_engineering_comparison():
    fig, ax = plt.subplots(figsize=(14, 8.3))
    ax.axis('off'); ax.set_xlim(0, 14); ax.set_ylim(0, 8.3)
    ax.text(7, 7.85, '从理论模型到真实轮腿工程：公式没有消失，而是被三层工程逻辑包围',
            ha='center', fontsize=18, weight='bold', color=NAVY)

    layers = [
        (0.7, 5.45, 12.6, 1.65, '#e8f4f8', BLUE,
         'A  与理论严格对应的核心',
         '闭链几何  ·  速度雅可比  ·  VMC  $J^T$  ·  状态反馈  $u=Kx$  ·  腿长增益调度'),
        (1.45, 3.35, 11.1, 1.55, '#eef7ef', GREEN,
         'B  同一数学关系的离散与数值实现',
         '采样与差分  ·  低通 / Kalman  ·  atan2 与装配分支  ·  单位和方向  ·  限幅与奇异保护'),
        (2.2, 1.15, 9.6, 1.65, '#fff4e9', ORANGE,
         'C  理想推导之外的真实系统功能',
         'IMU 姿态估计  ·  航向 / 腿长 / 横滚 PID  ·  离地与跳跃状态机  ·  功率 / 看门狗 / CAN'),
    ]
    for x, y, w, h, fc, ec, title, detail in layers:
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                     boxstyle='round,pad=.06,rounding_size=.12',
                     fc=fc, ec=ec, lw=2.3))
        ax.text(x+w/2, y+h*.66, title, ha='center', va='center',
                fontsize=14, color=ec, weight='bold')
        ax.text(x+w/2, y+h*.30, detail, ha='center', va='center',
                fontsize=11.5, color=NAVY)
    ax.annotate('', xy=(7, 5.12), xytext=(7, 4.92),
                arrowprops=dict(arrowstyle='-|>', lw=2, color=GREY))
    ax.annotate('', xy=(7, 3.05), xytext=(7, 2.82),
                arrowprops=dict(arrowstyle='-|>', lw=2, color=GREY))
    ax.text(7, .45,
            '判断标准：改变 A 会改变控制原理；改变 B 会改变数值品质；改变 C 会改变实物可用性与安全性',
            ha='center', fontsize=12.5, color=RED, weight='bold')
    save(fig, '12_theory_vs_engineering.png')


def attitude_axes():
    fig,axes=plt.subplots(1,2,figsize=(12,5.5))
    for ax in axes:
        ax.set_aspect('equal');ax.set_xlim(-1.6,1.9);ax.set_ylim(-1.3,1.8);ax.axis('off')
    ax=axes[0];p=np.radians(22)
    ax.set_title('侧视：pitch = 机身前后倾角',fontsize=15,color=NAVY)
    ax.plot([0,0],[0,1.3],'--',color=GREY);ax.text(-.15,1.43,'世界竖直参考',ha='center',fontsize=11)
    up=np.array([np.sin(p),np.cos(p)]);front=np.array([np.cos(p),-np.sin(p)])
    ax.annotate('',xy=up*1.2,xytext=(0,0),arrowprops=dict(arrowstyle='->',lw=3,color=RED))
    ax.plot(*np.vstack([-front,front]).T,color=BLUE,lw=7)
    ax.text(1.1,-.4,'车头 →',color=BLUE,fontsize=12)
    ax.add_patch(Arc((0,0),1.3,1.3,theta1=68,theta2=90,color=RED,lw=2))
    ax.text(.6,.9,'pitch\n图示前倾',color=RED,fontsize=13)
    ax.text(.1,-.9,'绕左右轴 Y 旋转；不是髋角或腿摆角\n机身直立为零，代码输入符号经适配器转换',ha='center',fontsize=11)
    ax=axes[1];a=np.radians(35)
    ax.set_title('俯视：yaw = 车头航向角',fontsize=15,color=NAVY)
    for dest,label in [((1.5,0),'世界 +X / yaw=0'),((0,1.4),'世界 +Y（左）')]:
        ax.annotate('',xy=dest,xytext=(0,0),arrowprops=dict(arrowstyle='->',color=GREY,lw=2))
        ax.text(dest[0],dest[1]+.15,label,ha='center',fontsize=10)
    front=np.array([np.cos(a),np.sin(a)])
    ax.annotate('',xy=1.3*front,xytext=(0,0),arrowprops=dict(arrowstyle='->',lw=3,color=BLUE))
    ax.text(1.2,.85,'车头',color=BLUE,fontsize=12)
    ax.add_patch(Arc((0,0),1.5,1.5,theta1=0,theta2=35,color=RED,lw=2));ax.text(.87,.22,'yaw',color=RED,fontsize=13)
    ax.text(-.5,-.3,'⊙ +Z 向上\n朝观察者',fontsize=11,color=NAVY)
    ax.text(.1,-.9,'绕竖直 Z 轴旋转，俯视左转为正\nyaw 走独立转向环，不属于六状态 LQR',ha='center',fontsize=11)
    fig.suptitle('并联 / 串联共用机身姿态定义；关节角按各自机构图解释',fontsize=15,y=1.02)
    save(fig,'13_pitch_yaw_axes.png')


if __name__ == '__main__':
    attitude_axes()
    readable_symbols();closure();jacobian();vmc();pipeline();singularity()
    readable_symbols(True);serial_inverse();serial_jacobian();serial_singularity()
    embedded_data_pipeline()
    theory_engineering_comparison()
    print('generated 13 tutorial figures in', OUT)
