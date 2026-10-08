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


def mechanism():
    A=np.array([0.,0.]);E=np.array([1.2,0.]);B=np.array([-.28,1.28]);D=np.array([1.58,1.18]);C=np.array([.62,2.34]);O=(A+E)/2
    fig,ax=plt.subplots(figsize=(9,6))
    ax.plot(*np.vstack([A,B,C]).T,'-o',lw=5,color=BLUE)
    ax.plot(*np.vstack([E,D,C]).T,'-o',lw=5,color=ORANGE)
    ax.plot(*np.vstack([A,E]).T,lw=5,color=NAVY)
    ax.plot(*np.vstack([O,C]).T,'--',lw=2.5,color=RED)
    for p,n,off in [(A,'A',(0,-.18)),(B,'B',(-.18,.03)),(C,'C',(0,.14)),(D,'D',(.14,.03)),(E,'E',(0,-.18)),(O,'O',(.02,-.2))]:
        ax.text(*(p+off),n,fontsize=15,weight='bold',ha='center')
    ax.annotate('',xy=(1.1,-.35),xytext=(.6,-.35),arrowprops=dict(arrowstyle='->',lw=2,color=NAVY));ax.text(1.13,-.35,'$x$',va='center',fontsize=14)
    ax.annotate('',xy=(.6,.2),xytext=(.6,-.35),arrowprops=dict(arrowstyle='->',lw=2,color=NAVY));ax.text(.68,.28,'$y$ (down)',ha='left',fontsize=13)
    ax.annotate('$d$',xy=(.6,.03),ha='center',va='bottom',fontsize=14)
    ax.text(-.24,.65,'$l_1$',fontsize=15,color=BLUE);ax.text(.08,1.82,'$l_2$',fontsize=15,color=BLUE)
    ax.text(1.32,.58,'$l_1$',fontsize=15,color=ORANGE);ax.text(1.12,1.78,'$l_2$',fontsize=15,color=ORANGE)
    ax.text(.49,1.25,'$L$',fontsize=15,color=RED)
    for center,start,end,label,pos,color in [(A,0,103,'$\phi_1$',(-.14,.34),BLUE),(E,77,180,'$\phi_4$',(.98,.34),ORANGE),(O,0,75,'$\phi_0$',(.83,.45),RED)]:
        ax.add_patch(Arc(center,.48,.48,theta1=start,theta2=end,color=color,lw=2));ax.text(*pos,label,fontsize=14,color=color)
    ax.text(.02,2.72,'Code coordinate system: origin A, +x forward, +y downward',fontsize=11,color=GREY)
    ax.set_aspect('equal');ax.set_xlim(-.75,2.0);ax.set_ylim(2.9,-.65);ax.axis('off')
    save(fig,'01_five_bar_symbols.png')


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
    H=np.array([0.,0.]);K=np.array([.72,1.05]);W=np.array([.28,2.05])
    fig,ax=plt.subplots(figsize=(7.5,6))
    ax.plot(*np.vstack([H,K]).T,'-o',lw=7,color=BLUE);ax.plot(*np.vstack([K,W]).T,'-o',lw=7,color=ORANGE)
    ax.plot(*np.vstack([H,W]).T,'--',lw=2.5,color=RED)
    for p,n,off in [(H,'H / hip',(-.15,-.14)),(K,'K / knee',(.14,0)),(W,'W / wheel',(.05,.16))]:ax.text(*(p+off),n,fontsize=13,weight='bold')
    ax.annotate('',xy=(1.05,-.28),xytext=(0,-.28),arrowprops=dict(arrowstyle='->',lw=2,color=NAVY));ax.text(1.08,-.28,'$x$ forward',va='center',fontsize=12)
    ax.annotate('',xy=(0,.45),xytext=(0,-.28),arrowprops=dict(arrowstyle='->',lw=2,color=NAVY));ax.text(-.08,.46,'$y$ down',ha='right',fontsize=12)
    ax.text(.42,.56,'$l_1$',fontsize=16,color=BLUE);ax.text(.58,1.58,'$l_2$',fontsize=16,color=ORANGE);ax.text(-.02,1.04,'$L$',fontsize=16,color=RED)
    ax.add_patch(Arc(H,.52,.52,theta1=52,theta2=90,color=BLUE,lw=2));ax.text(.48,.42,'$q_1$',fontsize=15,color=BLUE)
    ax.add_patch(Arc(K,.48,.48,theta1=95,theta2=155,color=ORANGE,lw=2));ax.text(.78,1.32,'$q_2$ relative',fontsize=14,color=ORANGE)
    ax.add_patch(Arc(H,.8,.8,theta1=0,theta2=74,color=RED,lw=2));ax.text(.44,.14,'$\phi_0$',fontsize=15,color=RED)
    ax.text(.05,2.48,'Convention: $q_1=0$ points down; $q_2$ is relative to the upper link',fontsize=12,color=GREY)
    ax.set_aspect('equal');ax.set_xlim(-.65,1.65);ax.set_ylim(2.65,-.55);ax.axis('off')
    save(fig,'07_serial_symbols.png')


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


if __name__ == '__main__':
    mechanism();closure();jacobian();vmc();pipeline();singularity()
    serial_symbols();serial_inverse();serial_jacobian();serial_singularity()
    embedded_data_pipeline()
    theory_engineering_comparison()
    print('generated 12 tutorial figures in', OUT)
