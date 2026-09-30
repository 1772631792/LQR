"""Views for variable-height reduced chassis data; all plotting is postprocessing."""
import numpy as np
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from simulation.robot_view import geometry as five_bar_geometry,draw_2d,REAR,FRONT,HIP
from simulation.serial_leg_view import geometry as serial_geometry
from config.leg_params import LEGS
from config.robot_params import PARAMS


def curves(figure,result,legs=False,robot_config=None):
    figure.clear()
    axes=figure.subplots(3,2,sharex=True)
    t=result['time'];s=result['states'];u=result['outputs'];ref=result['references']
    stride=max(1,len(t)//3000)
    if not legs:
        values=[(s[:,0],ref[:,0],'Position [m]'),(s[:,1],None,'Velocity [m/s]'),
                (np.rad2deg(s[:,2]),np.zeros(len(t)),'Pitch [deg]'),
                (np.rad2deg(s[:,6]),np.rad2deg(ref[:,2]),'Roll [deg]'),
                (np.rad2deg(s[:,8]),np.rad2deg(ref[:,3]),'Yaw [deg]'),
                (np.append(u[:,0],np.nan),None,'Drive force [N]')]
    else:
        serial=(robot_config or {}).get('leg_topology')=='serial'
        joint='joint torques hip/knee' if serial else 'hip torques A/E'
        values=[(s[:,4]+.18*s[:,6],ref[:,1]+.18*ref[:,2],'Left leg [m]'),
                (s[:,4]-.18*s[:,6],ref[:,1]-.18*ref[:,2],'Right leg [m]'),
                (np.append(u[:,1],np.nan),np.append(u[:,2],np.nan),'Support forces L/R [N]'),
                (np.append(u[:,4],np.nan),np.append(u[:,5],np.nan),'Wheel torques L/R [Nm]'),
                (np.append(u[:,6],np.nan),np.append(u[:,7],np.nan),f'Left {joint} [Nm]'),
                (np.append(u[:,8],np.nan),np.append(u[:,9],np.nan),f'Right {joint} [Nm]')]
    cursors=[]
    for ax,(value,target,label) in zip(axes.flat,values):
        ax.plot(t[::stride],value[::stride],color=REAR,lw=1.4,label='actual / first')
        if target is not None: ax.plot(t[::stride],target[::stride],color=FRONT,lw=1,ls='--',label='reference / second')
        ax.set_ylabel(label);ax.grid(alpha=.2)
        cursors.append(ax.axvline(0,color='#a83257',lw=1))
        if target is not None: ax.legend(fontsize=7,loc='best')
    for ax in axes[-1]: ax.set_xlabel('Simulation time [s]')
    figure.tight_layout()
    return cursors


def _serial_config(config):
    config=config or {}
    return config.get('thigh_length_m',.135),config.get('calf_length_m',.24)


def _draw_serial_2d(ax,state,height,xlimits,config):
    ax.clear();p,_,pitch,_=state;upper,lower=_serial_config(config)
    points,body,com,angles=serial_geometry(p,pitch,height,upper,lower)
    from matplotlib.patches import Polygon
    ax.set(xlim=xlimits,ylim=(-.025,.47),xlabel='Forward x [m]',ylabel='Height z [m]');ax.set_aspect('equal')
    ax.axhline(0,color=HIP,lw=2);ax.add_patch(Polygon(body,facecolor='#dbe7ee',edgecolor=HIP,lw=2,zorder=2))
    ax.plot(points[:,0],points[:,1],'-o',color=REAR,lw=5,ms=7,zorder=4)
    axle=points[2];ax.add_patch(Circle(axle,PARAMS.wheel_radius,fc='#343e4c',ec='#17212e',lw=2,zorder=5))
    ax.add_patch(Circle(axle,PARAMS.wheel_radius*.65,fc='#d2dce5',ec='white',lw=1,zorder=6))
    for index,label in enumerate(('H','K','W')):ax.annotate(label,points[index],xytext=(7,8),textcoords='offset points',weight='bold')
    ax.plot(*com,'x',ms=9,mew=2,color='#a83257',label='Body COM');ax.grid(alpha=.18);ax.legend(loc='upper right',fontsize=8)
    ax.text(.02,.97,f'qHip={np.rad2deg(angles[0]):.1f} deg\nqKnee={np.rad2deg(angles[1]):.1f} deg',transform=ax.transAxes,va='top',fontsize=9)


def view2d(axes,state,xlimits,robot_config=None):
    serial=(robot_config or {}).get('leg_topology')=='serial'
    for ax,sign,name in zip(axes,(1,-1),('Left five-bar','Right five-bar')):
        height=state[4]+sign*.18*state[6]
        if serial:_draw_serial_2d(ax,state[:4],height,xlimits,robot_config)
        else:draw_2d(ax,state[:4],height,xlimits)
        name=('Left serial leg' if sign==1 else 'Right serial leg') if serial else name
        ax.set_title(f'{name} | h={state[4]+sign*.18*state[6]:.3f} m',fontsize=11)


def view3d(ax,state,xlimits,robot_config=None):
    elev,azim=ax.elev,ax.azim;ax.clear();ax.view_init(elev=elev,azim=azim)
    p,_,pitch,_,height,_,roll,_,yaw,_=state
    cy,sy=np.cos(yaw),np.sin(yaw)
    def transform(points,y):
        points=np.asarray(points)
        dx=points[:,0]-p
        return np.column_stack([p+cy*dx-sy*y,sy*dx+cy*y,points[:,1]])
    serial=(robot_config or {}).get('leg_topology')=='serial';upper,lower=_serial_config(robot_config)
    geom=(lambda position,pitch,height:serial_geometry(position,pitch,height,upper,lower)) if serial else five_bar_geometry
    vertices=[]
    for y in (-LEGS.body_width/2,LEGS.body_width/2):
        _,body,_,_=geom(p,pitch,height+y*roll)
        vertices.extend(transform(body,y))
    vertices=np.array(vertices)
    faces=[vertices[ids] for ids in ([0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7])]
    ax.add_collection3d(Poly3DCollection(faces,facecolor='#d1dfe8',edgecolor=HIP,alpha=.65))
    for y in (-.18,.18):
        points,_,_,_=geom(p,pitch,height+y*roll)
        xyz=transform(points,y)
        segments=[([0,1,2],REAR)] if serial else [([0,1,2],REAR),([4,3,2],FRONT),([0,4],HIP)]
        for ids,color in segments:
            ax.plot(*xyz[ids].T,'-o',color=color,lw=4,ms=4)
        for index in ((0,) if serial else (0,4)):
            # Short transverse motor shaft to chassis side.
            innerpoints,_,_,_=geom(p,pitch,height+np.sign(y)*LEGS.body_width/2*roll)
            inner=transform([innerpoints[index]],np.sign(y)*LEGS.body_width/2)[0]
            ax.plot(*np.vstack([inner,xyz[index]]).T,color=HIP,lw=4)
        angle=np.linspace(0,2*np.pi,36)
        circle=np.column_stack([p+PARAMS.wheel_radius*np.cos(angle),PARAMS.wheel_radius*(1+np.sin(angle))])
        rims=[transform(circle,y+offset) for offset in (-.02,.02)]
        for rim in rims: ax.plot(*rim.T,color='#263343',lw=2)
        ax.plot_surface(np.array([r[:,0] for r in rims]),np.array([r[:,1] for r in rims]),np.array([r[:,2] for r in rims]),color='#46566a')
    ax.set(xlim=xlimits,ylim=(-.4,.4),zlim=(0,.48),xlabel='x [m]',ylabel='y [m]',zlabel='z [m]')
    ax.set_box_aspect((xlimits[1]-xlimits[0],.8,.48))
    topology='Serial-leg chassis' if serial else 'Five-bar chassis'
    ax.set_title(f'{topology} | pitch {np.rad2deg(pitch):+.1f} deg | yaw {np.rad2deg(yaw):+.1f} deg')
