#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
忠实复刻 ug_ekf_core (eskf.cpp) 的 Python 离线 ESKF，用于在 baseline bag 上
快速迭代参数（门控/噪声），无需重编 C++ 或重录仿真。

约定与 C++ 完全一致：NED-FRD，q_NB (body→NED)，error state 15 维，
g_NED=[0,0,+g]，dv/dt=R(a_B-b_a)+g_NED，q_{k+1}=q_k⊗Exp((ω-b_g)dt)。

用法: python3 offline_eskf.py <baseline.bag> [--no-gyro-gate] [--tilt-gyro-gate 0.5]
"""
import argparse, math, numpy as np, sys
import rosbag
import tf.transformations as tft

D = np.diag([1.0, -1.0, -1.0])   # FLU<->FRD
G = 9.81

def hat(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0.0]])

def expq(phi):
    th = np.linalg.norm(phi)
    if th < 1e-8:
        q = np.array([0.5*phi[0],0.5*phi[1],0.5*phi[2],1.0]);
        return q/np.linalg.norm(q)
    half=0.5*th; s=math.sin(half)/th
    return np.array([phi[0]*s,phi[1]*s,phi[2]*s,math.cos(half)])

def qmul(a,b):  # xyzw
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return np.array([
        aw*bx+ax*bw+ay*bz-az*by,
        aw*by-ax*bz+ay*bw+az*bx,
        aw*bz+ax*by-ay*bx+az*bw,
        aw*bw-ax*bx-ay*by-az*bz])

def qnorm(q): return q/np.linalg.norm(q)
def R_of(q):  return tft.quaternion_matrix(q)[:3,:3]

class ESKF:
    def __init__(s, p):
        s.p=np.zeros(3); s.v=np.zeros(3); s.q=np.array([0,0,0,1.0])
        s.bg=np.zeros(3); s.ba=np.zeros(3)
        s.P=np.zeros((15,15)); s.par=p; s.tprev=None
        s.acc_depth=0; s.rej_depth=0; s.acc_tilt=0; s.rej_tilt=0; s.reset=0
        s.acc_mag=0; s.rej_mag=0

    def setP0(s):
        d=np.zeros(15)
        d[0:3]=s.par['p0_pos']; d[3:6]=s.par['p0_vel']; d[6:9]=s.par['p0_att']
        d[9:12]=s.par['p0_bg']; d[12:15]=s.par['p0_ba']
        s.P=np.diag(d)

    def static_align(s, accs, gyros, mags=None):
        am=np.mean(accs,axis=0); gm=np.mean(gyros,axis=0)
        yaw=s.par['init_yaw']
        if mags is not None and len(mags):
            mm=np.mean(mags,axis=0)
            if np.linalg.norm(mm)>1e-12:
                yaw=s.par['decl']+math.atan2(-mm[1],mm[0])
        s.q=qnorm(np.array([0,0,math.sin(yaw/2),math.cos(yaw/2)]))
        s.bg=gm.copy()
        s.ba=am-np.array([0,0,-s.par['g']])

    def predict(s,t,gyro,acc):
        if s.tprev is None:
            s.tprev=t; return
        dt=t-s.tprev
        if dt<=0: return
        if dt>s.par['dt_max']:
            s.reset+=1; s.tprev=t; return
        w=gyro-s.bg; a=acc-s.ba
        R=R_of(s.q); gN=np.array([0,0,s.par['g']])
        aN=R@a+gN
        # F
        F=np.eye(15)
        F[0:3,3:6]=np.eye(3)*dt
        F[3:6,6:9]=-R@hat(a)*dt
        F[3:6,12:15]=-R*dt
        F[6:9,6:9]=np.eye(3)-hat(w)*dt
        F[6:9,9:12]=-np.eye(3)*dt
        # Q
        Q=np.zeros((15,15))
        n=s.par['noise']
        Q[3:6,3:6]=R@(n['a']**2*dt*np.eye(3))@R.T
        Q[6:9,6:9]=np.eye(3)*(n['g']**2*dt)
        Q[9:12,9:12]=np.eye(3)*(n['bg']**2*dt)
        Q[12:15,12:15]=np.eye(3)*(n['ba']**2*dt)
        # nominal
        s.p=s.p+s.v*dt+0.5*aN*dt*dt
        s.v=s.v+aN*dt
        s.q=qnorm(qmul(s.q,expq(w*dt)))
        s.P=F@s.P@F.T+Q
        s.P=0.5*(s.P+s.P.T)
        s.tprev=t

    def inject(s,dx):
        s.p+=dx[0:3]; s.v+=dx[3:6]
        s.q=qnorm(qmul(s.q,expq(dx[6:9])))
        s.bg+=dx[9:12]; s.ba+=dx[12:15]

    def update_depth(s,depth,R_d):
        rp=s.par['r_p']; Rm=R_of(s.q)
        zpred=(s.p+Rm@rp)[2]
        H=np.zeros((1,15)); H[0,2]=1
        Rhr=Rm@hat(rp); H[0,6:9]=-Rhr[2,:]
        y=depth-zpred
        S=(H@s.P@H.T)[0,0]+R_d
        if S<=0: s.rej_depth+=1; return
        nis=y*y/S
        if nis>s.par['nis_depth']: s.rej_depth+=1; return
        K=(s.P@H.T/S).reshape(15)
        s.inject(K*y)
        I=np.eye(15); IKH=I-np.outer(K,H.reshape(15))
        s.P=IKH@s.P@IKH.T+np.outer(K,K)*R_d
        s.P=0.5*(s.P+s.P.T); s.acc_depth+=1

    def update_tilt(s,acc,R_t):
        Rm=R_of(s.q); negg=np.array([0,0,-s.par['g']])
        hgrav=Rm.T@negg
        h=hgrav+s.ba
        H=np.zeros((3,15)); H[:,6:9]=hat(hgrav); H[:,12:15]=np.eye(3)
        y=acc-h
        S=H@s.P@H.T+np.eye(3)*R_t
        nis=y@np.linalg.solve(S,y)
        if nis>s.par['nis_tilt']: s.rej_tilt+=1; return
        K=s.P@H.T@np.linalg.inv(S)
        s.inject(K@y)
        I=np.eye(15); IKH=I-K@H
        s.P=IKH@s.P@IKH.T+(K*R_t)@K.T
        s.P=0.5*(s.P+s.P.T); s.acc_tilt+=1

    def update_mag(s,mag,R_y):
        if np.linalg.norm(mag)<1e-12: s.rej_mag+=1; return
        Rm=R_of(s.q)
        mw=Rm@mag
        decl=s.par['decl']
        def wrap(a):
            while a>math.pi: a-=2*math.pi
            while a<-math.pi: a+=2*math.pi
            return a
        y=wrap(decl-math.atan2(mw[1],mw[0]))
        H=np.zeros((1,15)); H[0,6:9]=Rm[2,:]
        S=(H@s.P@H.T)[0,0]+R_y
        if S<=0: s.rej_mag+=1; return
        nis=y*y/S
        if nis>s.par['nis_mag']: s.rej_mag+=1; return
        K=(s.P@H.T/S).reshape(15)
        s.inject(K*y)
        I=np.eye(15); IKH=I-np.outer(K,H.reshape(15))
        s.P=IKH@s.P@IKH.T+np.outer(K,K)*R_y
        s.P=0.5*(s.P+s.P.T); s.acc_mag+=1

def ned_rp(q):
    R=R_of(q)
    pitch=math.asin(max(-1,min(1,-R[2,0])))
    roll=math.atan2(R[2,1],R[2,2])
    return math.degrees(roll),math.degrees(pitch)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--tilt-gyro-gate',type=float,default=0.05)
    ap.add_argument('--tilt-acc-gate',type=float,default=0.5)
    ap.add_argument('--align-sec',type=float,default=8.0)
    ap.add_argument('--p0-bg',type=float,default=1e-4)
    ap.add_argument('--sigma-bg',type=float,default=1e-5)
    args=ap.parse_args()

    par=dict(g=G,dt_max=0.1,nis_depth=6.635,nis_tilt=11.345,nis_mag=6.635,
             init_yaw=0.0,decl=0.0,r_p=np.array([0,0,-0.1]),
             p0_pos=100.,p0_vel=0.01,p0_att=0.01,p0_bg=args.p0_bg,p0_ba=1e-3,
             noise=dict(g=1e-3,a=1e-2,bg=args.sigma_bg,ba=1e-4))
    R_depth=0.01; R_tilt=0.25; R_mag=0.01

    b=rosbag.Bag(args.bag)
    msgs=[]
    Rwn=np.array([[0,1,0],[1,0,0],[0,0,-1.0]]); Tb=np.diag([1,-1,-1.0])
    gt=[]  # (t, pitch_deg, depth, yaw_deg)
    def ned_rpy_gt(o):
        R=Rwn@R_of([o.x,o.y,o.z,o.w])@Tb
        pit=math.degrees(math.asin(max(-1,min(1,-R[2,0]))))
        yaw=math.degrees(math.atan2(R[1,0],R[0,0]))
        return pit,yaw
    for topic,m,_ in b.read_messages(topics=['/ug_glider/imu','/ug_glider/pressure',
                                             '/ug_glider/mag','/ug_glider/ground_truth/pose']):
        t=m.header.stamp.to_sec()
        if topic=='/ug_glider/imu':
            g=D@np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
            a=D@np.array([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
            msgs.append((t,'imu',g,a))
        elif topic=='/ug_glider/pressure':
            msgs.append((t,'pre',(m.fluid_pressure-101325.0)/(1028.0*G),None))
        elif topic=='/ug_glider/mag':
            mg=D@np.array([m.magnetic_field.x,m.magnetic_field.y,m.magnetic_field.z])
            msgs.append((t,'mag',mg,None))
        else:
            pit,yaw=ned_rpy_gt(m.pose.pose.orientation)
            gt.append((t,pit,-m.pose.pose.position.z,yaw))
    b.close()
    msgs.sort(key=lambda x:x[0])
    gtt=np.array([x[0] for x in gt])

    def wrapdeg(a):
        return (a+180)%360-180

    ekf=ESKF(par); ekf.setP0()
    aligned=False; t0=msgs[0][0]; accbuf=[]; gybuf=[]; magbuf=[]
    pitch_err=[]; depth_err=[]; yaw_err=[]; samples=[]
    for t,typ,d1,d2 in msgs:
        if not aligned:
            if typ=='imu': accbuf.append(d2); gybuf.append(d1)
            elif typ=='mag': magbuf.append(d1)
            if typ=='imu' and t-t0>=args.align_sec:
                ekf.static_align(np.array(accbuf),np.array(gybuf),
                                 np.array(magbuf) if magbuf else None)
                ekf.tprev=t; aligned=True
            continue
        if typ=='imu':
            gyro,acc=d1,d2
            ekf.predict(t,gyro,acc)
            if abs(np.linalg.norm(acc)-G)<args.tilt_acc_gate and np.linalg.norm(gyro)<args.tilt_gyro_gate:
                ekf.update_tilt(acc,R_tilt)
            i=np.argmin(np.abs(gtt-t))
            if abs(gtt[i]-t)<0.1:
                r,p=ned_rp(ekf.q); R=R_of(ekf.q)
                yaw=math.degrees(math.atan2(R[1,0],R[0,0]))
                pitch_err.append(p-gt[i][1])
                depth_err.append(ekf.p[2]-gt[i][2])
                yaw_err.append(wrapdeg(yaw-gt[i][3]))
                samples.append((t-t0,gt[i][3],yaw,gt[i][1],p))
        elif typ=='pre':
            ekf.update_depth(d1,R_depth)
        elif typ=='mag':
            ekf.update_mag(d1,R_mag)

    pe=np.array(pitch_err); de=np.array(depth_err); ye=np.array(yaw_err)
    print("tilt acc=%d rej=%d | depth acc=%d rej=%d | mag acc=%d rej=%d"%(
        ekf.acc_tilt,ekf.rej_tilt,ekf.acc_depth,ekf.rej_depth,ekf.acc_mag,ekf.rej_mag))
    print("  pitch RMSE=%.2f°  depth RMSE=%.3fm  yaw RMSE=%.2f°  b_g.y=%.4f rad/s(%.2f°/s)"%(
        math.sqrt(np.mean(pe**2)), math.sqrt(np.mean(de**2)), math.sqrt(np.mean(ye**2)),
        ekf.bg[1], math.degrees(ekf.bg[1])))
    print("  yaw 时序 (rel, GTyaw, ESyaw, GTpit, ESpit):")
    for k in range(0,len(samples),max(1,len(samples)//10)):
        s=samples[k]
        print("   %5.0f  %7.1f %7.1f  %6.1f %6.1f"%(s[0],s[1],s[2],s[3],s[4]))

if __name__=='__main__':
    main()
