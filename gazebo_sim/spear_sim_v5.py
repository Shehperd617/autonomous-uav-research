#!/usr/bin/env python3
import numpy as np, subprocess, signal, time, sys, re

WAYPOINTS = [
    (50,20,20),(30,10,18),(10,5,16),
    (0,20,18),(-10,-5,20),(-20,15,15),
    (-15,5,10),(-5,10,22),(10,15,17),
]
TGT_SPD=5.0; TGT_SPD_FAST=8.0; WP_THR=4.0
HIT_DIST=2.5; HZ=10; N_PN=4.0; INT_SPD=12.0
TERMINAL_R=8.0
USE_PN = "--pursuit" not in sys.argv

def pub(topic,x,y,z):
    try:
        subprocess.Popen(['ign','topic','-t',topic,'-m',
            'ignition.msgs.Twist','-p',
            f'linear: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}'],
            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    except:
        pass

def poses():
    try:
        r=subprocess.run(['ign','topic','-t',
            '/world/spear_intercept/dynamic_pose/info',
            '-e','-n','1'],timeout=3,capture_output=True,text=True)
        return pm(r.stdout,"interceptor"),pm(r.stdout,"target")
    except:
        return None,None

def pm(t,n):
    m=re.search(rf'name: "{n}".*?position\s*\{{(.*?)\}}',t,re.DOTALL)
    if not m: return None
    b=m.group(1)
    gv=lambda k: float(re.search(rf'{k}:\s*([-\d.e+]+)',b).group(1)) if re.search(rf'{k}:\s*([-\d.e+]+)',b) else 0.0
    return np.array([gv('x'),gv('y'),gv('z')])

class Nav:
    def __init__(self):
        self.plos=None; self.pR=None; self.pt=None
        self.Vc=0; self.R=0; self.tgo=999; self.zem=0
    def go(self,ip,tp,t):
        los=tp-ip; self.R=np.linalg.norm(los)
        if self.R<0.1: return np.zeros(3)
        lh=los/self.R
        if self.R<TERMINAL_R:
            self.plos=lh.copy(); self.pR=self.R; self.pt=t
            return lh*INT_SPD
        if self.plos is not None and self.pt is not None:
            dt=t-self.pt
            if dt>0.01:
                dlos=(lh-self.plos)/dt
                self.Vc=-(self.R-self.pR)/dt if self.pR else 0
                vc=max(self.Vc,3.0)
                self.tgo=self.R/vc
                lrp=dlos-np.dot(dlos,lh)*lh
                self.zem=self.R*np.linalg.norm(lrp)*self.tgo
                a=N_PN*vc*lrp
                v=lh*INT_SPD+a*min(self.tgo,2.0)
                s=np.linalg.norm(v)
                if s>INT_SPD*1.3: v=v/s*INT_SPD*1.3
                self.plos=lh.copy(); self.pR=self.R; self.pt=t
                return v
        self.plos=lh.copy(); self.pR=self.R; self.pt=t
        return lh*INT_SPD
    def reset(self):
        self.plos=None; self.pR=None; self.pt=None

def main():
    mode="PRONAV" if USE_PN else "PURSUIT"
    print(f"===== SPEAR v5 {mode} =====")
    nav=Nav()
    print("Connecting...",end='',flush=True)
    for _ in range(20):
        ip,tp=poses()
        if ip is not None: print("OK!"); break
        print(".",end='',flush=True); time.sleep(1)
    else: print("FAIL"); return
    wp=0; hits=0; t0=time.time(); dt=1.0/HZ; cd=0
    run=[True]; signal.signal(signal.SIGINT,lambda s,f:run.__setitem__(0,False))
    lp=0
    while run[0]:
        tt=time.time(); el=tt-t0
        ip,tp=poses()
        if ip is None or tp is None: time.sleep(dt); continue
        if wp<len(WAYPOINTS):
            w=np.array(WAYPOINTS[wp]); d=w-tp; dn=np.linalg.norm(d)
            if dn<WP_THR:
                wp=(wp+1)%len(WAYPOINTS)
                print(f"  [TGT] WP{wp}/{len(WAYPOINTS)}")
                continue
            s=TGT_SPD if wp<3 else TGT_SPD_FAST
            tv=(d/dn)*s
        else: wp=0; tv=np.zeros(3)
        pub("/model/target/cmd_vel",tv[0],tv[1],tv[2])
        sep=np.linalg.norm(tp-ip)
        if USE_PN:
            iv=nav.go(ip,tp,el)
            ph="PN" if sep>=TERMINAL_R else "TM"
        else:
            ch=tp-ip; cn=np.linalg.norm(ch)
            iv=(ch/cn*INT_SPD) if cn>0.5 else np.zeros(3)
            ph="PP"
        pub("/model/interceptor/cmd_vel",iv[0],iv[1],iv[2])
        if cd>0: cd-=1
        if sep<HIT_DIST and cd==0:
            hits+=1; cd=15; nav.reset()
            print(f"  *** HIT#{hits} t={el:.1f}s sep={sep:.1f}m [{ph}] ***")
        if lp%5==0:
            print(f"  t={el:5.1f}s Sep:{sep:5.1f}m Vc={nav.Vc:+.1f} ZEM={nav.zem:.1f} [{ph}] WP:{wp}")
        lp+=1
        sl=dt-(time.time()-tt)
        if sl>0: time.sleep(sl)
    rt=time.time()-t0
    print(f"\n  {mode}: {hits} hits in {rt:.0f}s ({hits/max(rt,1)*60:.1f}/min)")
    pub("/model/target/cmd_vel",0,0,0); pub("/model/interceptor/cmd_vel",0,0,0)

if __name__=="__main__": main()
