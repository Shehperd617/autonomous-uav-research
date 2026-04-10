#!/usr/bin/env python3
"""
SPEAR Monte Carlo v5.3 — adds --aim-jitter for stochastic guidance experiments.
Modes: passive | easy | medium | hard
Flags: --noise, --strict, --ekf, --apn, --aim-jitter <meters>
"""
import numpy as np
import csv, time, argparse
from dataclasses import dataclass
from collections import deque

# ============ CONFIG ============
TGT_SPD_MIN=5.0; TGT_SPD_MAX=12.0
TGT_BURST_SPD=22.0
INT_SPD=22.0
INT_SPD_TERMINAL=40.0
HIT_DIST_NORMAL=3.0
HIT_DIST_STRICT=1.5
N_PN=4.5
N_APN=4.5
TERMINAL_R=18.0
LEAD_GAIN=0.8
LEAD_GAIN_TERMINAL=1.0
DT=0.05
MAX_SIM_TIME=30.0
MAX_INITIAL_SEP=200.0
MIN_INITIAL_SEP=50.0
THREAT_RANGE=45.0

APN_MAX_ACCEL=30.0
APN_MAX_TGO=2.0

POSE_NOISE_STD=0.5
DROPOUT_PROB=0.05
DROPOUT_MAX_DUR=0.4
LATENCY_TICKS=2

EASY_BREAK_DURATION=2.0
EASY_RECOVERY_DURATION=2.0

# ============ EKF ============
class TargetEKF:
    def __init__(self):
        self.x=np.zeros(6)
        self.P=np.eye(6)*100.0
        self.q_pos=0.5
        self.q_vel=25.0
        self.R=np.eye(3)*(POSE_NOISE_STD**2 + 0.05)
        self.H=np.zeros((3,6))
        self.H[0,0]=1; self.H[1,1]=1; self.H[2,2]=1
        self.innovation_gate=12.0
        self.initialized=False
        self.last_t=None
        self.vel_history=deque(maxlen=6)
        self.vel_t_history=deque(maxlen=6)
        self.accel_est=np.zeros(3)

    def reset(self):
        self.initialized=False
        self.last_t=None
        self.vel_history.clear()
        self.vel_t_history.clear()
        self.accel_est=np.zeros(3)

    def predict(self, dt):
        F=np.eye(6)
        F[0,3]=dt; F[1,4]=dt; F[2,5]=dt
        self.x=F @ self.x
        Q=np.zeros((6,6))
        Q[0,0]=Q[1,1]=Q[2,2]=self.q_pos*dt
        Q[3,3]=Q[4,4]=Q[5,5]=self.q_vel*dt
        self.P=F @ self.P @ F.T + Q

    def update(self, z, in_terminal=False):
        y=z - self.H @ self.x
        if not in_terminal and np.linalg.norm(y) > self.innovation_gate:
            R_inflated=self.R*9.0
            S=self.H @ self.P @ self.H.T + R_inflated
            K=self.P @ self.H.T @ np.linalg.inv(S)
            self.x=self.x + K @ y
            I=np.eye(6)
            self.P=(I - K @ self.H) @ self.P
            self.P[3,3]+=5.0; self.P[4,4]+=5.0; self.P[5,5]+=5.0
            return
        S=self.H @ self.P @ self.H.T + self.R
        K=self.P @ self.H.T @ np.linalg.inv(S)
        self.x=self.x + K @ y
        I=np.eye(6)
        self.P=(I - K @ self.H) @ self.P

    def update_accel(self, t):
        self.vel_history.append(self.x[3:6].copy())
        self.vel_t_history.append(t)
        if len(self.vel_history)>=4:
            dv=self.vel_history[-1]-self.vel_history[0]
            dt=self.vel_t_history[-1]-self.vel_t_history[0]
            if dt>0.1:
                raw_accel=dv/dt
                raw_mag=np.linalg.norm(raw_accel)
                if raw_mag > APN_MAX_ACCEL:
                    raw_accel = raw_accel/raw_mag*APN_MAX_ACCEL
                self.accel_est=0.8*self.accel_est+0.2*raw_accel

    def step(self, observed_pos, t, in_terminal=False):
        if not self.initialized:
            self.x[0:3]=observed_pos
            self.x[3:6]=0
            self.last_t=t
            self.initialized=True
            return self.x[0:3].copy(), self.x[3:6].copy(), self.accel_est.copy()
        dt=t-self.last_t
        if dt>0.001:
            self.predict(dt)
            self.update(observed_pos, in_terminal=in_terminal)
            self.update_accel(t)
            self.last_t=t
        return self.x[0:3].copy(), self.x[3:6].copy(), self.accel_est.copy()

# ============ GUIDANCE ============
class ProNav3D:
    """
    PN/APN + lead pursuit with optional stochastic aim-point jitter.
    Aim jitter is added to the *target position estimate* the guidance
    sees, deliberately desynchronizing the predictive evader's optimal
    perpendicular dodge from the actual interceptor approach.
    """
    def __init__(self, use_ekf=False, use_apn=False, aim_jitter=0.0, rng=None):
        self.prev_los=None; self.prev_R=None; self.prev_t=None
        self.prev_tp=None
        self.Vc=0; self.R=0; self.tgo=999; self.zem=0
        self.tgt_vel=np.zeros(3)
        self.tgt_accel=np.zeros(3)
        self.use_ekf=use_ekf
        self.use_apn=use_apn and use_ekf
        self.aim_jitter=aim_jitter
        self.rng=rng if rng is not None else np.random.default_rng()
        self.ekf=TargetEKF() if use_ekf else None

    def compute(self, ip, observed_tp, t):
        if self.use_ekf:
            in_terminal = self.R > 0 and self.R < TERMINAL_R
            tp_filt, tv_filt, ta_filt = self.ekf.step(observed_tp, t, in_terminal=in_terminal)
            tp = tp_filt
            self.tgt_vel = tv_filt
            self.tgt_accel = ta_filt
        else:
            tp = observed_tp
            if self.prev_tp is not None and self.prev_t is not None:
                dt_v = t - self.prev_t
                if 0.005 < dt_v < 0.5:
                    new_tv = (tp - self.prev_tp) / dt_v
                    self.tgt_vel = 0.7*self.tgt_vel + 0.3*new_tv

        # Stochastic guidance kernel: jitter the target position estimate
        # used by guidance. Desynchronizes the adversarial loop without
        # requiring real perception noise.
        if self.aim_jitter > 0.0:
            tp = tp + self.rng.normal(0, self.aim_jitter, 3)

        los = tp - ip
        self.R = np.linalg.norm(los)
        if self.R < 0.1:
            return np.zeros(3)
        lh = los / self.R

        speed = INT_SPD_TERMINAL if self.R < TERMINAL_R else INT_SPD

        if self.prev_los is not None and self.prev_t is not None:
            dt = t - self.prev_t
            if 0.005 < dt < 0.5:
                dlos = (lh - self.prev_los) / dt
                self.Vc = -(self.R - self.prev_R)/dt if self.prev_R else 0
                vc = max(self.Vc, 4.0); self.tgo = self.R/vc
                lrp = dlos - np.dot(dlos,lh)*lh
                self.zem = self.R*np.linalg.norm(lrp)*self.tgo

                apn_active = self.use_apn and self.tgo < APN_MAX_TGO

                if self.R < TERMINAL_R:
                    aim = tp + self.tgt_vel*self.tgo*LEAD_GAIN_TERMINAL
                    if apn_active:
                        aim = aim + 0.5*self.tgt_accel*self.tgo*self.tgo*LEAD_GAIN_TERMINAL
                    lead_los = aim - ip
                    ll = np.linalg.norm(lead_los)
                    self._s(lh, t, tp)
                    return (lead_los/ll)*speed if ll>0.1 else lh*speed

                a_cmd = N_PN*vc*lrp
                if apn_active:
                    accel_perp = self.tgt_accel - np.dot(self.tgt_accel,lh)*lh
                    a_cmd = a_cmd + (N_APN/2.0)*accel_perp
                corr = a_cmd*min(self.tgo, 1.5)
                cm = np.linalg.norm(corr)
                if cm > speed*0.7: corr = corr/cm*speed*0.7

                aim = tp + self.tgt_vel*self.tgo*LEAD_GAIN
                if apn_active:
                    aim = aim + 0.5*self.tgt_accel*self.tgo*self.tgo*LEAD_GAIN
                lead_dir = aim - ip
                ld = np.linalg.norm(lead_dir)
                base = lead_dir/ld if ld>0.1 else lh
                v = base*speed + corr
                s = np.linalg.norm(v)
                if s > speed*1.2: v = v/s*speed*1.2
                self._s(lh, t, tp); return v
        self._s(lh, t, tp); return lh*speed

    def _s(self, lh, t, tp):
        self.prev_los=lh.copy(); self.prev_R=self.R; self.prev_t=t
        self.prev_tp=tp.copy()

# ============ EVASIVE TARGET ============
class EvasiveTarget:
    def __init__(self, mode, rng, base_speed):
        self.mode=mode
        self.rng=rng
        self.base_speed=base_speed
        self.jink_timer=0.0
        self.jink_dir=np.zeros(3)
        self.easy_state='ready'
        self.easy_state_timer=0.0
        self.break_dir=np.zeros(3)
        self.last_int_pos=None
        self.int_vel_est=np.zeros(3)

    def velocity(self, tgt_pos, int_pos, default_vel, sep, dt):
        if self.mode=='passive':
            return default_vel

        if self.last_int_pos is not None:
            new_iv=(int_pos-self.last_int_pos)/dt
            self.int_vel_est=0.6*self.int_vel_est+0.4*new_iv
        self.last_int_pos=int_pos.copy()

        threatened=sep<THREAT_RANGE

        if self.mode=='easy':
            if self.easy_state=='ready' and threatened:
                los=int_pos-tgt_pos
                ln=np.linalg.norm(los)
                if ln>0.1:
                    los_h=los/ln
                    perp=np.array([-los_h[1],los_h[0],0])
                    pn=np.linalg.norm(perp)
                    if pn>0.1:
                        self.break_dir=perp/pn
                        self.easy_state='breaking'
                        self.easy_state_timer=EASY_BREAK_DURATION
            if self.easy_state=='breaking':
                self.easy_state_timer-=dt
                if self.easy_state_timer<=0:
                    self.easy_state='recovering'
                    self.easy_state_timer=EASY_RECOVERY_DURATION
                else:
                    return self.break_dir*TGT_BURST_SPD
            if self.easy_state=='recovering':
                self.easy_state_timer-=dt
                if self.easy_state_timer<=0:
                    self.easy_state='ready'
                return default_vel
            return default_vel

        if self.mode=='medium':
            if threatened:
                self.jink_timer-=dt
                if self.jink_timer<=0:
                    self.jink_dir=self.rng.uniform(-1,1,3)
                    n=np.linalg.norm(self.jink_dir)
                    if n>0.1: self.jink_dir/=n
                    self.jink_timer=self.rng.uniform(0.6,1.2)
                return self.jink_dir*TGT_BURST_SPD
            return default_vel

        if self.mode=='hard':
            if threatened:
                int_speed=np.linalg.norm(self.int_vel_est)
                if int_speed>1.0:
                    closing=int_speed+self.base_speed*0.5
                    tgo=sep/max(closing,1.0)
                    pred_int=int_pos+self.int_vel_est*tgo*0.7
                    threat_axis=pred_int-tgt_pos
                    ta_n=np.linalg.norm(threat_axis)
                    if ta_n>0.1:
                        threat_axis/=ta_n
                        up=np.array([0,0,1.0])
                        perp=np.cross(threat_axis,up)
                        pn=np.linalg.norm(perp)
                        if pn>0.1:
                            perp/=pn
                            z_jink=self.rng.choice([-1,1])*0.4
                            escape=perp*1.0-threat_axis*0.3+np.array([0,0,z_jink])
                            en=np.linalg.norm(escape)
                            if en>0.1:
                                return (escape/en)*TGT_BURST_SPD
            return default_vel

        return default_vel

# ============ PERCEPTION NOISE ============
class NoisyPerception:
    def __init__(self, rng, enabled):
        self.rng=rng
        self.enabled=enabled
        self.dropout_remaining=0.0
        self.last_good_pos=None
        self.history=deque(maxlen=LATENCY_TICKS+1)

    def observe(self, true_pos, dt):
        if not self.enabled:
            return true_pos.copy()
        self.history.append(true_pos.copy())
        delayed=self.history[0] if len(self.history)>=LATENCY_TICKS+1 else true_pos.copy()
        if self.dropout_remaining>0:
            self.dropout_remaining-=dt
            return self.last_good_pos.copy() if self.last_good_pos is not None else delayed
        if self.rng.random()<DROPOUT_PROB*dt*20:
            self.dropout_remaining=self.rng.uniform(0.1,DROPOUT_MAX_DUR)
            return self.last_good_pos.copy() if self.last_good_pos is not None else delayed
        noisy=delayed+self.rng.normal(0,POSE_NOISE_STD,3)
        self.last_good_pos=noisy.copy()
        return noisy

# ============ ENGAGEMENT ============
@dataclass
class Result:
    seed: int
    hit: bool
    time_to_intercept: float
    initial_sep: float
    closing_vel_avg: float
    min_sep: float
    final_zem: float
    target_speed: float

def run_engagement(seed, mode, noise, hit_dist, use_ekf, use_apn, aim_jitter):
    rng=np.random.default_rng(seed)

    int_pos=rng.uniform(-100,100,3); int_pos[2]=rng.uniform(5,25)
    tgt_pos=rng.uniform(-100,100,3); tgt_pos[2]=rng.uniform(5,25)
    initial_sep=np.linalg.norm(tgt_pos-int_pos)
    tries=0
    while (initial_sep<MIN_INITIAL_SEP or initial_sep>MAX_INITIAL_SEP) and tries<20:
        int_pos=rng.uniform(-100,100,3); int_pos[2]=rng.uniform(5,25)
        tgt_pos=rng.uniform(-100,100,3); tgt_pos[2]=rng.uniform(5,25)
        initial_sep=np.linalg.norm(tgt_pos-int_pos)
        tries+=1

    waypoints=[rng.uniform(-50,50,3) for _ in range(3)]
    for w in waypoints: w[2]=rng.uniform(8,25)
    wp_idx=0

    tgt_speed=rng.uniform(TGT_SPD_MIN,TGT_SPD_MAX)
    nav=ProNav3D(use_ekf=use_ekf, use_apn=use_apn, aim_jitter=aim_jitter, rng=rng)
    evader=EvasiveTarget(mode,rng,tgt_speed)
    perception=NoisyPerception(rng,noise)

    t=0.0
    min_sep=initial_sep
    vc_sum=0.0; vc_count=0

    while t<MAX_SIM_TIME:
        wp=waypoints[wp_idx]
        d=wp-tgt_pos; dn=np.linalg.norm(d)
        if dn<3.0:
            wp_idx=(wp_idx+1)%len(waypoints)
            wp=waypoints[wp_idx]; d=wp-tgt_pos; dn=np.linalg.norm(d)
        default_vel=(d/dn)*tgt_speed if dn>0.1 else np.zeros(3)

        true_sep=np.linalg.norm(tgt_pos-int_pos)
        tgt_vel=evader.velocity(tgt_pos,int_pos,default_vel,true_sep,DT)
        tgt_pos=tgt_pos+tgt_vel*DT
        tgt_pos[2]=np.clip(tgt_pos[2],3.0,40.0)

        observed_tp=perception.observe(tgt_pos,DT)
        int_vel=nav.compute(int_pos,observed_tp,t)
        int_pos=int_pos+int_vel*DT

        sep=np.linalg.norm(tgt_pos-int_pos)
        if sep<min_sep: min_sep=sep
        if nav.Vc>0: vc_sum+=nav.Vc; vc_count+=1

        if sep<hit_dist:
            return Result(seed,True,t,initial_sep,
                          vc_sum/max(vc_count,1),min_sep,nav.zem,tgt_speed)
        t+=DT

    return Result(seed,False,MAX_SIM_TIME,initial_sep,
                  vc_sum/max(vc_count,1),min_sep,nav.zem,tgt_speed)

# ============ MAIN ============
def main():
    p=argparse.ArgumentParser()
    p.add_argument('-n','--runs',type=int,default=100)
    p.add_argument('-o','--output',default='mc_results.csv')
    p.add_argument('-m','--mode',choices=['passive','easy','medium','hard'],default='passive')
    p.add_argument('--noise',action='store_true')
    p.add_argument('--strict',action='store_true')
    p.add_argument('--ekf',action='store_true',help='use 6-state EKF')
    p.add_argument('--apn',action='store_true',help='use APN guidance (requires --ekf)')
    p.add_argument('--aim-jitter',type=float,default=0.0,
                   help='stochastic guidance kernel: aim-point jitter std dev in meters')
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--quiet',action='store_true')
    args=p.parse_args()

    hit_dist=HIT_DIST_STRICT if args.strict else HIT_DIST_NORMAL
    tags=[args.mode.upper()]
    if args.noise: tags.append('NOISE')
    if args.strict: tags.append('STRICT')
    if args.ekf: tags.append('EKF')
    if args.apn: tags.append('APN')
    if args.aim_jitter > 0: tags.append(f'JIT{args.aim_jitter:.2f}m')
    label='+'.join(tags)

    print("="*60)
    print(f"  SPEAR Monte Carlo v5.3 — {args.runs} engagements [{label}]")
    print(f"  hit sphere: {hit_dist}m   filter: {'EKF' if args.ekf else 'low-pass'}   "
          f"guidance: {'APN' if args.apn else 'PN'}   "
          f"aim-jitter: {args.aim_jitter:.2f}m")
    print("="*60)

    results=[]
    t0=time.time()
    rng=np.random.default_rng(args.seed)
    seeds=rng.integers(0,1_000_000,args.runs)

    for i,seed in enumerate(seeds):
        r=run_engagement(int(seed),args.mode,args.noise,hit_dist,
                         args.ekf,args.apn,args.aim_jitter)
        results.append(r)
        if not args.quiet:
            marker="HIT " if r.hit else "MISS"
            print(f"  [{i+1:4d}/{args.runs}] seed={seed:6d} sep0={r.initial_sep:6.1f}m "
                  f"tgt={r.target_speed:4.1f}m/s -> {marker} t={r.time_to_intercept:5.1f}s "
                  f"min={r.min_sep:.2f}m")

    elapsed=time.time()-t0
    hits=[r for r in results if r.hit]
    misses=[r for r in results if not r.hit]
    hit_rate=100*len(hits)/len(results)

    print(f"\n"+"="*60)
    print(f"  RESULTS — {args.runs} engagements [{label}] in {elapsed:.1f}s")
    print(f"="*60)
    print(f"  Hit rate:           {hit_rate:.1f}%  ({len(hits)}/{len(results)})")
    if hits:
        tti=[r.time_to_intercept for r in hits]
        cva=[r.closing_vel_avg for r in hits]
        zems=[r.final_zem for r in hits]
        ms=[r.min_sep for r in hits]
        print(f"  Time-to-intercept:  median={np.median(tti):.1f}s  "
              f"mean={np.mean(tti):.1f}s  max={np.max(tti):.1f}s")
        print(f"  Closing velocity:   median={np.median(cva):.1f}m/s")
        print(f"  Final ZEM:          median={np.median(zems):.2f}  mean={np.mean(zems):.2f}")
        print(f"  Hit min-sep:        median={np.median(ms):.2f}m")
    if misses:
        miss_min=[r.min_sep for r in misses]
        print(f"  MISSES:             {len(misses)}  median closest={np.median(miss_min):.1f}m  "
              f"best={np.min(miss_min):.1f}m  worst={np.max(miss_min):.1f}m")

    with open(args.output,'w',newline='') as f:
        w=csv.writer(f)
        w.writerow(['seed','hit','time_to_intercept','initial_sep',
                    'closing_vel_avg','min_sep','final_zem','target_speed'])
        for r in results:
            w.writerow([r.seed,int(r.hit),f"{r.time_to_intercept:.2f}",
                        f"{r.initial_sep:.2f}",f"{r.closing_vel_avg:.2f}",
                        f"{r.min_sep:.2f}",f"{r.final_zem:.2f}",
                        f"{r.target_speed:.2f}"])
    print(f"\n  Results written to: {args.output}")

if __name__=="__main__": main()
