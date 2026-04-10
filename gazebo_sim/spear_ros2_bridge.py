#!/usr/bin/env python3
import sys, time, numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from tf2_msgs.msg import TFMessage
except ImportError:
    print("Run: source /opt/ros/humble/setup.bash")
    sys.exit(1)

WAYPOINTS = [
    (50,20,20),(30,10,18),(10,5,16),
    (0,20,18),(-10,-5,20),(-20,15,15),
    (-15,5,10),(-5,10,22),(10,15,17),
]
TGT_SPD=8.0; TGT_SPD_FAST=11.0; WP_THR=5.0
HIT_DIST=3.0; N_PN=4.5; INT_SPD=22.0
TERMINAL_R=12.0; CONTROL_HZ=20
FREEZE_TICKS=15   # 0.75s — quick visible flash
REARM_DIST=12.0   # interceptor starts moving sooner
LEAD_GAIN=0.8     # how much to lead the target

class ProNav3D:
    def __init__(self):
        self.prev_los=None; self.prev_R=None; self.prev_t=None
        self.prev_tp=None
        self.Vc=0; self.R=0; self.tgo=999; self.zem=0
        self.tgt_vel=np.zeros(3)
    def reset(self):
        self.prev_los=None; self.prev_R=None; self.prev_t=None
        self.prev_tp=None
        self.Vc=0; self.zem=0
        self.tgt_vel=np.zeros(3)
    def compute(self, ip, tp, t):
        los=tp-ip; self.R=np.linalg.norm(los)
        if self.R<0.1: return np.zeros(3)
        lh=los/self.R
        # estimate target velocity for lead
        if self.prev_tp is not None and self.prev_t is not None:
            dt_v=t-self.prev_t
            if 0.005<dt_v<0.5:
                new_tv=(tp-self.prev_tp)/dt_v
                # low-pass filter to reduce jitter
                self.tgt_vel=0.7*self.tgt_vel+0.3*new_tv
        if self.prev_los is not None and self.prev_t is not None:
            dt=t-self.prev_t
            if 0.005<dt<0.5:
                dlos=(lh-self.prev_los)/dt
                self.Vc=-(self.R-self.prev_R)/dt if self.prev_R else 0
                vc=max(self.Vc,4.0); self.tgo=self.R/vc
                lrp=dlos-np.dot(dlos,lh)*lh
                self.zem=self.R*np.linalg.norm(lrp)*self.tgo
                if self.R<TERMINAL_R:
                    # terminal: pure lead-pursuit toward predicted intercept
                    aim=tp+self.tgt_vel*self.tgo*LEAD_GAIN
                    lead_los=aim-ip
                    ll=np.linalg.norm(lead_los)
                    if ll>0.1:
                        self._s(lh,t,tp); return (lead_los/ll)*INT_SPD
                    self._s(lh,t,tp); return lh*INT_SPD
                # APN with lead
                a=N_PN*vc*lrp
                corr=a*min(self.tgo,1.5)
                cm=np.linalg.norm(corr)
                if cm>INT_SPD*0.7: corr=corr/cm*INT_SPD*0.7
                # blend lead vector into base velocity
                aim=tp+self.tgt_vel*self.tgo*LEAD_GAIN
                lead_dir=aim-ip
                ld=np.linalg.norm(lead_dir)
                base=lead_dir/ld if ld>0.1 else lh
                v=base*INT_SPD+corr
                s=np.linalg.norm(v)
                if s>INT_SPD*1.2: v=v/s*INT_SPD*1.2
                self._s(lh,t,tp); return v
        self._s(lh,t,tp); return lh*INT_SPD
    def _s(self,lh,t,tp):
        self.prev_los=lh.copy(); self.prev_R=self.R; self.prev_t=t
        self.prev_tp=tp.copy()

class SpearNode(Node):
    def __init__(self):
        super().__init__('spear_bridge')
        self.int_pub=self.create_publisher(Twist,'/model/interceptor/cmd_vel',10)
        self.tgt_pub=self.create_publisher(Twist,'/model/target/cmd_vel',10)
        self.int_pos=None; self.tgt_pos=None
        self.pose_sub=self.create_subscription(
            TFMessage,
            '/world/spear_intercept/dynamic_pose/info',
            self.pose_cb, 10)
        self.cmd_timer=self.create_timer(1.0/CONTROL_HZ, self.tick)
        self.pub_timer=self.create_timer(0.05, self.republish)
        self.nav=ProNav3D(); self.wp=0; self.hits=0
        self.t0=time.time(); self.cd=0; self.lp=0
        self.pose_count=0
        self.last_iv=np.zeros(3)
        self.last_tv=np.zeros(3)
        self.armed=True
        self.get_logger().info(f"SPEAR v17 — {CONTROL_HZ}Hz — fast smart demo")

    def pose_cb(self, msg):
        for tf in msg.transforms:
            n=tf.child_frame_id
            p=tf.transform.translation
            if n=='interceptor':
                self.int_pos=np.array([p.x,p.y,p.z])
            elif n=='target':
                self.tgt_pos=np.array([p.x,p.y,p.z])
        self.pose_count+=1

    def pub_vel(self, publisher, v):
        m=Twist()
        m.linear.x=float(v[0]); m.linear.y=float(v[1]); m.linear.z=float(v[2])
        publisher.publish(m)

    def republish(self):
        self.pub_vel(self.int_pub, self.last_iv)
        self.pub_vel(self.tgt_pub, self.last_tv)

    def tick(self):
        el=time.time()-self.t0
        if self.int_pos is None or self.tgt_pos is None:
            if self.lp%20==0:
                self.get_logger().warn(f"Waiting for poses... ({self.pose_count})")
            self.lp+=1; return
        ip=self.int_pos.copy(); tp=self.tgt_pos.copy()

        # Target — always pursuing current waypoint
        w=np.array(WAYPOINTS[self.wp]); d=w-tp; dn=np.linalg.norm(d)
        if dn<WP_THR:
            self.wp=(self.wp+1)%len(WAYPOINTS)
            ph=1 if self.wp<3 else(2 if self.wp<6 else 3)
            self.get_logger().info(f"[TGT] WP{self.wp}/{len(WAYPOINTS)} Ph{ph}")
            w=np.array(WAYPOINTS[self.wp]); d=w-tp; dn=np.linalg.norm(d)
        s=TGT_SPD if self.wp<3 else TGT_SPD_FAST
        self.last_tv=(d/dn)*s if dn>0.1 else np.zeros(3)

        sep=np.linalg.norm(tp-ip)

        # Interceptor — brief freeze, then immediate re-engagement
        if self.cd>0:
            self.cd-=1
            self.last_iv=np.zeros(3)
            if self.cd==0:
                self.nav.reset()
                self.armed=True  # immediate re-arm after short freeze
        else:
            if not self.armed and sep>REARM_DIST:
                self.armed=True
            if self.armed:
                iv=self.nav.compute(ip,tp,el)
                # emergency long-range chase
                if sep>80.0:
                    chase=tp-ip; cn=np.linalg.norm(chase)
                    if cn>1.0: iv=(chase/cn)*INT_SPD
                sp=np.linalg.norm(iv)
                if sp>INT_SPD*1.5: iv=iv/sp*INT_SPD*1.5
                self.last_iv=iv
            else:
                self.last_iv=np.zeros(3)

        # Hit detection
        if sep<HIT_DIST and self.cd==0 and self.armed:
            self.hits+=1
            self.cd=FREEZE_TICKS
            self.armed=False
            self.last_iv=np.zeros(3)
            self.get_logger().info(
                f"*** HIT#{self.hits} t={el:.1f}s sep={sep:.1f}m ***")

        # Status
        if self.lp%10==0:
            ph="PN" if sep>=TERMINAL_R else "TM"
            if self.cd>0: ph="HIT"
            elif not self.armed: ph="RST"
            self.get_logger().info(
                f"t={el:5.1f}s Sep:{sep:5.1f}m Vc={self.nav.Vc:+.1f} "
                f"ZEM={self.nav.zem:.1f} [{ph}] WP:{self.wp} H:{self.hits}")
        self.lp+=1

def main():
    print("="*55)
    print("  SPEAR v17 — fast smart demo")
    print("="*55)
    rclpy.init()
    node=SpearNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt:
        el=time.time()-node.t0
        print(f"\n  {node.hits} hits in {el:.0f}s ({node.hits/max(el,1)*60:.1f}/min)")
    finally:
        node.pub_vel(node.int_pub,np.zeros(3))
        node.pub_vel(node.tgt_pub,np.zeros(3))
        node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()
