#!/usr/bin/env python3
"""Safe single-joint MIT move test for node 1 on can0.

CORRECT sequence (the motor only streams LIVE encoder data in CLOSED_LOOP;
in IDLE it returns stale position):
  1. Enter CLOSED_LOOP in PASSIVE mode (kp=0,kd=0 -> zero torque, motor free)
  2. Send passive frames so the encoder samples; read the LIVE position
  3. Hold at that live position with the target gains for 1s (smooth engage)
  4. Smooth move: start -> +A -> start -> -A -> start
  5. Always return to IDLE (zero torque) on completion or Ctrl-C

  python3 node1_move.py                 # node 1, can0, A=0.15rad, kp=20 kd=1
  python3 node1_move.py --amp 0.25 --kp 30
"""
import argparse, can, math, struct, time

CMD_MIT, CMD_SET_STATE, CMD_ENC_EST = 0x008, 0x007, 0x009
IDLE, CLOSED_LOOP = 1, 8
GEAR_RATIO = 8.0
MIT_P=(-12.5,12.5); MIT_V=(-45.0,45.0); MIT_KP=(0.0,500.0); MIT_KD=(0.0,5.0); MIT_T=(-18.0,18.0)

def can_id(n,c): return (n<<5)|c
def f2u(x,lo,hi,b): x=max(lo,min(hi,x)); return int((x-lo)/(hi-lo)*((1<<b)-1))
def send_raw(bus,n,c,d): bus.send(can.Message(arbitration_id=can_id(n,c),data=d,is_extended_id=False))
def set_state(bus,n,s): send_raw(bus,n,CMD_SET_STATE,struct.pack('<I',s))

def send_mit(bus,n,pos,vel=0.0,kp=0.0,kd=0.0,tq=0.0):
    p=f2u(pos,*MIT_P,16); v=f2u(vel,*MIT_V,12); kp_=f2u(kp,*MIT_KP,12); kd_=f2u(kd,*MIT_KD,12); t=f2u(tq,*MIT_T,12)
    data=bytes([(p>>8)&0xFF,p&0xFF,(v>>4)&0xFF,((v&0xF)<<4)|((kp_>>8)&0xF),kp_&0xFF,(kd_>>4)&0xFF,((kd_&0xF)<<4)|((t>>8)&0xF),t&0xFF])
    send_raw(bus,n,CMD_MIT,data)

def read_pos_abs(bus,node,duration=1.0):
    """Latest absolute output-shaft position (rad) from the 0x009 encoder stream.
    Only valid while the motor is in CLOSED_LOOP."""
    enc=can_id(node,CMD_ENC_EST); pos=None; dl=time.time()+duration
    while time.time()<dl:
        r=bus.recv(timeout=0.02)
        if r and r.arbitration_id==enc and len(r.data)>=4:
            pos=struct.unpack_from('<f',bytes(r.data),0)[0]*2*math.pi/GEAR_RATIO
    return pos

def latest_pos(bus,node):
    """Drain pending frames, return the most recent encoder pos or None."""
    pos=None; r=bus.recv(timeout=0.0)
    while r:
        if r.arbitration_id==can_id(node,CMD_ENC_EST) and len(r.data)>=4:
            pos=struct.unpack_from('<f',bytes(r.data),0)[0]*2*math.pi/GEAR_RATIO
        r=bus.recv(timeout=0.0)
    return pos

def smoothstep(t): t=max(0.,min(1.,t)); return t*t*(3-2*t)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--node',type=int,default=1); ap.add_argument('--can',default='can0')
    ap.add_argument('--amp',type=float,default=0.15); ap.add_argument('--kp',type=float,default=20.0)
    ap.add_argument('--kd',type=float,default=1.0); ap.add_argument('--seg',type=float,default=2.0)
    a=ap.parse_args()
    bus=can.interface.Bus(channel=a.can,interface='socketcan')
    node=a.node
    try:
        # 1) CLOSED_LOOP in PASSIVE so the encoder streams LIVE data (no torque).
        print(f"[test] node {node}/{a.can}: CLOSED_LOOP passive (kp=0,kd=0, no torque) ...")
        set_state(bus,node,CLOSED_LOOP); time.sleep(0.2)
        for _ in range(40):
            send_mit(bus,node,0.0,kp=0.0,kd=0.0); time.sleep(0.01)
        # 2) read the TRUE live position
        start=read_pos_abs(bus,node,1.0)
        if start is None:
            print("[test] ERROR: no live encoder feedback — powered? on this bus?")
            set_state(bus,node,IDLE); return
        print(f"[test] LIVE start position = {start:+.4f} rad (output shaft)")
        if abs(start) > MIT_P[1]-1.0:
            print(f"[test] ABORT: start {start:.2f} too close to ±12.5 limit")
            set_state(bus,node,IDLE); return
        # 3) hold at the live start with target gains 1s -> smooth engagement, no jump
        print(f"[test] plan: ±{a.amp} rad, kp={a.kp} kd={a.kd}, {a.seg}s/seg. Engaging in 1s (Ctrl-C->IDLE)")
        t_hold=time.time()+1.0
        while time.time()<t_hold:
            send_mit(bus,node,start,kp=a.kp,kd=a.kd); time.sleep(0.01)
        # 4) smooth move
        waypoints=[start, start+a.amp, start, start-a.amp, start]
        dt=0.01
        for i in range(len(waypoints)-1):
            p0,p1=waypoints[i],waypoints[i+1]; n=int(a.seg/dt)
            for k in range(n+1):
                tgt=p0+(p1-p0)*smoothstep(k/n)
                send_mit(bus,node,tgt,kp=a.kp,kd=a.kd)
                meas=latest_pos(bus,node)
                if k%50==0 and meas is not None:
                    print(f"  seg{i} t={k*dt:4.1f}s  cmd={tgt:+.3f}  meas={meas:+.3f}  err={tgt-meas:+.3f}")
                time.sleep(dt)
        print("[test] move complete")
    except KeyboardInterrupt:
        print("\n[test] interrupted")
    finally:
        print("[test] -> IDLE (zero torque)")
        for _ in range(5): send_mit(bus,node,0.0,kp=0.0,kd=0.0); time.sleep(0.01)
        set_state(bus,node,IDLE); bus.shutdown()

if __name__=='__main__':
    main()
