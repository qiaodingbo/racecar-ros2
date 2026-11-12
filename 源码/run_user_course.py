"""Run the installed controller with a bounded, supervised corner test."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

ROOT=Path('/root/racecar/tools/s_bend')


def child(heartbeat):
    sys.path.insert(0,str(ROOT))
    import dense_s_bend as control
    original=control.DenseSBend

    class TestRun(original):
        def __init__(self, settings, observing=False):
            super().__init__(settings,observing=observing)
            self.corner_active=False
            self.last_exit=None
            self.heartbeat=heartbeat
            self.test_stopping=None
            self.still_since=None
            self.last_motion=time.monotonic()
            self.record(dict(kind='test_throttle',at=self.last_motion,profile='speed_2p0_bends_1p6'))
            os.write(self.heartbeat,b'.')

        def stop(self,reason):
            if self.test_stopping is None:
                now=time.monotonic()
                self.test_stopping=reason,now
                self.record(dict(kind='test_stop_requested',at=now,reason=reason,
                                 distance=self.distance,speed=self.speed))
                print(f'TEST STOP REQUEST {reason} distance={self.distance:.2f}m speed={self.speed:.3f}m/s',flush=True)
            self.neutral(self.servo)

        def control(self):
            if self.stop_reason is not None:
                return
            os.write(self.heartbeat,b'.')
            if self.test_stopping is not None:
                now=time.monotonic()
                self.neutral(self.servo)
                recent=self.encoder_at is not None and now-self.encoder_at<.12
                if recent and abs(self.speed)<.035:
                    if self.still_since is None:
                        self.still_since=now
                else:
                    self.still_since=None
                reason,started=self.test_stopping
                if (self.still_since is not None and now-self.still_since>.30) or now-started>3.0:
                    self.record(dict(kind='test_settled',at=now,speed=self.speed,
                                     distance=self.distance,confirmed=self.still_since is not None))
                    os.write(self.heartbeat,b'S')
                    original.stop(self,reason)
                return
            now=time.monotonic()
            if now-self.started_at>55. or self.distance>36.:
                self.stop('test_limit')
                return
            if not self.ever_drove or abs(self.speed)>.08:
                self.last_motion=now
            if (self.ever_drove and now-self.last_motion>3.) or (not self.ever_drove and now-self.started_at>9.):
                self.stop('test_no_progress')
                return
            super().control()
            if self.test_stopping is not None:
                return
            active=self.planner.corner.active is not None
            if active and not self.corner_active:
                self.record(dict(kind='test_corner_enter',at=now,distance=self.distance))
                print(f'TEST ENTER CORNER distance={self.distance:.2f}m',flush=True)
            self.corner_active=active
            event=self.planner.corner.event
            if event and event['state']=='exit_reacquired' and event is not self.last_exit:
                self.last_exit=event
                self.record(dict(kind='test_exit_reacquired',at=now,distance=self.distance,
                                 yaw_deg=event['yaw_deg']))
                print(f'TEST EXIT REACQUIRED distance={self.distance:.2f}m yaw={event["yaw_deg"]:.1f}deg',flush=True)

    control.DenseSBend=TestRun
    sys.argv=[sys.argv[0]]
    control.main()


def emergency_neutral():
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init(args=[])
    node=rclpy.create_node('corner_test_stop')
    try:
        pub=node.create_publisher(Twist,'/car_cmd_vel',1)
        message=Twist()
        message.linear.x,message.angular.z=1500.,84.5
        for _ in range(65):
            pub.publish(message)
            time.sleep(.05)
        print('GUARDIAN NEUTRAL BURST COMPLETE',flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def supervise():
    interrupted=False
    def handle(signum,frame):
        nonlocal interrupted
        interrupted=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig,handle)
    r,w=os.pipe()
    proc=subprocess.Popen([sys.executable,__file__,'child',str(w)],cwd=ROOT,pass_fds=(w,))
    os.close(w)
    start=time.monotonic()
    last=None
    stopping=False
    reason=None
    try:
        while proc.poll() is None:
            ready,_,_=select.select([r],[],[],.10)
            data=os.read(r,8192) if ready else b''
            if data:
                last=time.monotonic()
                stopping=stopping or b'S' in data
            now=time.monotonic()
            if interrupted:
                reason='external stop'
            elif now-start>62.:
                reason='outer time limit'
            elif (last is not None and now-last>(2.5 if stopping else .65)) or (last is None and now-start>8.):
                reason='control heartbeat lost'
            if reason is not None:
                break
        if reason is not None:
            print('GUARDIAN STOP '+reason,flush=True)
            proc.terminate()
            try:
                proc.wait(timeout=1.1)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        if reason is not None or proc.returncode!=0:
            emergency_neutral()
        return proc.returncode
    finally:
        os.close(r)


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='child':
        child(int(sys.argv[2]))
    else:
        raise SystemExit(supervise())

