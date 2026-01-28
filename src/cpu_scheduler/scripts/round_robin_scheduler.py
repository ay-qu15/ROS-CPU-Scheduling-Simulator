#!/usr/bin/env python3
import rospy
from cpu_scheduler.msg import RegisterProcess, ScheduleDecision, ProcessCompleted
from collections import deque
import csv, os

class RR_Scheduler:
    def __init__(self):
        rospy.init_node('rr_scheduler')
        self.quantum = rospy.get_param('~time_quantum', 2.0)
        self.queue = deque()
        self.arrival = {}
        self.dispatch = {}

        # CSV log setup
        log_dir = os.path.expanduser('~/scheduler_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        self.csv = os.path.join(log_dir, 'rr_scheduler.csv')
        if not os.path.exists(self.csv):
            with open(self.csv,'w',newline='') as f:
                csv.writer(f).writerow(['pid','dispatch_time','completion_time','wait','turnaround'])

        rospy.Subscriber('register_process', RegisterProcess, self.on_register)
        rospy.Subscriber('process_completed', ProcessCompleted, self.on_complete)
        self.pub = rospy.Publisher('schedule_decision', ScheduleDecision, queue_size=10)

        rospy.loginfo(f"[RR] Started quantum={self.quantum}")
        self.main_loop()

    def on_register(self, msg: RegisterProcess):
        pid, burst = msg.pid, msg.burst_time
        self.queue.append({'pid':pid,'burst':burst})
        self.arrival[pid] = rospy.get_time()
        rospy.loginfo(f"[RR] Registered {pid}")

    def on_complete(self, msg: ProcessCompleted):
        rospy.loginfo(f"[RR] Received ProcessCompleted for PID={msg.pid}, remaining={msg.remaining_burst_time}")
        pid = msg.pid
        remaining = msg.remaining_burst_time
        comp = msg.timestamp
        dtime = self.dispatch.get(pid, comp)
        atime = self.arrival.get(pid, dtime)
        wait, turn = dtime-atime, comp-atime
        with open(self.csv,'a',newline='') as f:
            csv.writer(f).writerow([pid,dtime,comp,wait,turn])
        rospy.loginfo(f"[RR] Complete {pid}: rem={remaining}, wait={wait:.2f}, turn={turn:.2f}")
        if remaining>0:
            self.queue.append({'pid':pid,'burst':remaining})

    def main_loop(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.queue:
                p=self.queue.popleft()
                pid, burst = p['pid'], p['burst']
                slice_time = min(self.quantum, burst)
                self.dispatch[pid]=rospy.get_time()
                msg = ScheduleDecision(pid=pid, timeslice=slice_time)
                self.pub.publish(msg)
                rospy.loginfo(f"[RR] Dispatch {pid} for {slice_time}")
                rospy.sleep(slice_time)
            rate.sleep()

if __name__=='__main__':
    try: RR_Scheduler()
    except rospy.ROSInterruptException: pass