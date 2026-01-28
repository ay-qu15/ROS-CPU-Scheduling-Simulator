#!/usr/bin/env python3
import rospy
from cpu_scheduler.msg import RegisterProcess, ScheduleDecision, ProcessCompleted
from collections import deque
import csv, os

class FCFS_Scheduler:
    def __init__(self):
        rospy.init_node('fcfs_scheduler')
        self.queue = deque()
        self.arrival = {}
        self.dispatch = {}

        # CSV log setup
        log_dir = os.path.expanduser('~/scheduler_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        self.csv = os.path.join(log_dir, 'fcfs_scheduler.csv')
        if not os.path.exists(self.csv):
            with open(self.csv,'w',newline='') as f:
                csv.writer(f).writerow(['pid','dispatch_time','completion_time','wait','turnaround'])

        rospy.Subscriber('register_process', RegisterProcess, self.on_register)
        rospy.Subscriber('process_completed', ProcessCompleted, self.on_complete)
        self.pub = rospy.Publisher('schedule_decision', ScheduleDecision, queue_size=10)
        rospy.loginfo("[FCFS] Ready")
        rospy.spin()

    def on_register(self, msg: RegisterProcess):
        pid = msg.pid
        self.queue.append((pid, msg.burst_time))
        self.arrival[pid] = rospy.get_time()
        rospy.loginfo(f"[FCFS] Registered {pid}")
        rospy.sleep(0.1)  # Add a 0.1-second delay
        if len(self.queue) == 1:
            self.dispatch_next()

    def dispatch_next(self):
        pid, burst = self.queue[0]
        self.dispatch[pid] = rospy.get_time()
        dec = ScheduleDecision(pid=pid, timeslice=burst)
        self.pub.publish(dec)
        rospy.loginfo(f"[FCFS] Dispatch {pid}, burst={burst}")

    def on_complete(self, msg: ProcessCompleted):
        rospy.loginfo(f"[FCFS] Received ProcessCompleted for PID={msg.pid}, remaining={msg.remaining_burst_time}")
        pid = msg.pid
        comp = msg.timestamp
        dtime = self.dispatch.get(pid, comp)
        atime = self.arrival.get(pid, dtime)
        wait = dtime - atime
        turn = comp - atime
        with open(self.csv, 'a', newline='') as f:
            csv.writer(f).writerow([pid, dtime, comp, wait, turn])
        rospy.loginfo(f"[FCFS] Complete {pid}: wait={wait:.2f}, turn={turn:.2f}")
        if self.queue and self.queue[0][0] == pid:
            self.queue.popleft()
        if self.queue:
            self.dispatch_next()

if __name__ == '__main__':
    try:
        FCFS_Scheduler()
    except rospy.ROSInterruptException:
        pass