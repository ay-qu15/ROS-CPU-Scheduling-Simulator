#!/usr/bin/env python3
import rospy
from cpu_scheduler.msg import RegisterProcess, ScheduleDecision, ProcessCompleted
from collections import deque
import threading
import csv
import os

# Global variables
ready = deque()  # Queue of processes sorted by priority
arrival = {}  # Store arrival times
dispatch = {}  # Store dispatch times
current = None  # Currently running process
lock = threading.Lock()

# CSV file setup
log_dir = os.path.expanduser('~/scheduler_ws/logs')
os.makedirs(log_dir, exist_ok=True)  # Create directory if it doesn’t exist
csv_file = os.path.join(log_dir, 'priority_scheduler.csv')
# Initialize CSV with headers if it doesn’t exist
if not os.path.exists(csv_file):
    with open(csv_file, 'w', newline='') as f:
        csv.writer(f).writerow(['pid', 'dispatch_time', 'completion_time', 'wait', 'turnaround'])


def register_cb(msg):
    """Callback for registering new processes."""
    with lock:
        # Add process to ready queue with its details
        ready.append({
            'pid': msg.pid,
            'burst': msg.burst_time,
            'prio': msg.prio,
            'arrival': rospy.get_time()
        })
        # Sort by priority (lower number = higher priority), then arrival time
        tmp = sorted(ready, key=lambda x: (x['prio'], x['arrival']))
        ready.clear()
        ready.extend(tmp)
        rospy.loginfo(f"[PRIO] Registered {msg.pid}")
        rospy.sleep(0.1)  # Small delay to ensure process node is ready


def complete_cb(msg):
    """Callback for when a process completes."""
    global current
    with lock:
        rospy.loginfo(f"[PRIO] Received ProcessCompleted for PID={msg.pid}")
        if current and msg.pid == current['pid']:
            rospy.loginfo(f"[PRIO] {msg.pid} completed")
            pid = msg.pid
            completion_time = msg.timestamp
            dispatch_time = dispatch.get(pid, completion_time)  # Fallback to completion if not dispatched
            arrival_time = arrival.get(pid, dispatch_time)  # Fallback to dispatch if not arrived
            wait = dispatch_time - arrival_time
            turnaround = completion_time - arrival_time

            # Write process details to CSV
            with open(csv_file, 'a', newline='') as f:
                csv.writer(f).writerow([pid, dispatch_time, completion_time, wait, turnaround])

            # Clean up
            current = None
            arrival.pop(pid, None)
            dispatch.pop(pid, None)


def scheduler_loop():
    """Main scheduling loop."""
    global current
    pub = rospy.Publisher('schedule_decision', ScheduleDecision, queue_size=10)
    rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown():
        with lock:
            if current is None and ready:
                current = ready.popleft()  # Get highest-priority process
                pid, burst = current['pid'], current['burst']
                dispatch[pid] = rospy.get_time()  # Record dispatch time
                arrival.setdefault(pid, current['arrival'])  # Ensure arrival time is stored
                dec = ScheduleDecision(pid=pid, timeslice=burst)
                rospy.loginfo(f"[PRIO] Dispatch {pid} (prio={current['prio']})")
                pub.publish(dec)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('priority_scheduler')
    rospy.Subscriber('register_process', RegisterProcess, register_cb)
    rospy.Subscriber('process_completed', ProcessCompleted, complete_cb)
    rospy.loginfo("[PRIO] Started")
    try:
        scheduler_loop()
    except rospy.ROSInterruptException:
        pass