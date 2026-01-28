#!/usr/bin/env python3
import rospy, sys
from cpu_scheduler.msg import RegisterProcess, ScheduleDecision, ProcessCompleted

def process_node(pid, burst_time, prio):
    rospy.init_node(f'process_{pid}', anonymous=True)
    pub_reg = rospy.Publisher('register_process', RegisterProcess, queue_size=1, latch=True)
    pub_done = rospy.Publisher('process_completed', ProcessCompleted, queue_size=1)

    rospy.sleep(1.0)
    reg = RegisterProcess()
    reg.pid = pid
    reg.burst_time = burst_time
    reg.prio = prio
    pub_reg.publish(reg)
    rospy.loginfo(f"[Process {pid}] Registered (burst={burst_time}, prio={prio})")

    state = {'remaining': burst_time, 'done': False}

    def on_schedule(msg):
        if msg.pid != pid or state['done']:
            return
        run = min(msg.timeslice, state['remaining'])
        rospy.loginfo(f"[Process {pid}] Running for {run:.2f}s")
        rospy.sleep(run)
        state['remaining'] -= run

        done = ProcessCompleted()
        done.pid = pid
        done.remaining_burst_time = state['remaining']
        done.timestamp = rospy.get_time()
        pub_done.publish(done)
        rospy.loginfo(f"[Process {pid}] Published ProcessCompleted with remaining={state['remaining']:.2f}")
        rospy.loginfo(f"[Process {pid}] Slice done, remaining={state['remaining']:.2f}")

        if state['remaining'] <= 0:
            state['done'] = True
            rospy.signal_shutdown(f"[Process {pid}] Finished")

    rospy.Subscriber('schedule_decision', ScheduleDecision, on_schedule)
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: process_node.py <pid> <burst_time> <priority>")
        sys.exit(1)
    process_node(sys.argv[1], float(sys.argv[2]), int(sys.argv[3]))