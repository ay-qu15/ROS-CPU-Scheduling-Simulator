# ROS CPU Scheduling Simulator

**ROS CPU Scheduling Simulator** is a distributed simulation tool developed for the Modern Operating Systems course. Using the **Robot Operating System (ROS)** framework, the simulator models process scheduling in a decentralized node architecture and evaluates the performance of fundamental CPU scheduling algorithms.

---

## Project Overview

The goal of this project is to bridge **theoretical Operating Systems concepts** with practical, distributed system implementation. The simulator models:

- Process interactions and scheduling decisions  
- Performance analysis via average wait time and turnaround time  
- Dynamic communication between nodes using ROS topics and services  

---

## System Architecture

The simulator consists of three primary components:

1. **Scheduler Node**  
   - Implements FCFS, Round Robin (with adjustable time quantum), and Priority scheduling algorithms  
   - Manages process execution order and dispatch logic  

2. **Process Nodes**  
   - Simulate independent tasks with user-defined or randomized burst times and priorities  
   - Communicate with the scheduler node to request CPU time  

3. **Visualization Engine**  
   - Logs real-time performance metrics (Average Wait Time, Turnaround Time)  
   - Provides insights into scheduling behavior through textual output or graphical plots  

---

## Implemented Scheduling Algorithms & Metrics

| Scheduler       | Avg. Wait Time (s) | Avg. Turnaround Time (s) |
|-----------------|-----------------|-------------------------|
| FCFS             | 0.103           | 5.609                   |
| Round Robin      | 2.136           | 4.142                   |
| Priority         | 0.347           | 5.856                   |

**Observations:**

- **FCFS:** Minimal initial wait times but suffers from higher turnaround times due to non-preemptive execution  
- **Round Robin:** Balanced for time-sensitive workloads; frequent context switching slightly increases wait times  
- **Priority:** High-priority processes can monopolize CPU; starvation is mitigated with aging  

---

## Development Phases

1. **Phase 1:** Basic FCFS implementation and process registration logic  
2. **Phase 2:** Integration of Round Robin and Priority scheduling with adjustable parameters  
3. **Phase 3:** Real-time logging, performance analysis, and metric visualization  

---

## Highlights

- Demonstrates practical **ROS node communication** for distributed systems  
- Implements **core CPU scheduling algorithms** for OS performance evaluation  
- Measures key performance metrics: Average Wait Time and Turnaround Time  
- Modular design allows easy extension for additional algorithms or scheduling policies  

---

## Tools & Technologies

- **Framework:** Robot Operating System (ROS)  
- **Language:** Python / C++ (based on your implementation)  
- **Visualization:** Terminal logs or ROS-compatible plotting tools  
- **Platform:** Ubuntu Linux with ROS environment  

---
