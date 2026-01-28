#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import os

log_dir = os.path.expanduser('~/scheduler_ws/logs')
schedulers = ['fcfs_scheduler', 'rr_scheduler', 'priority_scheduler']
plt.figure(figsize=(10, 6))
for scheduler in schedulers:
    df = pd.read_csv(os.path.join(log_dir, f'{scheduler}.csv'))
    x = range(len(df['pid']))  # Use indices for x-axis
    plt.plot(x, df['wait'].to_numpy(), label=f'{scheduler} Wait', marker='o')
    plt.plot(x, df['turnaround'].to_numpy(), label=f'{scheduler} Turnaround', marker='x')
    plt.xticks(x, df['pid'])  # Set pid strings as x-tick labels
plt.legend()
plt.xlabel('Process ID')
plt.ylabel('Time (s)')
plt.title('Scheduling Performance Comparison')
plt.grid(True)
plt.savefig(os.path.join(log_dir, 'scheduling_comparison.png'))
plt.show()