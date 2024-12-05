#!/usr/bin/env python

##
#
# Open solver_stats.csv and compute the average iteration time.
#
##

import csv
import numpy as np

with open('solver_stats.csv', 'r') as f:
    reader = csv.reader(f)
    next(reader)  # Skip header
    times = [float(row[1]) for row in reader]

times = np.array(times)
mean_time_ms = np.mean(times) * 1e3
std_time_ms = np.std(times) * 1e3
print(f"Avg. iteration time: {mean_time_ms:.4f} ms ± {std_time_ms:.4f} ms")
