#!/bin/bash

# Use taskset to run the Python script on specific cores (e.g., core 0 and 1)
taskset -c 0,1 python3 learn.py