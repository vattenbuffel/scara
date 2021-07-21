import sys
import os

def print_enable():
        sys.stdout = sys.__stdout__

def print_disable():
    print("WARNING: Disabling print")
    sys.stdout = open(os.devnull, 'w')