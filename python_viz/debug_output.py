#!/usr/bin/env python3
"""
Debug what output we're getting from the simulation
"""

import subprocess

def test_simulation():
    print("Testing simulation output...")
    
    result = subprocess.run([
        '../run_simulation.sh', '--map_file', '../maps/simple_corridor.map', '--seed', '42'
    ], capture_output=True, text=True, timeout=30)
    
    print(f"Return code: {result.returncode}")
    print(f"Stdout length: {len(result.stdout)}")
    print(f"Stderr length: {len(result.stderr)}")
    
    print("\nFirst 10 lines of stdout:")
    for i, line in enumerate(result.stdout.split('\n')[:10]):
        print(f"{i}: {repr(line)}")
    
    print("\nFirst 5 lines of stderr:")
    for i, line in enumerate(result.stderr.split('\n')[:5]):
        print(f"{i}: {repr(line)}")

if __name__ == '__main__':
    test_simulation()