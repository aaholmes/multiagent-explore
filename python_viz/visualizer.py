#!/usr/bin/env python3
"""
Python visualization frontend for multi-robot exploration simulation.
Creates animated GIFs showing robot movement and map discovery.
"""

import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import numpy as np
import argparse
from pathlib import Path
import subprocess
import sys

class ExplorationVisualizer:
    def __init__(self, simulation_data, map_width, map_height):
        self.data = simulation_data
        self.map_width = map_width
        self.map_height = map_height
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        
        # Color scheme
        self.colors = {
            'unexplored': '#2C2C2C',  # Dark gray
            'empty': '#E8E8E8',       # Light gray  
            'obstacle': '#1A1A1A',    # Black
            'robot0': '#FF6B6B',      # Red
            'robot1': '#4ECDC4',      # Teal
            'path0': '#FFB3B3',       # Light red
            'path1': '#B3E5E0'        # Light teal
        }
        
    def create_animation(self, output_path="exploration.gif", interval=200):
        """Create animated GIF showing exploration progress"""
        
        def animate(frame):
            self.ax.clear()
            
            if frame >= len(self.data):
                frame = len(self.data) - 1
                
            tick_data = self.data[frame]
            
            # Draw map grid
            for y in range(self.map_height):
                for x in range(self.map_width):
                    cell_type = self.get_cell_state(tick_data, x, y)
                    color = self.colors[cell_type]
                    
                    rect = Rectangle((x, self.map_height - y - 1), 1, 1, 
                                   facecolor=color, edgecolor='white', linewidth=0.5)
                    self.ax.add_patch(rect)
            
            # Draw robot positions
            for robot in tick_data['robots']:
                x, y = robot['position']
                robot_id = robot['id']
                color = self.colors[f'robot{robot_id}']
                
                # Robot position
                self.ax.scatter(x + 0.5, self.map_height - y - 0.5, 
                              c=color, s=200, marker='o', edgecolors='black', linewidth=2, zorder=10)
                
                # Robot ID label
                self.ax.text(x + 0.5, self.map_height - y - 0.5, str(robot_id), 
                           ha='center', va='center', fontweight='bold', fontsize=12, zorder=11)
            
            # Set up the plot
            self.ax.set_xlim(0, self.map_width)
            self.ax.set_ylim(0, self.map_height)
            self.ax.set_aspect('equal')
            self.ax.set_title(f'Multi-Robot Exploration - Tick {tick_data["tick"]} - Phase: {tick_data["phase"]}', 
                            fontsize=14, fontweight='bold')
            
            # Add legend
            legend_elements = [
                plt.Rectangle((0, 0), 1, 1, facecolor=self.colors['unexplored'], label='Unexplored'),
                plt.Rectangle((0, 0), 1, 1, facecolor=self.colors['empty'], label='Explored'),
                plt.Rectangle((0, 0), 1, 1, facecolor=self.colors['obstacle'], label='Obstacle'),
                plt.scatter([], [], c=self.colors['robot0'], s=100, label='Robot 0'),
                plt.scatter([], [], c=self.colors['robot1'], s=100, label='Robot 1')
            ]
            self.ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.15, 1))
            
            # Remove ticks
            self.ax.set_xticks([])
            self.ax.set_yticks([])
            
        # Create animation
        anim = animation.FuncAnimation(self.fig, animate, frames=len(self.data), 
                                     interval=interval, repeat=True, blit=False)
        
        # Save as GIF
        print(f"Creating GIF animation: {output_path}")
        anim.save(output_path, writer='pillow', fps=5)
        print(f"✅ Animation saved: {output_path}")
        
        return anim
    
    def get_cell_state(self, tick_data, x, y):
        """Determine the state of a cell based on robot maps"""
        # Check robot maps for this position
        for robot in tick_data['robots']:
            robot_map = robot['map']
            idx = y * self.map_width + x
            if idx < len(robot_map):
                if robot_map[idx] == 'Obstacle':
                    return 'obstacle'
                elif robot_map[idx] == 'Empty':
                    return 'empty'
        
        return 'unexplored'

def parse_simulation_output(output_text, map_width, map_height):
    """Parse simulation output text and extract visualization data"""
    lines = output_text.strip().split('\n')
    simulation_data = []
    current_tick = None
    robots_data = {}
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Parse tick header
        if line.startswith('=== Tick'):
            if current_tick is not None and robots_data:
                # Save previous tick data
                simulation_data.append({
                    'tick': current_tick,
                    'robots': list(robots_data.values()),
                    'phase': robots_data.get(0, {}).get('phase', 'Unknown')
                })
            
            current_tick = int(line.split()[2])
            robots_data = {}
            
        # Parse robot position and phase
        elif line.startswith('Robot') and 'pos=' in line and 'phase=' in line:
            parts = line.split()
            robot_id = int(parts[1].rstrip(':'))
            
            # Extract position
            pos_part = [p for p in parts if p.startswith('pos=')][0]
            pos_coords = pos_part.split('=')[1].strip('()')
            x, y = map(int, pos_coords.split(','))
            
            # Extract phase
            phase_part = [p for p in parts if p.startswith('phase=')][0]
            phase = phase_part.split('=')[1]
            
            robots_data[robot_id] = {
                'id': robot_id,
                'position': (x, y),
                'phase': phase,
                'map': ['Unexplored'] * (map_width * map_height)
            }
            
        # Parse robot map
        elif line.startswith('Robot') and "'s map:" in line:
            robot_id = int(line.split()[1].rstrip("'s"))
            map_lines = []
            i += 1
            
            # Read map lines until we hit empty line or next section
            while i < len(lines) and lines[i].strip() and not lines[i].startswith('Robot') and not lines[i].startswith('==='):
                map_line = lines[i].rstrip()
                if map_line and not map_line.startswith(' '):
                    # Skip coordinate prefixes like " ################## "
                    if len(map_line) >= map_width:
                        map_lines.append(map_line)
                i += 1
            i -= 1  # Back up one line
            
            # Convert map to cell states
            if robot_id in robots_data and map_lines:
                cell_map = []
                for y, map_line in enumerate(map_lines):
                    for x, char in enumerate(map_line[:map_width]):
                        if char == '#':
                            cell_map.append('Obstacle')
                        elif char == '.':
                            cell_map.append('Empty')
                        elif char == 'R':
                            cell_map.append('Empty')  # Robot position is empty space
                        else:
                            cell_map.append('Unexplored')
                
                robots_data[robot_id]['map'] = cell_map
                
        i += 1
    
    # Add final tick
    if current_tick is not None and robots_data:
        simulation_data.append({
            'tick': current_tick,
            'robots': list(robots_data.values()),
            'phase': robots_data.get(0, {}).get('phase', 'Complete')
        })
    
    return simulation_data

def run_simulation_and_create_gif(map_file, seed=42, output_gif="exploration.gif"):
    """Run Rust simulation and create GIF visualization"""
    
    # Run the simulation
    print(f"🚀 Running simulation: {map_file} (seed: {seed})")
    result = subprocess.run([
        '../run_simulation.sh', '--map_file', map_file, '--seed', str(seed)
    ], capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"❌ Simulation failed: {result.stderr}")
        return False
    
    # Parse map dimensions from map file
    map_path = Path('..') / map_file
    with open(map_path, 'r') as f:
        map_lines = [line.rstrip() for line in f.readlines() if line.strip()]
    
    map_height = len(map_lines)
    map_width = len(map_lines[0]) if map_lines else 0
    
    print(f"📏 Map dimensions: {map_width}x{map_height}")
    
    # Parse simulation output
    print("📊 Parsing simulation data...")
    simulation_data = parse_simulation_output(result.stdout, map_width, map_height)
    
    if not simulation_data:
        print("❌ No simulation data found")
        return False
    
    print(f"✅ Parsed {len(simulation_data)} ticks of data")
    
    # Create visualization
    print("🎬 Creating visualization...")
    viz = ExplorationVisualizer(simulation_data, map_width, map_height)
    viz.create_animation(output_gif, interval=300)
    
    return True

def main():
    parser = argparse.ArgumentParser(description='Create GIF visualization of multi-robot exploration')
    parser.add_argument('--map_file', default='maps/sample_room.map', help='Map file to simulate')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    parser.add_argument('--output', default='exploration.gif', help='Output GIF filename')
    
    args = parser.parse_args()
    
    # Check dependencies
    try:
        import matplotlib
        import matplotlib.animation
        print("✅ matplotlib available")
    except ImportError:
        print("❌ matplotlib required: pip install matplotlib")
        sys.exit(1)
    
    # Create GIF
    success = run_simulation_and_create_gif(args.map_file, args.seed, args.output)
    
    if success:
        print(f"🎉 Successfully created: {args.output}")
    else:
        print("❌ Failed to create visualization")
        sys.exit(1)

if __name__ == '__main__':
    main()