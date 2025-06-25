#!/usr/bin/env python3
"""
Simple test to create a basic GIF from simulation output
"""

import subprocess
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import re

def run_simulation_and_capture():
    """Run simulation and capture enough output for visualization"""
    print("🚀 Running simulation...")
    
    # Run simulation and capture output
    process = subprocess.Popen([
        '../run_simulation.sh', '--map_file', 'maps/simple_corridor.map', '--seed', '42'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    output_lines = []
    try:
        # Read lines as they come
        for line in iter(process.stdout.readline, ''):
            output_lines.append(line.rstrip())
            if len(output_lines) > 200:  # Limit output
                break
        
        process.terminate()
        process.wait(timeout=1)
    except:
        process.kill()
    
    return '\n'.join(output_lines)

def parse_simple_data(output_text):
    """Parse basic tick and robot position data"""
    lines = output_text.split('\n')
    data = []
    current_tick = None
    robots = {}
    
    for line in lines:
        line = line.strip()
        
        # Parse tick
        if line.startswith('=== Tick'):
            if current_tick is not None and robots:
                data.append({
                    'tick': current_tick,
                    'robots': list(robots.values())
                })
            
            current_tick = int(line.split()[2])
            robots = {}
        
        # Parse robot positions  
        elif line.startswith('Robot') and 'pos=' in line:
            parts = line.split()
            robot_id = int(parts[1].rstrip(':'))
            
            # Extract position
            pos_part = [p for p in parts if p.startswith('pos=')][0]
            pos_coords = pos_part.split('=')[1].strip('()')
            x, y = map(int, pos_coords.split(','))
            
            robots[robot_id] = {
                'id': robot_id,
                'position': (x, y)
            }
    
    # Add final tick
    if current_tick is not None and robots:
        data.append({
            'tick': current_tick,
            'robots': list(robots.values())
        })
    
    return data

def create_simple_gif(data, map_width=12, map_height=5, output_file="simple_exploration.gif"):
    """Create a simple animated GIF showing robot movement"""
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    def animate(frame):
        ax.clear()
        
        if frame >= len(data):
            frame = len(data) - 1
            
        tick_data = data[frame]
        
        # Draw grid background
        for x in range(map_width):
            for y in range(map_height):
                rect = Rectangle((x, map_height - y - 1), 1, 1, 
                               facecolor='lightgray', edgecolor='black', linewidth=0.5)
                ax.add_patch(rect)
        
        # Draw robots
        colors = ['red', 'blue']
        for robot in tick_data['robots']:
            x, y = robot['position']
            robot_id = robot['id']
            
            ax.scatter(x + 0.5, map_height - y - 0.5, 
                      c=colors[robot_id], s=300, marker='o', 
                      edgecolors='black', linewidth=2, zorder=10)
            
            ax.text(x + 0.5, map_height - y - 0.5, str(robot_id), 
                   ha='center', va='center', fontweight='bold', 
                   fontsize=12, color='white', zorder=11)
        
        ax.set_xlim(0, map_width)
        ax.set_ylim(0, map_height)
        ax.set_aspect('equal')
        ax.set_title(f'Multi-Robot Exploration - Tick {tick_data["tick"]}', 
                    fontsize=14, fontweight='bold')
        ax.set_xticks([])
        ax.set_yticks([])
    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, frames=len(data), 
                                 interval=500, repeat=True)
    
    # Save as GIF
    print(f"💾 Saving GIF: {output_file}")
    anim.save(output_file, writer='pillow', fps=2)
    print(f"✅ Created: {output_file}")
    
    return anim

def main():
    print("🎬 Simple Multi-Robot Exploration GIF Creator")
    print("=" * 45)
    
    # Run simulation
    output = run_simulation_and_capture()
    
    if not output:
        print("❌ No simulation output captured")
        return
    
    print(f"📊 Captured {len(output.split('\\n'))} lines of output")
    
    # Parse data
    data = parse_simple_data(output)
    
    if not data:
        print("❌ No simulation data parsed")
        return
        
    print(f"✅ Parsed {len(data)} ticks of data")
    
    # Create GIF
    create_simple_gif(data, output_file="simple_corridor_exploration.gif")
    
    print("🎉 GIF creation complete!")

if __name__ == '__main__':
    main()