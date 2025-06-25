#!/usr/bin/env python3
"""
Create demo GIF for multi-robot exploration showing individual robot maps
"""

import subprocess
import matplotlib
matplotlib.use('Agg')  # Use headless backend for GIF creation
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import os
import sys
import re

def run_simulation_from_root():
    """Run simulation from the project root directory"""
    print("🚀 Running complex room simulation...")
    
    # Change to project root
    os.chdir('..')
    
    # Run the simulation
    result = subprocess.run([
        './run_simulation.sh', '--map_file', 'maps/sample_room.map', '--seed', '42'
    ], capture_output=True, text=True, timeout=120)
    
    print(f"Simulation completed with return code: {result.returncode}")
    print(f"Output length: {len(result.stdout)} characters")
    
    if result.returncode != 0:
        print("Stderr:", result.stderr[:500])
        return None
    
    return result.stdout

def parse_simulation_data(output_text):
    """Parse simulation output to extract robot positions and maps"""
    lines = output_text.split('\n')
    frames = []
    current_frame = None
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Parse tick header
        if line.startswith('=== Tick'):
            # Save previous frame if it exists
            if current_frame and current_frame['robots']:
                frames.append(current_frame)
            
            tick_num = int(line.split()[2])
            current_frame = {
                'tick': tick_num,
                'robots': {}
            }
            
        # Parse robot position and phase
        elif line.startswith('Robot') and 'pos=' in line and 'phase=' in line:
            robot_part, data_part = line.split(': ', 1)
            robot_id = int(robot_part.split()[1])
            
            # Parse position and phase
            pos_match = re.search(r'pos=\((\d+),\s*(\d+)\)', data_part)
            phase_match = re.search(r'phase=(\w+)', data_part)
            
            if pos_match and phase_match:
                x, y = int(pos_match.group(1)), int(pos_match.group(2))
                phase = phase_match.group(1)
                
                current_frame['robots'][robot_id] = {
                    'id': robot_id,
                    'x': x,
                    'y': y,
                    'phase': phase,
                    'map': []
                }
        
        # Parse robot map
        elif line.startswith("Robot") and "'s map:" in line:
            robot_id = int(line.split()[1].rstrip("'s"))
            
            # Read map lines
            i += 1
            map_lines = []
            while i < len(lines) and not lines[i].startswith('Robot') and not lines[i].startswith('===') and lines[i].strip():
                map_line = lines[i].rstrip()
                if map_line and len(map_line) > 0:
                    map_lines.append(map_line)
                i += 1
            i -= 1  # Back up one line
            
            # Store map for this robot
            if current_frame and robot_id in current_frame['robots']:
                current_frame['robots'][robot_id]['map'] = map_lines
        
        # End of parsing - save final frame
        elif i == len(lines) - 1 and current_frame and current_frame['robots']:
            frames.append(current_frame)
        
        i += 1
    
    return frames

def create_robot_maps_gif(frames, output_file="complex_room_exploration.gif"):
    """Create animated GIF showing both robots' individual maps side by side"""
    
    if not frames:
        print("❌ No frames to animate")
        return
    
    print(f"🎬 Creating animation with {len(frames)} frames")
    
    # Fixed map bounds based on the actual map file (sample_room.map is 20x11)
    # These bounds should encompass the entire possible exploration area
    min_x, max_x = 0, 20
    min_y, max_y = 0, 11
    MAP_WIDTH = max_x - min_x
    MAP_HEIGHT = max_y - min_y
    
    # Create figure with subplots for each robot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    fig.suptitle('Multi-Robot Exploration: Individual Robot Maps', fontsize=16, fontweight='bold')
    
    def animate(frame_idx):
        ax1.clear()
        ax2.clear()
        
        if frame_idx >= len(frames):
            frame_idx = len(frames) - 1
        
        frame_data = frames[frame_idx]
        
        # Colors for robots
        robot_colors = ['#FF4444', '#4444FF']  # Red and Blue
        
        for subplot_idx, (robot_id, ax) in enumerate([(0, ax1), (1, ax2)]):
            # Set black background for fog of war effect
            ax.set_facecolor('#000000')
            
            # Only draw explored areas (fog of war effect)
            if robot_id in frame_data['robots']:
                robot_data = frame_data['robots'][robot_id]
                robot_map = robot_data['map']
                
                if robot_map:
                    # Find the robot's position to determine map offset
                    robot_pos_x, robot_pos_y = robot_data['x'], robot_data['y']
                    
                    # Find robot position in the map to calculate offset
                    robot_map_x, robot_map_y = None, None
                    for y, line in enumerate(robot_map):
                        for x, char in enumerate(line):
                            if char == 'R':
                                robot_map_x, robot_map_y = x, y
                                break
                        if robot_map_x is not None:
                            break
                    
                    # Calculate offset from map coordinates to world coordinates
                    if robot_map_x is not None and robot_map_y is not None:
                        offset_x = robot_pos_x - robot_map_x
                        offset_y = robot_pos_y - robot_map_y
                        
                        # Draw only the cells that have been explored
                        for y, line in enumerate(robot_map):
                            for x, char in enumerate(line):
                                # Only draw if this cell has been explored (not space/unexplored)
                                if char in ['#', '.', 'R']:
                                    # Determine cell color based on character
                                    if char == '#':
                                        color = '#404040'  # Gray for obstacles
                                    elif char == '.':
                                        color = '#C0C0C0'  # Light gray for explored empty space
                                    elif char == 'R':
                                        color = robot_colors[robot_id]  # Robot color for robot position
                                    
                                    # Calculate world coordinates using offset
                                    world_x = x + offset_x
                                    world_y = y + offset_y
                                    
                                    rect = Rectangle((world_x, world_y), 1, 1, 
                                                   facecolor=color, edgecolor='white', linewidth=0.5)
                                    ax.add_patch(rect)
            
            # Set fixed bounds for stable viewport
            ax.set_xlim(min_x, max_x)
            ax.set_ylim(min_y, max_y)
            ax.set_aspect('equal')
            
            # Set title with robot info
            if robot_id in frame_data['robots']:
                robot_data = frame_data['robots'][robot_id]
                ax.set_title(f'Robot {robot_id} - {robot_data["phase"]}\\n'
                           f'Position: ({robot_data["x"]}, {robot_data["y"]})', 
                           fontsize=12, fontweight='bold', color=robot_colors[robot_id])
            else:
                # No data for this robot
                ax.set_title(f'Robot {robot_id} - No Data', fontsize=12, color='gray')
            
            ax.set_xticks([])
            ax.set_yticks([])
        
        # Add tick info
        fig.suptitle(f'Multi-Robot Exploration - Tick {frame_data["tick"]}\\n'
                    f'Algorithm: Iterative Boundary Trace & Coordinated Sweep', 
                    fontsize=14, fontweight='bold')
    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, frames=len(frames), 
                                 interval=500, repeat=True, blit=False)
    
    # Save GIF
    print(f"💾 Saving GIF: {output_file}")
    writer = animation.PillowWriter(fps=2)
    anim.save(output_file, writer=writer)
    print(f"✅ Created: {output_file}")
    
    plt.close()
    return output_file

def main():
    print("🎬 Multi-Robot Exploration Demo GIF Creator")
    print("=" * 50)
    
    # Run simulation
    output = run_simulation_from_root()
    if not output:
        print("❌ Failed to get simulation output")
        return
    
    # Parse data
    frames = parse_simulation_data(output)
    if not frames:
        print("❌ No animation frames found")
        return
    
    print(f"✅ Parsed {len(frames)} animation frames")
    
    # Create GIF
    gif_file = create_robot_maps_gif(frames)
    
    print("🎉 Demo GIF creation complete!")
    print(f"📁 Generated: python_viz/{gif_file}")

if __name__ == '__main__':
    main()