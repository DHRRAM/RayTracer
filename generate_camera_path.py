#!/usr/bin/env python3
"""
Generate camera keyframes with noise for handheld/shaky camera effect.
Outputs XML keyframe tags that can be pasted into your scene file.
"""

import math
import random

def perlin_noise_1d(x, seed=0):
    """Simple 1D Perlin-like noise approximation"""
    random.seed(int(x * 1000) + seed)
    return random.uniform(-1, 1)

def smooth_noise(t, octaves=3, persistence=0.5, seed=0):
    """Multi-octave smooth noise"""
    total = 0.0
    amplitude = 1.0
    max_value = 0.0
    
    for i in range(octaves):
        frequency = 2 ** i
        total += perlin_noise_1d(t * frequency, seed + i) * amplitude
        max_value += amplitude
        amplitude *= persistence
    
    return total / max_value

def lerp(a, b, t):
    """Linear interpolation"""
    return a + (b - a) * t

def generate_noisy_camera_path(
    start_pos, end_pos,
    start_time=0.0, end_time=5.0,
    num_keyframes=20,
    noise_amplitude=0.5,
    noise_frequency=2.0,
    target_pos=(0, 0, 0),
    start_fov=45.0,
    end_fov=45.0,
    seed=42
):
    """
    Generate camera keyframes with noise along a path.
    
    Args:
        start_pos: (x, y, z) starting position
        end_pos: (x, y, z) ending position
        start_time: animation start time
        end_time: animation end time
        num_keyframes: number of keyframes to generate
        noise_amplitude: how much the camera meanders (in world units)
        noise_frequency: how quickly it meanders (higher = more jittery)
        target_pos: where camera looks
        start_fov: initial field of view
        end_fov: final field of view
        seed: random seed for reproducibility
    """
    
    print(f"<!-- Generated camera path: {num_keyframes} keyframes -->")
    print(f"<!-- Path from {start_pos} to {end_pos} -->")
    print(f"<!-- Noise amplitude: {noise_amplitude}, frequency: {noise_frequency} -->")
    print()
    
    # Calculate perpendicular directions for noise
    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    dz = end_pos[2] - start_pos[2]
    
    # Create perpendicular vectors for lateral/vertical noise
    # Perpendicular in XZ plane (horizontal meander)
    perp_xz = (-dz, 0, dx)
    len_xz = math.sqrt(perp_xz[0]**2 + perp_xz[2]**2)
    if len_xz > 0:
        perp_xz = (perp_xz[0]/len_xz, 0, perp_xz[2]/len_xz)
    else:
        perp_xz = (1, 0, 0)
    
    # Perpendicular in vertical direction
    perp_y = (0, 1, 0)
    
    for i in range(num_keyframes):
        t = i / max(1, num_keyframes - 1)  # 0.0 to 1.0
        time = lerp(start_time, end_time, t)
        
        # Base position along straight line
        base_x = lerp(start_pos[0], end_pos[0], t)
        base_y = lerp(start_pos[1], end_pos[1], t)
        base_z = lerp(start_pos[2], end_pos[2], t)
        
        # Add noise in perpendicular directions
        noise_xz = smooth_noise(t * noise_frequency, octaves=3, seed=seed) * noise_amplitude
        noise_y = smooth_noise(t * noise_frequency, octaves=3, seed=seed + 100) * noise_amplitude * 0.5
        noise_roll = smooth_noise(t * noise_frequency, octaves=2, seed=seed + 200) * noise_amplitude * 0.3
        
        # Apply noise
        pos_x = base_x + perp_xz[0] * noise_xz
        pos_y = base_y + noise_y
        pos_z = base_z + perp_xz[2] * noise_xz
        
        # FOV variation
        fov = lerp(start_fov, end_fov, t)
        fov_noise = smooth_noise(t * noise_frequency * 0.5, octaves=2, seed=seed + 300) * 2.0
        fov += fov_noise
        
        # Output XML keyframe
        print(f'    <keyframe time="{time:.3f}">')
        print(f'        <position>{pos_x:.4f} {pos_y:.4f} {pos_z:.4f}</position>')
        print(f'        <target>{target_pos[0]} {target_pos[1]} {target_pos[2]}</target>')
        print(f'        <fov>{fov:.2f}</fov>')
        print(f'    </keyframe>')
        print()

def generate_orbital_path_with_noise(
    radius=10.0,
    height=5.0,
    start_angle=0.0,
    end_angle=360.0,
    start_time=0.0,
    end_time=5.0,
    num_keyframes=30,
    noise_amplitude=0.3,
    noise_frequency=3.0,
    target_pos=(0, 0, 0),
    fov=45.0,
    seed=42
):
    """
    Generate camera keyframes that orbit around a target with noise.
    """
    
    print(f"<!-- Generated orbital camera path: {num_keyframes} keyframes -->")
    print(f"<!-- Orbit radius: {radius}, height: {height} -->")
    print(f"<!-- Noise amplitude: {noise_amplitude}, frequency: {noise_frequency} -->")
    print()
    
    for i in range(num_keyframes):
        t = i / max(1, num_keyframes - 1)
        time = lerp(start_time, end_time, t)
        angle_deg = lerp(start_angle, end_angle, t)
        angle_rad = math.radians(angle_deg)
        
        # Base orbital position
        base_x = target_pos[0] + radius * math.cos(angle_rad)
        base_z = target_pos[2] + radius * math.sin(angle_rad)
        base_y = target_pos[1] + height
        
        # Add noise
        noise_radial = smooth_noise(t * noise_frequency, octaves=3, seed=seed) * noise_amplitude
        noise_y = smooth_noise(t * noise_frequency, octaves=3, seed=seed + 100) * noise_amplitude
        noise_tangent = smooth_noise(t * noise_frequency, octaves=3, seed=seed + 200) * noise_amplitude
        
        # Apply noise in cylindrical coordinates
        actual_radius = radius + noise_radial
        pos_x = target_pos[0] + actual_radius * math.cos(angle_rad + noise_tangent * 0.1)
        pos_z = target_pos[2] + actual_radius * math.sin(angle_rad + noise_tangent * 0.1)
        pos_y = base_y + noise_y
        
        # FOV variation
        fov_current = fov + smooth_noise(t * noise_frequency * 0.5, octaves=2, seed=seed + 300) * 3.0
        
        print(f'    <keyframe time="{time:.3f}">')
        print(f'        <position>{pos_x:.4f} {pos_y:.4f} {pos_z:.4f}</position>')
        print(f'        <target>{target_pos[0]} {target_pos[1]} {target_pos[2]}</target>')
        print(f'        <fov>{fov_current:.2f}</fov>')
        print(f'    </keyframe>')
        print()


if __name__ == "__main__":
    print("=== Camera Path Generator ===\n")
    
    # Example 1: Straight path with handheld shake
    print("<!-- EXAMPLE 1: Straight path with handheld camera shake -->")
    generate_noisy_camera_path(
        start_pos=(15, 5, 0),
        end_pos=(-15, 5, 0),
        start_time=0.0,
        end_time=5.0,
        num_keyframes=25,
        noise_amplitude=0.4,      # Moderate shake
        noise_frequency=4.0,      # Medium speed wobble
        target_pos=(0, 0, 0),
        start_fov=45.0,
        end_fov=45.0,
        seed=42
    )
    
    print("\n" + "="*60 + "\n")
    
    # Example 2: Orbital camera with subtle drift
    print("<!-- EXAMPLE 2: Orbital camera with subtle drift -->")
    generate_orbital_path_with_noise(
        radius=12.0,
        height=6.0,
        start_angle=0.0,
        end_angle=360.0,
        start_time=0.0,
        end_time=8.0,
        num_keyframes=40,
        noise_amplitude=0.2,      # Subtle drift
        noise_frequency=2.0,
        target_pos=(0, 2, 0),
        fov=45.0,
        seed=123
    )
    
    print("\n" + "="*60 + "\n")
    print("<!-- Copy the keyframes above into your <camera> tag in your XML file -->")
