#!/usr/bin/env python3
"""
Automatically add or replace camera keyframes in an XML scene file.
This script will parse your XML, remove existing keyframes, and insert new ones.
"""

import math
import random
import re
import sys
from pathlib import Path

def perlin_noise_1d(x, seed=0):
    """Simple 1D Perlin-like noise approximation"""
    random.seed(int(x * 1000) + seed)
    return random.uniform(-1, 1)

def smooth_noise(t, octaves=3, persistence=0.8, seed=0):
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

def generate_keyframes(
    start_pos, end_pos,
    start_time=0.0, end_time=5.0,
    num_keyframes=20,
    noise_amplitude=0.5,
    noise_frequency=2.0,
    target_pos=(0, 0, 0),
    end_target_pos=None,  # If None, target stays fixed
    start_fov=45.0,
    end_fov=45.0,
    seed=42
):
    """
    Generate camera keyframes with noise along a path.
    Returns a list of keyframe XML strings.
    
    Args:
        target_pos: Starting target position (or fixed target if end_target_pos is None)
        end_target_pos: Ending target position (None = target doesn't move)
    """
    
    keyframes = []
    
    # If end_target_pos not specified, target stays fixed
    if end_target_pos is None:
        end_target_pos = target_pos
    
    # Calculate perpendicular directions for noise
    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    dz = end_pos[2] - start_pos[2]
    
    # Create perpendicular vectors for lateral/vertical noise
    perp_xz = (-dz, 0, dx)
    len_xz = math.sqrt(perp_xz[0]**2 + perp_xz[2]**2)
    if len_xz > 0:
        perp_xz = (perp_xz[0]/len_xz, 0, perp_xz[2]/len_xz)
    else:
        perp_xz = (1, 0, 0)
    
    for i in range(num_keyframes):
        t = i / max(1, num_keyframes - 1)
        time = lerp(start_time, end_time, t)
        
        # Base position along straight line
        base_x = lerp(start_pos[0], end_pos[0], t)
        base_y = lerp(start_pos[1], end_pos[1], t)
        base_z = lerp(start_pos[2], end_pos[2], t)
        
        # DEBUG: Print first keyframe
        if i == 0:
            print(f"  DEBUG: First keyframe base position: ({base_x}, {base_y}, {base_z})")
        
        # Add noise
        noise_xz = smooth_noise(t * noise_frequency, octaves=3, seed=seed) * noise_amplitude
        noise_y = smooth_noise(t * noise_frequency, octaves=3, seed=seed + 100) * noise_amplitude * 0.5
        
        pos_x = base_x + perp_xz[0] * noise_xz
        pos_y = base_y + noise_y
        pos_z = base_z + perp_xz[2] * noise_xz
        
        # Animate target position
        target_x = lerp(target_pos[0], end_target_pos[0], t)
        target_y = lerp(target_pos[1], end_target_pos[1], t)
        target_z = lerp(target_pos[2], end_target_pos[2], t)
        
        # FOV variation - constant FOV (no noise)
        fov = lerp(start_fov, end_fov, t)
        
        # Build keyframe XML with attributes (matching base camera format)
        kf = f'\t<keyframe time="{time:.3f}">\n'
        kf += f'\t\t<position x="{pos_x:.4f}" y="{pos_y:.4f}" z="{pos_z:.4f}"/>\n'
        kf += f'\t\t<target x="{target_x:.4f}" y="{target_y:.4f}" z="{target_z:.4f}"/>\n'
        kf += f'\t\t<fov value="{fov:.2f}"/>\n'
        kf += f'\t</keyframe>'
        keyframes.append(kf)
    
    return keyframes

def generate_orbital_keyframes(
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
    """Generate orbital camera keyframes with noise"""
    
    keyframes = []
    
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
        
        actual_radius = radius + noise_radial
        pos_x = target_pos[0] + actual_radius * math.cos(angle_rad + noise_tangent * 0.1)
        pos_z = target_pos[2] + actual_radius * math.sin(angle_rad + noise_tangent * 0.1)
        pos_y = base_y + noise_y
        
        fov_current = fov + smooth_noise(t * noise_frequency * 0.5, octaves=2, seed=seed + 300) * 3.0
        
        kf = f'\t<keyframe time="{time:.3f}">\n'
        kf += f'\t\t<position x="{pos_x:.4f}" y="{pos_y:.4f}" z="{pos_z:.4f}"/>\n'
        kf += f'\t\t<target x="{target_pos[0]:.4f}" y="{target_pos[1]:.4f}" z="{target_pos[2]:.4f}"/>\n'
        kf += f'\t\t<fov value="{fov_current:.2f}"/>\n'
        kf += f'\t</keyframe>'
        keyframes.append(kf)
    
    return keyframes

def update_xml_with_keyframes(xml_file, keyframes, backup=True):
    """
    Update XML file with new camera keyframes.
    Removes existing keyframes and adds new ones.
    Also updates base camera position/target to match first keyframe.
    """
    
    xml_path = Path(xml_file)
    if not xml_path.exists():
        print(f"Error: File not found: {xml_file}")
        return False
    
    # Read the XML file
    with open(xml_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Create backup if requested
    if backup:
        backup_path = xml_path.with_suffix('.xml.bak')
        with open(backup_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✓ Backup created: {backup_path}")
    
    # Find the camera section
    camera_match = re.search(r'<camera[^>]*>.*?</camera>', content, re.DOTALL)
    if not camera_match:
        print("Error: No <camera> section found in XML file")
        return False
    
    original_camera_section = camera_match.group(0)
    camera_section = original_camera_section
    
    # Extract first keyframe position and target for base camera settings
    first_kf_match = re.search(r'<position>([\d\.\-\s]+)</position>\s*<target>([\d\.\-\s]+)</target>', keyframes[0])
    if first_kf_match:
        first_pos = first_kf_match.group(1).strip().split()
        first_target = first_kf_match.group(2).strip().split()
        
        # Update base position and target to match first keyframe
        camera_section = re.sub(
            r'<position[^>]*x="[^"]*"[^>]*y="[^"]*"[^>]*z="[^"]*"[^>]*/?>',
            f'<position x="{first_pos[0]}" y="{first_pos[1]}" z="{first_pos[2]}"/>',
            camera_section
        )
        camera_section = re.sub(
            r'<target[^>]*x="[^"]*"[^>]*y="[^"]*"[^>]*z="[^"]*"[^>]*/?>',
            f'<target x="{first_target[0]}" y="{first_target[1]}" z="{first_target[2]}"/>',
            camera_section
        )
    
    # Remove existing keyframes
    camera_without_keyframes = re.sub(
        r'\s*<keyframe[^>]*>.*?</keyframe>\s*',
        '',
        camera_section,
        flags=re.DOTALL
    )
    
    # Find the closing </camera> tag
    closing_tag_match = re.search(r'(</camera>)', camera_without_keyframes)
    if not closing_tag_match:
        print("Error: Malformed camera section")
        return False
    
    # Insert new keyframes before the closing tag
    insert_pos = closing_tag_match.start()
    
    # Add comment and keyframes
    keyframe_block = '\n\t<!-- Generated camera animation keyframes -->\n'
    keyframe_block += '\n'.join(keyframes) + '\n'
    
    new_camera_section = (
        camera_without_keyframes[:insert_pos] +
        keyframe_block +
        camera_without_keyframes[insert_pos:]
    )
    
    # Replace the old camera section with the new one (use original for search)
    new_content = content.replace(original_camera_section, new_camera_section)
    
    # Write the updated XML
    with open(xml_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    
    print(f"✓ Updated {xml_file} with {len(keyframes)} keyframes")
    return True

def main():
    # Configuration - EDIT THESE VALUES
    xml_file = "out/build/x64-release/RayTracer/cerealBar.xml"
    
    # Animation type: 'linear' or 'orbital'
    animation_type = 'linear'
    
    # Linear path settings
    linear_config = {
        'start_pos': (-1.67664, 0.676916, 4.69288),
        'end_pos': (-1.67664, 0.676916, 2.89288), 
        'start_time': 0.0,
        'end_time': 10.0,
        'num_keyframes': 10,
        'noise_amplitude': 0.1,
        'noise_frequency': 0.1,
        'target_pos': (0.879957, 0.302181, 0),
        'end_target_pos': (0.879957, 0.302181, -1.64971),
        'start_fov': 24.0,
        'end_fov': 24.0,
        'seed': 42
    }
    
    # Orbital path settings
    orbital_config = {
        'radius': 12.0,
        'height': 6.0,
        'start_angle': 0.0,
        'end_angle': 360.0,
        'start_time': 0.0,
        'end_time': 8.0,
        'num_keyframes': 40,
        'noise_amplitude': 0.2,
        'noise_frequency': 2.0,
        'target_pos': (0, 2, 0),
        'fov': 45.0,
        'seed': 123
    }
    
    # Generate keyframes based on type
    print(f"Generating {animation_type} camera animation...")
    
    if animation_type == 'linear':
        keyframes = generate_keyframes(**linear_config)
        print(f"  Path: {linear_config['start_pos']} → {linear_config['end_pos']}")
        print(f"  Duration: {linear_config['end_time']}s @ {linear_config['num_keyframes']} keyframes")
        print(f"  Noise: amplitude={linear_config['noise_amplitude']}, frequency={linear_config['noise_frequency']}")
    elif animation_type == 'orbital':
        keyframes = generate_orbital_keyframes(**orbital_config)
        print(f"  Orbit: radius={orbital_config['radius']}, height={orbital_config['height']}")
        print(f"  Duration: {orbital_config['end_time']}s @ {orbital_config['num_keyframes']} keyframes")
        print(f"  Noise: amplitude={orbital_config['noise_amplitude']}, frequency={orbital_config['noise_frequency']}")
    else:
        print(f"Error: Unknown animation type '{animation_type}'")
        return 1
    
    # Update the XML file
    print()
    if update_xml_with_keyframes(xml_file, keyframes, backup=True):
        print()
        print("✓ Success! Your XML file has been updated.")
        print(f"  Original backed up with .bak extension")
        print(f"  Ready to render with: RayTracer.exe {Path(xml_file).name} --anim-fps=24")
        return 0
    else:
        print()
        print("✗ Failed to update XML file")
        return 1

if __name__ == "__main__":
    sys.exit(main())

# ==============================================================================
# EXAMPLE CONFIGURATIONS FOR ANIMATED TARGETS
# ==============================================================================

"""
EXAMPLE 1: Fixed Target (camera moves, target stays still)
-----------------------------------------------------------
linear_config = {
    'start_pos': (10, 5, 0),
    'end_pos': (-10, 5, 0),
    'target_pos': (0, 2, 0),        # Looking at origin
    'end_target_pos': None,         # Target doesn't move
    ...
}

EXAMPLE 2: Pan Shot (camera and target both move together)
-----------------------------------------------------------
linear_config = {
    'start_pos': (10, 5, 5),
    'end_pos': (-10, 5, 5),
    'target_pos': (10, 2, 0),       # Start looking at right object
    'end_target_pos': (-10, 2, 0),  # End looking at left object
    ...
}

EXAMPLE 3: Dolly Zoom / Reveal Effect (camera pulls back, target rises)
------------------------------------------------------------------------
linear_config = {
    'start_pos': (0, 2, 5),         # Close to scene
    'end_pos': (0, 8, 15),          # Pull back and up
    'target_pos': (0, 0, 0),        # Start looking at ground
    'end_target_pos': (0, 3, 0),    # End looking higher up
    'start_fov': 30.0,              # Narrow FOV when close
    'end_fov': 60.0,                # Wide FOV when far
    ...
}

EXAMPLE 4: Tracking Shot (follow moving object)
------------------------------------------------
linear_config = {
    'start_pos': (5, 3, 8),
    'end_pos': (-5, 3, 8),
    'target_pos': (5, 1, 0),        # Track object from right
    'end_target_pos': (-5, 1, 0),   # To left
    ...
}

EXAMPLE 5: Dramatic Reveal (camera stays, target pans across scene)
--------------------------------------------------------------------
linear_config = {
    'start_pos': (0, 5, 10),
    'end_pos': (0, 5, 10),          # Camera doesn't move!
    'target_pos': (-5, 2, 0),       # Start looking left
    'end_target_pos': (5, 2, 0),    # End looking right
    ...
}
"""
