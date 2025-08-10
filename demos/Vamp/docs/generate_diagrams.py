#!/usr/bin/env python3
"""
Generate SVG diagrams from Mermaid source files.

Usage:
    python3 generate_diagrams.py

Requirements:
    npm install -g @mermaid-js/mermaid-cli
"""

import subprocess
from pathlib import Path

def generate_svg(mmd_file, svg_file):
    """Generate SVG from Mermaid file."""
    try:
        result = subprocess.run([
            'mmdc', '-i', str(mmd_file), '-o', str(svg_file),
            '--theme', 'default', '--backgroundColor', 'white'
        ], capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"Generated {svg_file}")
            return True
        else:
            print(f"Failed: {mmd_file}")
            return False
            
    except FileNotFoundError:
        print("mmdc not found. Install: npm install -g @mermaid-js/mermaid-cli")
        return False

def main():
    """Generate all diagrams."""
    diagrams_dir = Path(__file__).parent / "diagrams"
    
    diagrams = [
        "01-system-architecture-overview",
        "02-class-architecture-patterns", 
        "03-planning-request-sequence",
        "04-simd-performance-architecture"
    ]
    
    print("Generating VAMP architecture diagrams...")
    
    success_count = 0
    for diagram in diagrams:
        mmd_file = diagrams_dir / f"{diagram}.mmd"
        svg_file = diagrams_dir / f"{diagram}.svg"
        
        if mmd_file.exists():
            if generate_svg(mmd_file, svg_file):
                success_count += 1
        else:
            print(f"Not found: {mmd_file}")
    
    print(f"Generated {success_count}/{len(diagrams)} diagrams")

if __name__ == "__main__":
    main()
