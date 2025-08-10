#!/usr/bin/env python3
"""
Generate SVG diagrams from Mermaid source files.

Usage:
    python3 generate_diagrams.py

Requirements:
    npm install -g @mermaid-js/mermaid-cli
"""

import subprocess
import os
from pathlib import Path

def generate_svg_from_mermaid(mmd_file, svg_file):
    """Generate SVG from Mermaid file using mmdc CLI."""
    try:
        # Try mmdc first (if installed globally)
        result = subprocess.run([
            'mmdc', '-i', str(mmd_file), '-o', str(svg_file),
            '--theme', 'default', '--backgroundColor', 'white'
        ], capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f" Generated {svg_file}")
            return True
        else:
            print(f" mmdc failed for {mmd_file}: {result.stderr}")
            return False
            
    except FileNotFoundError:
        print(" mmdc not found. Install with: npm install -g @mermaid-js/mermaid-cli")
        return False

def main():
    """Generate all diagrams."""
    diagrams_dir = Path(__file__).parent / "diagrams"
    
    # Diagram files
    diagrams = [
        ("architecture-overview.mmd", "architecture-overview.svg"),
        ("performance-dataflow.mmd", "performance-dataflow.svg"), 
        ("extension-patterns.mmd", "extension-patterns.svg")
    ]
    
    print("Generating VAMP architecture diagrams...")
    
    success_count = 0
    for mmd_file, svg_file in diagrams:
        mmd_path = diagrams_dir / mmd_file
        svg_path = diagrams_dir / svg_file
        
        if mmd_path.exists():
            if generate_svg_from_mermaid(mmd_path, svg_path):
                success_count += 1
        else:
            print(f" Source file not found: {mmd_path}")
    
    print(f"\nGenerated {success_count}/{len(diagrams)} diagrams successfully.")
    
    if success_count == len(diagrams):
        print("\n All diagrams generated! You can now view them in:")
        print("  - README.md")
        print("  - ARCHITECTURE.md") 
        print("  - GitHub (automatic SVG rendering)")

if __name__ == "__main__":
    main()
