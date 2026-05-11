import sys
import re

def convert_lgf_to_dot(input_file, output_file):
    nodes = [] # List of tuples: (id, label, x, y)
    edges = []
    
    # Regex to find numbers inside parentheses like (12, 45)
    coord_pattern = re.compile(r"\((\d+),(\d+)\)")
    scale_factor = 100  # Spreads nodes out so labels don't overlap immediately

    with open(input_file, 'r') as f:
        lines = f.readlines()

    current_section = None
    
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
            
        if line.startswith('@nodes'):
            current_section = 'nodes'
            continue
        elif line.startswith('@arcs') or line.startswith('@edges'):
            current_section = 'edges'
            continue
        elif line.startswith('@'):
            current_section = 'other'
            continue

        parts = line.split()
        
        if current_section == 'nodes' and len(parts) >= 2:
            node_id = parts[0]
            label = parts[1]
            
            # Extract x and y from the label string "(x,y)"
            match = coord_pattern.search(label)
            if match:
                x_val = float(match.group(1)) * scale_factor
                y_val = float(match.group(2)) * scale_factor
                nodes.append((node_id, label, x_val, y_val))
            else:
                # Fallback if no coordinates found
                nodes.append((node_id, label, 0, 0))
                
        elif current_section == 'edges' and len(parts) >= 2:
            edges.append((parts[0], parts[1]))

    # Write to DOT format with spatial attributes
    with open(output_file, 'w') as f:
        f.write("digraph G {\n")
        f.write('    node [shape=circle];\n') 
        
        # Define nodes with x, y coordinates
        for node_id, label, x, y in nodes:
            # Gephi recognizes 'x' and 'y' attributes for initial placement
            f.write(f'    "{node_id}" [label="{label}", x={x}, y={y}];\n')
            
        # Define edges
        for u, v in edges:
            f.write(f'    "{u}" -> "{v}";\n')
        f.write("}\n")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python convert.py input.lgf output.dot")
    else:
        convert_lgf_to_dot(sys.argv[1], sys.argv[2])
        print(f"Converted {sys.argv[1]} to {sys.argv[2]} with coordinates.")