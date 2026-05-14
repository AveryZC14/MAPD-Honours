import csv
import re
import os

# --- Configuration ---
INPUT_FILE = './../instances/warehouseLarge/warehouseLarge_16000.lgf'
NODES_OUTPUT = 'visualisation_csv/nodes.csv'
EDGES_OUTPUT = 'visualisation_csv/edges.csv'
COORD_MULTIPLIER = 10.0  # Change this constant as needed
# ---------------------

def convert_lgf_to_csv(input_path):
    nodes = []
    edges = []
    current_section = None
    headers = []

    # Regex to extract coordinates from strings like "(12.5, 40)"
    coord_pattern = re.compile(r'\(([^,]+),\s*([^)]+)\)')

    if not os.path.exists(os.path.dirname(NODES_OUTPUT)):
        os.makedirs(os.path.dirname(NODES_OUTPUT))

    with open(input_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            # Section detection
            if line.startswith('@'):
                if line.startswith('@nodes'):
                    current_section = 'nodes'
                elif line.startswith('@edges') or line.startswith('@arcs'):
                    current_section = 'edges'
                else:
                    current_section = None
                headers = [] # Reset headers for new section
                continue

            # Skip the header line (the line immediately following @nodes or @edges)
            if not headers:
                headers = line.split()
                continue

            parts = line.split()

            if current_section == 'nodes':
                node_id = parts[0]
                # Try to find coordinates in the whole line
                match = coord_pattern.search(line)
                
                if match:
                    # Convert to float, multiply, then back to string
                    x_val = float(match.group(1)) * COORD_MULTIPLIER
                    y_val = float(match.group(2)) * COORD_MULTIPLIER
                else:
                    x_val, y_val = 0.0, 0.0
                
                nodes.append({'Id': node_id, 'Label': node_id, 'X': x_val, 'Y': y_val})

            elif current_section == 'edges':
                # LGF edges/arcs usually have Source as parts[0] and Target as parts[1]
                if len(parts) >= 2:
                    edges.append({'Source': parts[0], 'Target': parts[1]})

    # Write Nodes CSV
    with open(NODES_OUTPUT, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['Id', 'Label', 'X', 'Y'])
        writer.writeheader()
        writer.writerows(nodes)

    # Write Edges CSV
    with open(EDGES_OUTPUT, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['Source', 'Target'])
        writer.writeheader()
        writer.writerows(edges)

    print(f"Success! Processed {len(nodes)} nodes and {len(edges)} edges.")

if __name__ == "__main__":
    convert_lgf_to_csv(INPUT_FILE)