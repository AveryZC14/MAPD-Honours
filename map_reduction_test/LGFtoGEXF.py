import re
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

# --- Configuration ---
INPUT_FILE = './../instances/custom/tiny/tiny_c1.lgf'
OUTPUT_GEXF = 'visualisation/tiny_c1.gexf'
COORD_MULTIPLIER = 50
# ---------------------

def convert_lgf_to_gexf(input_path):
    output_dir = os.path.dirname(OUTPUT_GEXF)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    nodes = []
    edges = []
    current_section = None
    coord_pattern = re.compile(r'\(([^,]+),\s*([^)]+)\)')

    if not os.path.exists(input_path):
        print(f"Error: Input file '{input_path}' not found.")
        return

    with open(input_path, 'r') as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'): continue
        
        if line.startswith('@'):
            if line.startswith('@nodes'): current_section = 'nodes'
            elif line.startswith('@edges') or line.startswith('@arcs'): current_section = 'edges'
            else: current_section = None
            continue

        # Skip header lines (like "label" or "label weight")
        if line.lower() in ['label', 'label weight', 'source target weight']:
            continue

        parts = line.split()
        
        if current_section == 'nodes':
            match = coord_pattern.search(line)
            # Use the first part (e.g. "0:(-1,-1)") as both ID and Label for clarity
            node_id = parts[0]
            x, y = (float(match.group(1)) * COORD_MULTIPLIER, float(match.group(2)) * COORD_MULTIPLIER) if match else (0.0, 0.0)
            nodes.append({'id': node_id, 'label': node_id, 'x': y, 'y': x})

        elif current_section == 'edges' and len(parts) >= 2:
            source = parts[0]
            target = parts[1]
            # Grab weight if it exists, otherwise default to 1.0
            weight = parts[2] if len(parts) > 2 else "1.0"
            edges.append({'source': source, 'target': target, 'weight': weight})

    # Build GEXF
    ns_viz = "http://www.gexf.net/1.2draft/viz"
    gexf = ET.Element('gexf', xmlns='http://www.gexf.net/1.2draft', version='1.2')
    gexf.set("xmlns:viz", ns_viz)
    
    # We set weight as an attribute if you want to use it for layout/ranking
    graph = ET.SubElement(gexf, 'graph', defaultedgetype='directed')
    
    # Add Nodes
    node_elements = ET.SubElement(graph, 'nodes')
    for n in nodes:
        node_xml = ET.SubElement(node_elements, 'node', id=n['id'], label=n['label'])
        ET.SubElement(node_xml, f'{{{ns_viz}}}position', x=str(n['x']), y=str(n['y']), z="0.0")

    # Add Edges
    edge_elements = ET.SubElement(graph, 'edges')
    for i, e in enumerate(edges):
        # We assign 'weight' for Gephi's math and 'label' for Gephi's display
        edge_xml = ET.SubElement(edge_elements, 'edge', 
                                 id=str(i), 
                                 source=e['source'], 
                                 target=e['target'], 
                                 weight=e['weight'],
                                 label=f"w:{e['weight']}")

    # Write to File
    xml_str = minidom.parseString(ET.tostring(gexf)).toprettyxml(indent="  ")
    with open(OUTPUT_GEXF, 'w') as f:
        f.write(xml_str)

    print(f"Success! Generated {OUTPUT_GEXF}")
    print(f"Stats: {len(nodes)} nodes, {len(edges)} edges.")

if __name__ == "__main__":
    convert_lgf_to_gexf(INPUT_FILE)