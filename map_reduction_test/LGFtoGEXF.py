import re
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

# --- Configuration ---
# INPUT_FILE = './../instances/warehouseLarge/warehouseLarge_16000.lgf'
# INPUT_FILE = './../instances/random/random_400.lgf'
INPUT_FILE = './../instances/custom/tiny/tiny.lgf'
INPUT_FILE = './../instances/custom/tiny/tiny_c1.lgf'
# OUTPUT_GEXF = 'visualisation/random64x64graph2.gexf'
OUTPUT_GEXF = 'visualisation/tiny.gexf'
OUTPUT_GEXF = 'visualisation/tiny_c1.gexf'
COORD_MULTIPLIER = 10
# ---------------------

def convert_lgf_to_gexf(input_path):
    # Ensure the output directory exists
    output_dir = os.path.dirname(OUTPUT_GEXF)
    if output_dir and not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
            print(f"Created directory: {output_dir}")
        except OSError as e:
            print(f"Error creating directory {output_dir}: {e}")
            return

    nodes = []
    edges = []
    current_section = None
    headers = []
    coord_pattern = re.compile(r'\(([^,]+),\s*([^)]+)\)')

    # 1. Parse the LGF file
    if not os.path.exists(input_path):
        print(f"Error: Input file '{input_path}' not found.")
        return

    with open(input_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            if line.startswith('@'):
                if line.startswith('@nodes'): current_section = 'nodes'
                elif line.startswith('@edges') or line.startswith('@arcs'): current_section = 'edges'
                else: current_section = None
                headers = []
                continue
            if not headers:
                headers = line.split()
                continue

            parts = line.split()
            if current_section == 'nodes':
                match = coord_pattern.search(line)
                # Convert to float and scale
                x, y = (float(match.group(1)) * COORD_MULTIPLIER, float(match.group(2)) * COORD_MULTIPLIER) if match else (0.0, 0.0)
                # nodes.append({'id': parts[0], 'x': x, 'y': y})
                nodes.append({'id': parts[0], 'x': y, 'y': x}) #swap x and y
            elif current_section == 'edges' and len(parts) >= 2:
                edges.append({'source': parts[0], 'target': parts[1]})

    # 2. Build GEXF XML Structure
    # We include the 'viz' namespace specifically for Gephi to recognize positions
    attr_qname = "xmlns:viz"
    ns_viz = "http://www.gexf.net/1.2draft/viz"
    
    gexf = ET.Element('gexf', xmlns='http://www.gexf.net/1.2draft', version='1.2')
    gexf.set(attr_qname, ns_viz)
    
    graph = ET.SubElement(gexf, 'graph', defaultedgetype='directed')
    
    # Add Nodes
    node_elements = ET.SubElement(graph, 'nodes')
    for n in nodes:
        node_xml = ET.SubElement(node_elements, 'node', id=n['id'], label=n['id'])
        # Adding position using the viz namespace
        ET.SubElement(node_xml, f'{{{ns_viz}}}position', x=str(n['x']), y=str(n['y']), z="0.0")

    # Add Edges
    edge_elements = ET.SubElement(graph, 'edges')
    for i, e in enumerate(edges):
        ET.SubElement(edge_elements, 'edge', id=str(i), source=e['source'], target=e['target'])

    # 3. Write to File (Pretty Print)
    xml_str = minidom.parseString(ET.tostring(gexf)).toprettyxml(indent="  ")
    with open(OUTPUT_GEXF, 'w') as f:
        f.write(xml_str)

    print(f"Success! Generated {OUTPUT_GEXF}")
    print(f"Stats: {len(nodes)} nodes, {len(edges)} edges.")

if __name__ == "__main__":
    convert_lgf_to_gexf(INPUT_FILE)