import re
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

# --- Configuration ---
IN_OUT = {
    './../instances/custom/tiny/tiny.lgf':'visualisation/tiny.gexf',
    './../instances/custom/tiny/tiny_c1.lgf':'visualisation/tiny_c1.gexf',
    './../instances/custom/tiny/tinyComplex.lgf':'visualisation/tinyComplex.gexf',
    './../instances/custom/tiny/tinyComplex_c1.lgf':'visualisation/tinyComplex_c1.gexf',
    './../instances/custom/tiny/tinyComplex_c2.lgf':'visualisation/tinyComplex_c2.gexf',
}

COORD_MULTIPLIER = 150
OFFSET_VAL = 10.0       # Shift for standard overlapping nodes
NULL_OFFSET_VAL = -10  # Larger shift for (-1, -1) null nodes
# ---------------------

def convert_lgf_to_gexf(input_path, output_path):
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    nodes = []
    edges = []
    current_section = None
    coord_pattern = re.compile(r'\(([^,]+),\s*([^)]+)\)')
    
    coord_counts = {}

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

        if line.lower() in ['label', 'label weight', 'source target weight']:
            continue

        parts = line.split()
        
        if current_section == 'nodes':
            match = coord_pattern.search(line)
            node_id = parts[0]
            
            raw_x, raw_y = (float(match.group(1)), float(match.group(2))) if match else (0.0, 0.0)
            
            coord_pair = (raw_x, raw_y)
            occurrence = coord_counts.get(coord_pair, 0)
            coord_counts[coord_pair] = occurrence + 1
            
            # Determine which offset to use
            # If coordinates are (-1, -1), use the special null offset
            current_offset = NULL_OFFSET_VAL if (raw_x == -1.0 and raw_y == -1.0) else OFFSET_VAL
            
            final_x = (raw_x * COORD_MULTIPLIER) + (occurrence * current_offset)
            final_y = (raw_y * COORD_MULTIPLIER) + (occurrence * current_offset)
            
            # negate final x to make it vertically flipped
            final_x = -final_x
            
            # Note: keeping your x/y swap for Gephi orientation
            nodes.append({'id': node_id, 'label': node_id, 'x': final_y, 'y': final_x})

        elif current_section == 'edges' and len(parts) >= 2:
            source = parts[0]
            target = parts[1]
            weight = parts[2] if len(parts) > 2 else "1.0"
            edges.append({'source': source, 'target': target, 'weight': weight})

    # Build GEXF Structure
    ns_viz = "http://www.gexf.net/1.2draft/viz"
    gexf = ET.Element('gexf', xmlns='http://www.gexf.net/1.2draft', version='1.2')
    gexf.set("xmlns:viz", ns_viz)
    graph = ET.SubElement(gexf, 'graph', defaultedgetype='directed')
    
    node_elements = ET.SubElement(graph, 'nodes')
    for n in nodes:
        node_xml = ET.SubElement(node_elements, 'node', id=n['id'], label=n['label'])
        ET.SubElement(node_xml, f'{{{ns_viz}}}position', x=str(n['x']), y=str(n['y']), z="0.0")

    edge_elements = ET.SubElement(graph, 'edges')
    for i, e in enumerate(edges):
        ET.SubElement(edge_elements, 'edge', 
                     id=str(i), 
                     source=e['source'], 
                     target=e['target'], 
                     weight=e['weight'],
                     label=e['weight'])

    xml_str = minidom.parseString(ET.tostring(gexf)).toprettyxml(indent="  ")
    with open(output_path, 'w') as f:
        f.write(xml_str)

    print(f"Processed: {input_path} -> {output_path}")

if __name__ == "__main__":
    for input_f, output_f in IN_OUT.items():
        convert_lgf_to_gexf(input_f, output_f)