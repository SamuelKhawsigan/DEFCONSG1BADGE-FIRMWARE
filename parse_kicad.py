import sys, re
from collections import defaultdict

def extract_connections(filename):
    with open(filename, 'r') as f:
        content = f.read()

    # Find all wires
    wires = re.findall(r'\(wire\s+\(pts\s+\(xy\s+([-\d.]+)\s+([-\d.]+)\)\s+\(xy\s+([-\d.]+)\s+([-\d.]+)\)', content)
    
    adj = defaultdict(set)
    for x1, y1, x2, y2 in wires:
        p1, p2 = (float(x1), float(y1)), (float(x2), float(y2))
        adj[p1].add(p2)
        adj[p2].add(p1)

    # Find labels
    labels = {}
    for m in re.finditer(r'\((global_label|label)\s+"([^"]+)"\s+\w*\s*\(at\s+([-\d.]+)\s+([-\d.]+)', content):
        name = m.group(2)
        p = (float(m.group(3)), float(m.group(4)))
        labels[p] = name

    # Find symbol pins. This is tricky because the pin in the schematic instance doesn't have a name, 
    # it just has a number. The name is in the symbol definition.
    # However, for ESP32, it's defined locally. Let's just look at the symbol definition.
    # Actually, ESP32 is defined in control.kicad_sch as (symbol (lib_id "espressif:ESP32-C6-WROOM-1") ... )
    # But when instantiated, the pins are at specific coordinates based on (at X Y angle) of the instance.
    return labels, adj, wires

print("Parsing...")
labels_in, adj_in, wires_in = extract_connections('SCHEMATICS/inoutput.kicad_sch')
labels_ctrl, adj_ctrl, wires_ctrl = extract_connections('SCHEMATICS/control.kicad_sch')

for p, name in labels_ctrl.items():
    if "SPI" in name or "TFT" in name or "LCD" in name:
        print(f"Ctrl Label: {name} at {p}")
