import re

with open("SCHEMATICS/control.kicad_sch", "r") as f:
    content = f.read()

# Simple parser to find pins in the schematic and what nets they connect to.
# Kicad 6/7 schematic files are sexpressions.
# We are looking for lines like: (pin "X" (uuid ...)) followed by some connection? 
# Actually, the easiest way is to find the net labels and see which symbol pins they attach to, 
# but they are just placed at the same coordinates.

# Let's search for (symbol (lib_id "espressif:ESP32-C6-WROOM-1") ... )
import sys
start = content.find("ESP32-C6")
if start == -1:
    print("ESP32-C6 not found")
else:
    # Just print the symbol block to see its structure
    print("Found ESP32-C6")

