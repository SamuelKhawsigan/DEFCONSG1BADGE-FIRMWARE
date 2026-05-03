import re
with open("SCHEMATICS/control.kicad_sch") as f:
    c = f.read()

# find ESP32 instance
inst_match = re.search(r'\(symbol\s+\(lib_id\s+"espressif:ESP32-C6-WROOM-1"\).*?\(at\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\)', c, re.DOTALL)
if inst_match:
    x0, y0, r0 = map(float, inst_match.groups())
    print(f"ESP32 at {x0}, {y0}, rot {r0}")
else:
    print("ESP32 not found")

# Let's extract global labels
labels = re.findall(r'\(global_label\s+"([^"]+)"\s+\w+\s*\(at\s+([-\d.]+)\s+([-\d.]+)', c)
for name, x, y in labels:
    if "SPI" in name or "TFT" in name:
        print(f"Label {name} at {x}, {y}")

