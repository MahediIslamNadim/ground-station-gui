import serial
import serial.tools.list_ports

print("=== Serial Port Test ===\n")

# List all ports
print("1. Scanning COM ports...")
ports = serial.tools.list_ports.comports()
print(f"   Found {len(ports)} port(s):")

for p in ports:
    print(f"   - {p.device}: {p.description}")

print("\n2. Trying to open each port...")
for i in range(1, 20):
    port_name = f"COM{i}"
    try:
        s = serial.Serial(port_name, 115200, timeout=1)
        print(f"   OK: {port_name} - Connected!")
        s.close()
    except serial.SerialException as e:
        pass
    except Exception as e:
        pass

print("\n3. Check Arduino IDE for your COM port number")
print("   Then type it in the Python GUI manual field")
print("\n=== Done ===")
