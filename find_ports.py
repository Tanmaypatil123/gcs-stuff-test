#!/usr/bin/env python3
"""
Simple script to find available COM ports
"""

import serial.tools.list_ports

print("\n" + "="*60)
print("   Available COM Ports Detector")
print("="*60 + "\n")

ports = list(serial.tools.list_ports.comports())

if not ports:
    print("✗ No COM ports found!")
    print("\nMake sure:")
    print("  1. Flight controller is connected via USB")
    print("  2. Drivers are installed")
    print("  3. Cable is working")
else:
    print(f"Found {len(ports)} COM port(s):\n")
    for i, port in enumerate(ports, 1):
        print(f"{i}. Port: {port.device}")
        print(f"   Description: {port.description}")
        print(f"   Hardware ID: {port.hwid}")
        print()

print("="*60)
print("\nUse one of these ports in your connection string!")
print("Example: CONNECTION_STRING = 'COM4'")
print("="*60 + "\n")
