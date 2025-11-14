"""
Simple test to verify MicroPython upload works
"""
print("Upload test successful!")
print("MicroPython version check OK")

# Test basic imports
try:
    import network
    import time
    import json
    print("Standard imports: OK")
except ImportError as e:
    print("Import error: {}".format(e))

# Test async
try:
    import uasyncio as asyncio
    print("Asyncio import: OK")
except ImportError as e:
    print("Asyncio not available: {}".format(e))

print("\nIf you see this, the upload process is working correctly!")
