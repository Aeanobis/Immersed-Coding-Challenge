import logging
import time
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

logging.basicConfig()

log = logging.getLogger()
log.setLevel(logging.INFO)

# Create a Modbus client instance
client = ModbusSerialClient(method="rtu", port="COM4", baudrate=115200, timeout=2)

# Connect to the Modbus server
connection = client.connect()
if not connection:
    log.error("Unable to connect to the Modbus server")
    exit(1)

try:
    # Start loop
    for value in range(10):
        try:
            
            print(f"Writing {value} to register 0")
            # Write the current value to register 0
            result = client.write_register(address=0, value=value, slave=1)
            if result.isError():
                log.error(f"Failed to write value {value} to register 0: {result}")
                continue

            time.sleep(0.25)  # Add a short delay to avoid overwhelming the bus
            # Write False to coil at address 56
            result = client.write_coil(address=56, value=False, slave=1)
            if result.isError():
                log.error(f"Failed to write to coil 56: {result}")
                continue

            time.sleep(0.25)  # Add a short delay to avoid overwhelming the bus
            # Write True to coil at address 56
            result = client.write_coil(address=56, value=True, slave=1)
            if result.isError():
                log.error(f"Failed to write to coil 56: {result}")
                continue

            time.sleep(10)  # Add a short delay to avoid overwhelming the bus

        except ModbusIOException as e:
            log.error(f"Modbus IO Exception: {e}")
        except Exception as e:
            log.error(f"An unexpected error occurred: {e}")

finally:
    # Close the client connection
    client.close()

log.info("DONE")
