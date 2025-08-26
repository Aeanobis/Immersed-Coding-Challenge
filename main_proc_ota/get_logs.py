import logging
import time

logging.basicConfig()

log = logging.getLogger()
log.setLevel(logging.INFO)

from pymodbus.file_message import FileRecord, ReadFileRecordRequest

from pymodbus.client import ModbusSerialClient


client = ModbusSerialClient(method="rtu", port="COM60", baudrate=115200, timeout=2)

#First set up the timing for oldest and newest logs in db
client.write_coils(address=57, values=[True], slave=1)
time.sleep(0.5)     #Wait for times to be retrieved

#Read the time stamp of oldest and newest logs
_oldest = client.read_holding_registers(address=1500,count=2,slave=1)
oldest = _oldest.registers[0] << 16 | _oldest.registers[1]
_newest = client.read_holding_registers(address=1502,count=2,slave=1)
newest = _newest.registers[0] << 16 | _newest.registers[1]

log.info("oldest ts " + str(oldest) + " newest ts " + str(newest))

#Set the range for log retrival
log_start_ts = oldest
log_end_ts   = newest
client.write_registers(address=1504, values=[(log_start_ts >> 16) & 0xFFFF, (log_start_ts & 0xFFFF)], slave=1)
client.write_registers(address=1506, values=[(log_end_ts >> 16) & 0xFFFF, (log_end_ts & 0xFFFF)], slave=1)
client.write_coil(address=58, value=[True], slave=1)
time.sleep(1)
log.info("Starting file read")
#Now the file is ready start reading it
# Start reading file records
record_address = 0  # Starting address
record_length = 32  # Each record is fixed at 64 (words)
file_number = 2     # Log file is fixed at 2

#record1 = FileRecord(reference_type=0x06, file_number=0x02, record_number=0x00, record_length=0x20)
#records.append(record1)
#record2 = FileRecord(reference_type=0x06, file_number=0x02, record_number=0x01, record_length=0x20)
#records.append(record2)
# File to save the data
output_file = 'modbus_data.txt'
try:
    with open(output_file, 'w') as file:
        while True:
            log.info("Reading record address %s", str(record_address))
            records = []  # Reset the records list for each iteration
            record1 = FileRecord(reference_type=0x06, file_number=0x02, record_number=record_address, record_length=0x20)
            records.append(record1)            
            request = ReadFileRecordRequest(records=records, slave=1)
            response = client.execute(request)

            if response.isError():
                log.info("Reached end of file or error: %s", str(response))
                break
            else:
                log.info("Modbus response: %s", response)  # Log raw response
                try:
                    # Attempt to access the data directly
                    ascii_data = ''
                    for record in response.records:
                        for i in range(0, len(record.record_data), 2):
                            # Read two registers at a time
                            char1 = chr(record.record_data[i] & 0xFF) if i < len(record.record_data) else ''
                            char2 = chr(record.record_data[i+1] & 0xFF) if i+1 < len(record.record_data) else ''
                            # Concatenate in reversed order
                            ascii_data += char2 + char1
                            log.debug("Characters from registers: %s, %s", char1, char2)
                    file.write(ascii_data)
                except AttributeError as e:
                    log.error("Error accessing record data: %s", e)
            # Process the data
            #log.info("Data: ", response.records)

            # Increment to read next segment
            record_address = record_address + 1

except Exception as e:
    log.error("Modbus Error: ", str(e))

finally:
    client.close()