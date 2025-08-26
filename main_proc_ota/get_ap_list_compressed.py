import logging
import time
import zlib
import lz4.frame

logging.basicConfig()

log = logging.getLogger()
log.setLevel(logging.INFO)

from pymodbus.file_message import FileRecord, ReadFileRecordRequest

from pymodbus.client import ModbusSerialClient


client = ModbusSerialClient(method="rtu", port="COM9", baudrate=115200, timeout=2)

#Write to the WiFi config register a value of 4 to set the scan flag
client.write_registers(address=1700, values=[4], slave=1)

time.sleep(8)

log.info("Starting file read")
#Now the file is ready start reading it
# Start reading file records
record_address = 0  # Starting address
record_length = 64  # Each record is fixed at 128 (words)
file_number = 3     # Log file is fixed at 2

#record1 = FileRecord(reference_type=0x06, file_number=0x02, record_number=0x00, record_length=0x20)
#records.append(record1)
#record2 = FileRecord(reference_type=0x06, file_number=0x02, record_number=0x01, record_length=0x20)
#records.append(record2)
# File to save the data
output_file = 'wifi_ap_list_comp.txt'
output_file_decomp = 'wifi_ap_list.txt'
start_time = time.time()
try:
    with open(output_file, 'wb') as file:
        while True:
            log.info("Reading record address %s", str(record_address))
            records = []  # Reset the records list for each iteration
            record1 = FileRecord(reference_type=0x06, file_number=0x03, record_number=record_address, record_length=0x40)
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
                            # Concatenate in reversed order
                            file.write(record.record_data[i + 1].to_bytes(1, 'little'))
                            file.write(record.record_data[i].to_bytes(1, 'little'))
                    
                except AttributeError as e:
                    log.error("Error accessing record data: %s", e)
            # Process the data
            #log.info

            #log.info("Data: ", response.records)

            # Increment to read next segment
            record_address = record_address + 1

except Exception as e:
    log.error("Modbus Error: ", str(e))

finally:
    client.close()
    end_time = time.time()
    time_diff = end_time - start_time
    print(f"Execution time = {time_diff}")

#Now decompress the data
with open(output_file, 'rb') as file:
    compressed_data = file.read()
    decompressed_data = lz4.frame.decompress(compressed_data)
    with open (output_file_decomp, 'wb') as outfile:
        outfile.write(decompressed_data)