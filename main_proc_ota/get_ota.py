import logging

logging.basicConfig()

log = logging.getLogger()
log.setLevel(logging.DEBUG)

from pymodbus.file_message import FileRecord, ReadFileRecordRequest

from pymodbus.client import ModbusSerialClient


client = ModbusSerialClient(method="rtu", port="COM60", baudrate=115200, timeout=2)

records = []
# Create records to be read and append to records
record1 = FileRecord(reference_type=0x06, file_number=0x01, record_number=0x00, record_length=0x20)
records.append(record1)
record2 = FileRecord(reference_type=0x06, file_number=0x01, record_number=0x01, record_length=0x20)
records.append(record2)

request = ReadFileRecordRequest(records=records, slave=1)
response = client.execute(request)
if not response.isError():
    # List of Records could be accessed with response.records
    print(response.records)
else:
    # Handle Error
    print(response)