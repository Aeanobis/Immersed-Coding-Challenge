
with open('erase_file_4K.bin', 'wb') as f:
    data = bytearray(b'\xff'*4096)
    f.write(data)
    
with open('erase_file_16K.bin', 'wb') as f:
    data = bytearray(b'\xff'*(16 * 1024))
    f.write(data)
    
with open('erase_file_4096K.bin', 'wb') as f:
    data = bytearray(b'\xff'*(4096 * 1024))
    f.write(data)