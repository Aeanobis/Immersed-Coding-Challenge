@echo off		
set /p certs=Flash certificates? 	1. Yes		2. No		
set /p config=Flash configuration? 	1. Yes		2. No		

if %certs%==1 (
	echo Writting certs
	python %~dp0\certs\nvs_partition_gen.py generate %~dp0\certs\certs_nvs.csv %~dp0\certs\certs_nvs.bin 16384
	esptool -p %1 -b 460800 --no-stub --before default_reset --after no_reset --chip esp32 write_flash --force --flash_mode dio --flash_size detect --flash_freq 40m 0x424000 %~dp0\nvs_seed\erase_file_4K.bin
	esptool -p %1 -b 460800 --no-stub --before default_reset --after no_reset --chip esp32 write_flash --force --flash_mode dio --flash_size detect --flash_freq 40m 0x420000 %~dp0\certs\certs_nvs.bin
)

if %config%==1 (
	echo Writting config
	esptool -p %1 -b 460800 --no-stub --before default_reset --after no_reset --chip esp32 write_flash --force --flash_mode dio --flash_size detect --flash_freq 40m --force 0xe000 %~dp0\nvs_seed\erase_file_16K.bin
	esptool -p %1 -b 460800 --no-stub --before default_reset --after no_reset --chip esp32 write_flash --force --flash_mode dio --flash_size detect --flash_freq 40m --force 0x12000 %~dp0\nvs_seed\erase_file_4K.bin
)

esptool -p %1 -b 460800 --no-stub --before default_reset --after no_reset --chip esp32 write_flash --force --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 build\bootloader\bootloader.bin 0xa000 build\partition_table\partition-table.bin 0x1c000 build\ota_data_initial.bin 0x20000 build\Amethyst.bin
