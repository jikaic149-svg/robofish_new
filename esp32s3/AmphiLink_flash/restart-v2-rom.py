"""Release the software forced-download flag and restart the expected XIAO."""
import sys
from esptool.targets.esp32s3 import ESP32S3ROM
esp=ESP32S3ROM(sys.argv[1],baud=115200)
try:
    esp.connect(mode='no_reset',attempts=1)
    if ':'.join(f'{b:02x}' for b in esp.read_mac())!='dc:b4:d9:3b:2b:00':
        raise RuntimeError('MAC mismatch before restart')
    esp.write_reg(esp.RTC_CNTL_OPTION1_REG,0,esp.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
    esp.watchdog_reset()
finally:esp._port.close()
