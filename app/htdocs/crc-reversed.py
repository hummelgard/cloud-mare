import crcmod
import crcmod.predefined

STX = b'\x02'
ETX = b'\x03'
data = b'123456789'
message = data#= STX + data + ETX

crc16 = crcmod.predefined.Crc('crc-16-mcrf4xx')
crc16.update(message)
crc = crc16.hexdigest()
print(crc16.digest())
print(crc)
