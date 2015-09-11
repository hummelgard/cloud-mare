

import crc16
print(hex(crc16.crc16xmodem(b'1234567890')))

def checkCRC(message):
    #CRC-16-CITT poly, the CRC sheme used by ymodem protocol
    poly = 0x11021
    #16bit operation register, initialized to zeros
    reg = 0xFFFF
    #pad the end of the message with the size of the poly
    message += '\x00\x00' 
    #for each bit in the message
    for byte in message:
        mask = 0x80
        while(mask > 0):
            #left shift by one
            reg<<=1
            #input the next bit from the message into the right hand side of the reg

            if ord(byte) & mask:   
                reg += 1
            mask>>=1
            #if a one popped out the left of the reg, xor reg w/poly
            if reg > 0xffff:            
                #eliminate any one that popped out the left
                reg &= 0xffff           
                #xor with the poly, this is the remainder
                reg ^= poly
    return reg

print(hex(checkCRC("1234567890")))

POLYNOMIAL = 0x1021
PRESET = 0

def _initial(c):
    crc = 0
    c = c << 8
    for j in range(8):
        if (crc ^ c) & 0x8000:
            crc = (crc << 1) ^ POLYNOMIAL
        else:
            crc = crc << 1
        c = c << 1
    return crc

_tab = [ _initial(i) for i in range(256) ]

def _update_crc(crc, c):
    cc = 0xff & c

    tmp = (crc >> 8) ^ cc
    crc = (crc << 8) ^ _tab[tmp & 0xff]
    crc = crc & 0xffff
    print (crc)

    return crc

def crc(str):
    crc = PRESET
    for c in str:
        crc = _update_crc(crc, ord(c))
    return crc

def crcb(*i):
    crc = PRESET
    for c in i:
        crc = _update_crc(crc, c)
    return crc


print(hex(crc('1234567890')))
