
packet = bytearray()
packet.append(0xfa)
packet.append(0x01)
packet.append(0xfd)
packet.append(0x80) #direction
packet.append(0x01)
packet.append(0x01) #acceleration
#packet.append(clockwise_direction)
packet.append(0x00)
packet.append(0x00)
packet.append(0x0c)
packet.append(0x80)
crc = (sum(packet) & 0xFF)
print(hex(crc))

# fa 01 fd 00 01 01 00 00 0c 80 86  - anti clockwise
# fa 01 fd 80 01 01 00 00 0c 80 06  - clockwise

print(int(3200).to_bytes(4,byteorder='big'))


packet = bytearray()
packet.append(0xfa)
packet.append(0x01)
packet.append(0x33)
crc = (sum(packet) & 0xFF)
print(hex(crc))