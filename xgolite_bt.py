import asyncio
from bleak import BleakClient

ADDRESS = "88:57:21:92:73:02"
CHAR_WRITE = "0000fff2-0000-1000-8000-00805f9b34fb"

def create_packet(order_byte, value_bytes):
    """
    Create a proper XGO protocol packet
    Protocol: [0x55, 0x00, len+0x08, mode, order, value..., checksum, 0x00, 0xAA]
    """
    mode = 0x01
    length = len(value_bytes) + 0x08
    
    # Calculate checksum
    value_sum = sum(value_bytes)
    sum_data = (length + mode + order_byte + value_sum) % 256
    sum_data = 255 - sum_data
    
    # Build packet
    packet = [0x55, 0x00, length, mode, order_byte]
    packet.extend(value_bytes)
    packet.extend([sum_data, 0x00, 0xAA])
    
    return bytes(packet)

async def main():
    async with BleakClient(ADDRESS, timeout=30.0) as client:
        print("Connected:", client.is_connected)
        
        # Example: Move forward at speed 10
        # VX command (0x30), value needs to be converted
        # For xgolite: speed range is -25 to 25
        # Formula: 128 + (128 * speed / limit)
        speed = 10  # Forward speed
        speed_value = int(128 + 128 * speed / 25)  # Convert to 0-255 range
        
        packet = create_packet(0x30, [speed_value])
        print(f"Sending packet: {packet.hex()}")
        
        await client.write_gatt_char(CHAR_WRITE, packet, response=False)
        print("Forward command sent!")
        
        # Wait 2 seconds
        await asyncio.sleep(2)
        
        # Stop command (speed = 0)
        stop_packet = create_packet(0x30, [128])  # 128 = neutral/stop
        await client.write_gatt_char(CHAR_WRITE, stop_packet, response=False)
        print("Stop command sent!")

asyncio.run(main())
