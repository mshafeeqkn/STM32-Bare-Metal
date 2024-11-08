import usb.core
import usb.util

from inputimeout import inputimeout, TimeoutOccurred


# Find the USB device (replace with your device's Vendor ID and Product ID)
VENDOR_ID = 0x0483  # Replace with your device's Vendor ID
PRODUCT_ID = 0x575a  # Replace with your device's Product ID

# Find the device
dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

if dev is None:
    raise ValueError("Device not found")

max_packet_size = dev[0][(0, 0)][0].wMaxPacketSize

# Ensure the device is set to the first configuration
if dev.is_kernel_driver_active(0):
    dev.detach_kernel_driver(0)

if dev.get_active_configuration() is None:
    dev.set_configuration()

# Define the endpoint (replace with the correct endpoint address)
RD_ENDPOINT = 0x81  # Typically 0x81 for IN endpoints
WR_ENDPOINT = 0x01  # Typically 0x81 for IN endpoints
BYTES_TO_READ = max_packet_size  # Number of bytes to read
BYTES_TO_WRITE = max_packet_size  # Number of bytes to read
TIMEOUT = 1000  # Timeout in milliseconds

while True:
    # Wait for user input
    try:
        user_input = inputimeout(prompt="Enter something: ", timeout=0.5)
        dev.write(WR_ENDPOINT, user_input[:2], TIMEOUT)
        print(f"\rYou entered: {user_input[:2]}", end="")
    except TimeoutOccurred:
        pass

    try:
        # Read data from the endpoint
        data = dev.read(RD_ENDPOINT, BYTES_TO_READ, TIMEOUT)
        print(f"Data read: {data}")
    except usb.core.USBError as e:
        print(f"USB Error: {e}")

