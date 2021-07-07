from serial_data_communicator.serial_communicator import serial_com

if __name__ == '__main__':
    print("hello world")
    serial_com.send_data("shajse", add_ending=True)
