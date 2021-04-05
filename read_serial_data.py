import serial
import time
import csv

# empty list to store the data
right_wheel = []
left_wheel = []
demadn = []

# ser = serial.Serial();
arduino = serial.Serial(port='COM6', baudrate=9600, timeout=.1)
time.sleep(2)

# if (b=='s'), then start writing to file
with open('sensor_data.csv', mode='w') as sensor_data:
    # open csv file in write mode
    sensor_data = csv.writer(sensor_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    #  write header for each column of csv file
    sensor_data.writerow(['Right_wheel_speed', 'Demand', 'Left_wheel_speed'])

    # reads serial monitor for 50 seconds
    for i in range(50):
        b = arduino.readline()               # read a byte string until line terminator
        string_n = b.decode()                # decode byte string into Unicode
        string = string_n.rstrip()           # remove \n and \r
        split_str = string.split(',')        # split at the commas to get each value


        # flt = float(string)            # convert string to float
        # data.append(flt)               # add to the end of data list

        # if (b = 'k'), then stop writing to file
        sensor_data.writerow(split_str)
        print(split_str)


        time.sleep(0.1)                # wait (sleep) 0.1 seconds

arduino.close()



#  Print the list data[] to console
# for line in data:
#     print(line)

# --------- another way to do the things
# def write_read(x):
    # arduino.write(bytes(x, 'utf-8'))
    # time.sleep(0.05)
    # data = arduino.readline()
    # return data

# while True:
    # num = input("Enter a number: ") # Taking input from user
    # value = write_read(num)
    # print(value) # printing the value
