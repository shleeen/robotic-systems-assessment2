import serial
import time
import csv
import matplotlib.pyplot as plt

def main():
    # First read the sensor_data
    readSerialMonitor()

    # Then try to plot things
    # plotSpeedVsDist()

    print(" :) \n")


def readSerialMonitor():
    arduino = serial.Serial(port='COM6', baudrate=9600, timeout=.1)
    # time.sleep(2)

    with open('sensor_data.csv', mode='w') as sensor_data:
        # open csv file in write mode
        sensor_data = csv.writer(sensor_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        #  write header for each column of csv file
        sensor_data.writerow(['Speed', 'Distance', 'Lux'])

        # reads serial monitor for 180 seconds
        for i in range(180):
            b = arduino.readline()                  # read a byte string until line terminator
            string_n = b.decode()                   # decode byte string into Unicode
            string = string_n.rstrip()              # remove \n and \r
            split_str = string.split(',')           # split at the commas to get each value
            # flt = float(string)                   # convert string to float

            sensor_data.writerow(split_str)         # write to the csv file
            print(split_str)
            # time.sleep(0.1)                       # wait (sleep) 0.1 seconds
    arduino.close()


def plotSpeedVsDist():
    speed=[]
    dist=[]
    lux=[]

    csvfile = open('sensor_data.csv', 'r')
    plots= csv.reader(csvfile, delimiter=',')
    for row in plots:
        # speed.append(float(row[0]))
        # dist.append(float(row[1]))
        lux.append(float(row[2]))

    f1 = plt.figure()
    plt.plot(lux)
    plt.legend()
    plt.xlabel('Lux')
    plt.ylabel('???')
    plt.show()


if __name__ == "__main__":
    main()
