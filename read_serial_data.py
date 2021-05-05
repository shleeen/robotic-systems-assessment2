import serial
import pandas as pd
import time
import csv
import matplotlib.pyplot as plt

def main():
    # First read the sensor_data
    #readSerialMonitor()
    plotTimeVsDist()

    # Then try to plot things
    # plotSpeedVsDist()

    print(" :) \n")


def readSerialMonitor():
    arduino = serial.Serial(port='/dev/cu.usbmodem14101', baudrate=9600, timeout=.1)
    # time.sleep(2)

    with open('sensor_data_blue_lightOn.csv', mode='w') as sensor_data:
        # open csv file in write mode
        sensor_data = csv.writer(sensor_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        #  write header for each column of csv file
        sensor_data.writerow(['proximity_distance/mm', 'als_distance/mm', 'als_lux/lux','elapsed_time/s'])

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


def plotTimeVsDist():



    prox_meas=[]
    dist_als = []
    lux = []
    time = []



    csvfile = open('sensor_data_blue_lightOn.csv', 'r')
    df = pd.read_csv('sensor_data_blue_lightOn.csv')
    df.drop_duplicates(inplace=True)
    df.to_csv('sensor_data_blue_lightOn.csv', index=False)
    plots= csv.reader(csvfile, delimiter=',')
    for row in plots:
        prox_meas.append(float(row[0]))
        dist_als.append(float(row[1]))
        lux.append(float(row[2]))
        time.append(float(row[3]))

    f1 = plt.figure()
    plt.plot(time,prox_meas, label='Proximity Sensor')
    plt.plot(time,dist_als ,label= 'Ambient Light Sensor')
    plt.legend()
    plt.xlabel('Time/ms')
    plt.ylabel('Distance Calculated By Sensor/mm')
    plt.savefig('blue-lightOn-DistvsTime.pdf')
    plt.show()


if __name__ == "__main__":
    main()