#!/usr/bin/env python

"""Example on how to load data from a Comma Separated Value file"""

#useful package for operations on numerical arrays (similar to Matlab)
import numpy as np

def main():
    #open a file for reading, and automatically close it when done
    with open('test.csv','r') as file_id:
        #use one of NumPy functions to load the data into an array
        data=np.genfromtxt(file_id,delimiter=',')
        #iterate over the rows
        for row in data:
            #print individual elements
            print 'First:',row[0],'Second:',row[1],'Third:',row[2]

if __name__ == '__main__':
    main()
