#!/usr/bin/env python

"""Example on how to write data to a Comma Separated Value file"""

def main():
    #open a file for writing, and automatically close it when done
    with open('test_write.csv','w') as file_id:
        #write two lines with some arbitarry values
        file_id.write('%.6f, %.6f, %.6f\n'%(0.1,0.5,0.6))
        file_id.write('%.6f, %.6f, %.6f\n'%(0.2,0.6,0.7))

if __name__ == '__main__':
    main()
