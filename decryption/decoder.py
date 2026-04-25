try:
    filenum = int(input('Input log file number to decode: '))
except:
    print('Please enter an integer.')

filename = 'log' + str(filenum) + '.bin'

logfile = open(filename, 'rb')
b = logfile.read()

#work out number of packets:

full_packets = (len(b) // 256)
extra_bits = len(b) % 256

csv = open('output' + str(filenum) + '.csv', 'w')
csv.write('time,error_x,Px,Ix,Dx,error_y,Py,Iy,Dy,Arm,Mode\n')
offset = 0

def iterate(offset):
    values = [0,0,0,0,0,0,0,0,0,0,0]
    values[0] = (b[3 + offset] + b[2 + offset]*(2**8) + b[1 + offset]*(2**16) + b[0 + offset]*(2**24)) #time
    values[1] = (int.from_bytes(bytes([b[4 + offset],b[5 + offset]]),'big', signed=True)/100)          #error_x
    values[2] = (int.from_bytes(bytes([b[6 + offset],b[7 + offset]]),'big', signed=True)/100)          #Px
    values[3] = (int.from_bytes(bytes([b[8 + offset],b[9 + offset]]),'big', signed=True)/100)          #Ix
    values[4] = (int.from_bytes(bytes([b[10 + offset],b[11 + offset]]),'big', signed=True)/100)        #Dx
    values[5] = (int.from_bytes(bytes([b[12 + offset],b[13 + offset]]),'big', signed=True)/100)        #error_y
    values[6] = (int.from_bytes(bytes([b[14 + offset],b[15 + offset]]),'big', signed=True)/100)        #Py
    values[7] = (int.from_bytes(bytes([b[16 + offset],b[17 + offset]]),'big', signed=True)/100)        #Iy
    values[8] = (int.from_bytes(bytes([b[18 + offset],b[19 + offset]]),'big', signed=True)/100)        #Dy
    values[9] = (b[20 + offset])                                                                       #Arm
    values[10] = (b[21 + offset])                                                                      #Mode

    csv.write(str(values)[1:-1] + '\n')

while full_packets > 0:
    for i in range(0, 11):
        iterate(offset)
        offset = offset + 22
    offset = offset + 14
    full_packets = full_packets - 1
