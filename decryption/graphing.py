import csv

try:
    filenum = int(input('Input csv file number to graph: '))
except:
    print('Please enter an integer.')

file = open('output' + str(filenum) + '.csv', 'r')

data = []
with open('output' + str(filenum) + '.csv') as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
    for row in reader: # each row is a list
        data.append(row)

print(data[1][0])