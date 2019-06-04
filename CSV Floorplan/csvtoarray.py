import csv

def csvtoarray(csvfile = 'floorplan.csv'):
    datafile = open(csvfile, 'r')
    datareader = csv.reader(datafile, delimiter=',')
    data = []
    for row in datareader:
        data.append(row)
    # print(data)
    # print(data[0][1])
    # data[y][x], origin at top left corner
    return(data)