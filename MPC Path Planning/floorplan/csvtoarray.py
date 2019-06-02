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

def arrayToObstacles(array):
	obstacles = []
	for i in range(len(array[0])):
		for j in range(len(array[1])):
			if int(array[i][j]) == 1:
				obstacles.append([j, i])

	dim = max(len(array[0]), len(array[1]))

	return obstacles, dim