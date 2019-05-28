import numpy as np
import itertools
from functions import *
import matplotlib.pyplot as plt
from matplotlib import animation

# Inputs
H_p = 2 # number of steps in prediction horizon
H_c = 1 # number of steps in control horizon
dt = 1 # seconds in one time step

# positional coordinates
x = [0] 
y = [0]
v = [0]
phi = [0]
goal = [-5, -5]

k = 0 # initialize time steps
goalReached=False

obstacles = [[0, 1], [2, 1]]
min_buffer = 0.1

# The method that prints all  
# possible strings of length k. 
# It is mainly a wrapper over  
# recursive function printAllKLengthRec()
# https://www.geeksforgeeks.org/print-all-combinations-of-given-length/ 
def printAllKLength(set, k):

    initialScore = 1e100
    optimal = [""]*k
    optimal.append(initialScore) 
  
    n = len(set)  
    prefix = []
    return printAllKLengthRec(set, prefix, n, k, optimal)
  
# The main recursive method 
# to print all possible  
# strings of length k 
def printAllKLengthRec(set, prefix, n, k, optimal): 
    score = optimal[-1]
    best = [1e100]

    # Base case: k is 0, 
    # print prefix 
    if (k == 0):
        newScore = calc_score(prefix, dt, H_c, x, y, k, goal, obstacles, min_buffer)
        if newScore < score:
            optimal = prefix.copy()
            optimal.append(newScore)
            return optimal
    
    # One by one add all characters  
    # from set and recursively  
    # call for k equals to k-1 

    for i in range(n): 

        # Next character of input added 
        newPrefix = prefix.copy()
        newPrefix.append(set[i])
          
        # k is decreased, because  
        # we have added a new character 
        new = printAllKLengthRec(set, newPrefix, n, k - 1, optimal)
        if new[-1] < best[-1]:
            best = new

    return best

combos = [[-5.0, -3.0], [-5.0, 3.0], [5.0, -3.0], [5.0, 3.0]]
k = 2

#print(printAllKLength(combos, k)) 

# # First set up the figure, the axis, and the plot element we want to animate
# fig = plt.figure()
# ax = plt.axes(xlim=(0, 2), ylim=(-2, 2))
# line, = ax.plot([], [], lw=2)

# # initialization function: plot the background of each frame
# def init():
#     line.set_data([], [])
#     return line,

# # animation function.  This is called sequentially



# def animate(i):
#     #x = np.linspace(0, 2, 1000)
#     #y = np.sin(2 * np.pi * (x - 0.01 * i))
#     x = 2
#     y = x**2
#     line.set_data(x, y)
#     return line,

# call the animator.  blit=True means only re-draw the parts that have changed.
#anim = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=20, blit=True)

#plt.show()


# x = np.arange(-10,10)
# y = x**2

# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.plot(x,y)

# coords = []

# def onclick(event):
#     global ix, iy
#     ix, iy = event.xdata, event.ydata
#     print ('x = %d, y = %d'%(ix, iy))

#     global coords
#     coords.append((ix, iy))

#     if len(coords) == 2:
#         fig.canvas.mpl_disconnect(cid)

#     return coords
# cid = fig.canvas.mpl_connect('button_press_event', onclick)




plt.figure("Jump Trajectory Lite")
plt.title("Drag the dots to the locations specified below:")
plt.xlabel("Red --> Jump Take-off, Blue --> Start of Jump, Green --> Drop Point")

### Prompt user to place points 1, 2, and h_s
x1 = 0
y1 = 0
x2 = 0
y2 = 0
drop_height = 0

class setLocations:
    def __init__(self, rect, color):
        self.rect = rect
        self.press = None
        self.color = color

    def connect(self):
        'connect to all dots that are being clicked on'
        self.cidpress = self.rect.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.cidrelease = self.rect.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.cidmotion = self.rect.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)

    def on_press(self, event):
        'when button is pressed, check if mouse is over a dot and store data'
        if event.inaxes != self.rect.axes: return

        contains, attrd = self.rect.contains(event)

        print(contains)
        
        if not contains: 
            print("test")
            circle1 = plt.Circle((0.5,0.5), 0.2 , color='red')
            plt.gcf().gca().add_artist(circle1)
            dr = setLocations(circle1, 'red')
            dr.connect()
            drs.append(dr)
            return

        x0, y0 = self.rect.center
        self.press = x0, y0, event.xdata, event.ydata

    def on_motion(self, event):
        'if mouse is clicked and held over dot, move dot along with mouse'
        if self.press is None: return
        x0, y0, xpress, ypress = self.press
        dx = event.xdata - xpress # distance moved in x direction
        dy = event.ydata - ypress # distance moved in y direction
        self.rect.center = x0+dx, y0+dy

    def on_release(self, event):
        'on release, reset press data and keep object in most recent location'
        self.press = None
        self.rect.figure.canvas.draw()
        
    def disconnect(self):
        'disconnect all stored connection ids'
        self.rect.figure.canvas.mpl_disconnect(self.cidpress)
        self.rect.figure.canvas.mpl_disconnect(self.cidrelease)
        self.rect.figure.canvas.mpl_disconnect(self.cidmotion)

img_height = 300
img_width = 400
circle1 = plt.Circle((img_width*3/12,img_height/6), img_width/150,color='red')

plt.gcf().gca().add_artist(circle1)

drs = []
dr = setLocations(circle1, 'blue')
dr.connect()
drs.append(dr)

plt.show()


# # The main function that recursively prints all repeated 
# # permutations of the given string. It uses data[] to store 
# # all permutations one by one 
# # https://www.geeksforgeeks.org/print-all-permutations-with-repetition-of-characters/
# def allLexicographicRecur(string, data, last, index, optimal): 

#     length = len(string) 
#     hiScore = float(optimal[-1])
  
#     # One by one fix all characters at the given index and 
#     # recur for the subsequent indexes 
#     for i in range(length): 

#         # Fix the ith character at index and if this is not 
#         # the last index then recursively call for higher 
#         # indexes 

#         data[index] = string[i] 

#         # If this is the last index then print the string 
#         # stored in data[] 
#         if index==last:
#             score = calc_score(data, dt, H_c, x, y, k, goal, obstacles, min_buffer)
#             if score < hiScore:
#                 optimal = data.copy()
#                 optimal.append(score)
#                 return optimal
#         else:
#             new = allLexicographicRecur(string, data, last, index+1, optimal)
#             if new[-1] < optimal[-1]:
#                 optimal = new

#     return optimal
  
# # This function sorts input string, allocate memory for data 
# # (needed for allLexicographicRecur()) and calls 
# # allLexicographicRecur() for printing all permutations 
# def allLexicographic(string): 
#     length = len(string) 
  
#     # Create a temp array that will be used by 
#     # allLexicographicRecur() 
#     data = [""] * (length) 
#     optimal = [""] * (length)
#     optimal.append(100000000000000.0) 
  
#     # Sort the input string so that we get all output strings in 
#     # lexicographically sorted order     
#     #string = sorted(string) 
  
#     # Now print all 
#     return allLexicographicRecur(string, data, length-1, 0, optimal) 
  
# # Find control sequence with lowest score
# optimal_sequence = allLexicographic(combos)[:-1]