import string
from math import cos, sin
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib import patches

def plot_koule_test(verts):
    i = 1
    num_koules = 0;
    plt.xlabel("X")
    plt.ylabel("Y")
    fig = plt.figure()
    for v in verts:
        num_koules = (len(v) - 5) / 4
        fig.clear()
        ax = fig.add_subplot(111)
        ax.set_xlim(-0.0015, 1.0015)
        ax.set_ylim(-0.0015, 1.0015)
        ship = plt.Circle((v[0], v[1]), 0.03, edgecolor = "blue", fill = False)
        ax.add_patch(ship)
        plt.plot([v[0]+cos(v[2])*0.03], [v[1]+sin(v[2])*0.03], 'ro')  # direction of ship
        plt.plot([v[0], v[0]+v[3]], [v[1], v[1]+v[4]])  # velocity of ship
        for j in range(num_koules):
            koule = plt.Circle((v[5+4*j], v[6+4*j]), 0.015, edgecolor = "red", fill = False)
            ax.add_patch(koule)
            # plt.plot([v[5], v[5]+v[7]], [v[6], v[6]+v[8]])  # velocity of koule
        plt.savefig(str(i) + ".png")
        i += 1
        
def plot_from_file(filepath):
    f = open(filepath, 'r')
    verts = []
    for line in f:
        if (line[0] in string.digits):
            row = string.split(line.rstrip(' \n'), " ")
            for i in range(len(row)):
                row[i] = float(row[i])
            verts.append(tuple(row))
    plot_koule_test(verts)    
