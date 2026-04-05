import numpy as np
import matplotlib.pyplot as plt
import cmasher as cmr
import tqdm as tqd

Y_POINTS = 15
ASPECT_RATIO = 10
VISCOSITY = 0.01
PLOT_EVERY = 5000

POISSON_ITERS = 50

def main():
    cell_length = 1.0/ (Y_POINTS - 1)
    X_POINTS = (Y_POINTS-1)* ASPECT_RATIO +1

    x_range = np.linspace(0.0, 1.0 * ASPECT_RATIO, X_POINTS)
    y_range = np.linspace(0.0,1.0, Y_POINTS)

    coords_x, coords_y = np.meshgrid(x_range,y_range)

    #initial condition
    velocity_x = np.ones((Y_POINTS+1,X_POINTS))
    velocity_y = np.zeros((Y_POINTS,X_POINTS+1))

if __name__ == "__main__":
    main()
