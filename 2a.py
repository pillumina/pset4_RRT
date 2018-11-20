import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches




# function to find the distance between 2 points

def get_distance(point1, point2):
    if len(point1) != 2 or len(point2) != 2:
        return 'please provide x,y coordinates for each point'
    x_1 = point1[0]
    y_1 = point1[1]
    x_2 = point2[0]
    y_2 = point2[1]

    distance = np.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2)
    return distance


if __name__ == '__main__':

    # define the goal point
    goal = (250,250)

    # define all other points (randomly generated)
    p_list = []
    for i in range(100):
        point = (random.randint(0,500), random.randint(0,500))
        p_list.append(point)

    # find the closest point
    distance = []
    for point in p_list:
        distance.append(get_distance(goal, point))

    smallest_distance_index = distance.index(min(distance))
    closest_point = p_list[smallest_distance_index]
    print('the closest point is : ', closest_point)
    # plot out all of the points
    for point in p_list:
        plt.scatter(point[0], point[1], marker = '.', c='b')
    plt.scatter(closest_point[0], closest_point[1], marker='*', c='k')
    plt.scatter(goal[0], goal[1],  marker='D', c='r')
    plt.show()