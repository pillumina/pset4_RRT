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



# border of the environment
grid_width = 500
grid_length = 500

border = []
for i in range(grid_length):
    border.append((0, i))
    border.append((grid_width,i))
for i in range(grid_width):
    border.append((i, 0))
    border.append((i,grid_length))

# Obstacle
obs = []
inside_obstacle = []
obs_length = 20
obs_width = 40
obs_x = 200
obs_y = 300

for i in range(obs_x,obs_x+obs_width):
    obs.append((i, obs_y))
    obs.append((i, obs_y+obs_length))
for i in range(obs_y, obs_y+obs_length):
    obs.append((obs_x, i))
    obs.append((obs_x+obs_width, i))
for i in range(obs_x,obs_x+obs_width):
    for j in range(obs_y, obs_y+obs_length):
        inside_obstacle.append((i,j))
# plt.figure(figsize=(15, 10))

plt.subplot(211)
for border_point in border:
    plt.scatter(border_point[0], border_point[1], c='gray', marker = '.')
for obstacle_point in obs:
    plt.scatter(obstacle_point[0], obstacle_point[1], c='gray', marker = '.')
# for in_obs in inside_obstacle:
#     plt.scatter(in_obs[0], in_obs[1], c='gray', marker='.')
# plt.show()

free_move_space = []
for i in range(grid_width):
    for j in range(grid_length):
        if (i,j) in inside_obstacle:
            pass
        free_move_space.append((i,j))

c_space_border_overlap = []
for point in free_move_space:
    for obstacle_point in obs:
        if get_distance(point, obstacle_point) == 40:
            c_space_border_overlap.append(point)



max_x = max(c_space_border_overlap,key=lambda item:item[0])[0]
min_x = min(c_space_border_overlap,key=lambda item:item[0])[0]
max_y = max(c_space_border_overlap,key=lambda item:item[1])[1]
min_y = min(c_space_border_overlap,key=lambda item:item[1])[1]


c_space_border = []
for x in range(min_x, max_x):
    for y in range(min_y, max_y):
        c_space_border.append((x,y))


# plt.figure(figsize=(15, 10))
plt.subplot(212)
for border_point in border:
    plt.scatter(border_point[0], border_point[1], c='gray', marker = '.')
for obstacle_point in c_space_border:
    plt.scatter(obstacle_point[0], obstacle_point[1], c='gray', marker = '.')
plt.show()