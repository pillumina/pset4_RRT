from math import sin, cos, pi
import random
import matplotlib.pyplot as plt

def next_move(x, y, theta, spd, t):
    # nxt_theta = (theta + 360) % 360
    nxt_theta = theta + (spd[0] - spd[1]) * t
    nxt_rad = nxt_theta * pi / 180
    if nxt_theta <= 45:
        nxt_x = x + t * sin(nxt_rad)
        nxt_y = y + t * cos(nxt_rad)
    elif nxt_theta > 45 and nxt_theta <= 90:
        nxt_x = x + t * cos(pi/2 - nxt_rad)
        nxt_y = y + t * sin(pi/2 - nxt_rad)
    elif nxt_theta > 90 and nxt_theta <= 135:
        nxt_x = x + t * cos(nxt_rad - pi/2)
        nxt_y = y + t * sin(nxt_rad - pi/2)
    elif nxt_theta > 135 and nxt_theta <= 180:
        nxt_x = x + t * sin(pi - nxt_rad)
        nxt_y = y + t * cos(pi - nxt_rad)
    elif nxt_theta > 180 and nxt_theta <= 225:
        nxt_x = x + t * sin(nxt_rad - pi)
        nxt_y = y + t * cos(nxt_rad - pi)
    elif nxt_theta > 255 and nxt_theta <= 270:
        nxt_x = x + t * cos(1.5 * pi - nxt_rad)
        nxt_y = y + t * sin(1.5 * pi - nxt_rad)
    elif nxt_theta > 270 and nxt_theta <= 315:
        nxt_x = x + t * cos(nxt_rad - 1.5 * pi)
        nxt_y = y + t * sin(nxt_rad - 1.5 * pi)
    elif nxt_theta > 315 and nxt_theta <= 360:
        nxt_x = x + t * sin(2 * pi - nxt_rad)
        nxt_y = y + t * cos(2 * pi - nxt_rad)
    return nxt_x, nxt_y, nxt_theta

def robot_traj(init_loc, goal_loc):
    init_x, init_y, init_theta = init_loc
    goal_x, goal_y, goal_theta = goal_loc
    cur_x, cur_y = init_x, init_y
    cur_theta = init_theta
    traj = []
    while (abs(cur_x - goal_x) > 1) and (abs(cur_y - goal_y) > 1):
        traj.append((cur_x, cur_y))
        # cur_theta = cur_theta + random.randint(-45, 45)
        cur_x, cur_y, cur_theta = next_move(cur_x, cur_y, cur_theta, [2, 1], 10)
    plt.scatter(init_x, init_y, c='b', marker='o', s=500)
    plt.scatter(goal_x, goal_y, c='r', marker='*', s=500)
    traj_x = [traj[i][0] for i in range(len(traj))]
    traj_y = [traj[i][1] for i in range(len(traj))]
    traj_x.append(goal_x)
    traj_y.append(goal_y)
    plt.plot(traj_x, traj_y, 'ko-')
    print(traj[-2])
    plt.show()

if __name__ == '__main__':
    init_loc = (50, 50, 0)
    goal_loc = (170, 183, 0)
    robot_traj(init_loc, goal_loc)
