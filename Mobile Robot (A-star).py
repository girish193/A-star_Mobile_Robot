import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import time


def workspace_check(x_new, y_new):
    """

        In this function the given value is checked to see if it lies in the workspace

        Parameters
        ----------
        x_new: X coordinate of given input
        y_new: Y coordinate of given input

        Returns
        -------
        True  : If the input lies inside workspace
        False : If the input lies outside workspace

    """

    if (x_new >= 0) and (y_new >= 0) and (x_new <= 400) and (y_new <= 300):
        return True
    else:
        return False


def obstacle_space(x_new, y_new, r_robot=10, c_robot=5):
    """

        In this function the given value is checked to see if it lies in the obstacle space

        Parameters
        ----------
        x_new: X coordinate of given input
        y_new: Y coordinate of given input
        r_robot: radius of mobile robot (default value = 10)
        c_robot: clearance of mobile robot (default value = 5)

        Returns
        -------
        True  : If the input lies outside obstacle space
        False : If the input lies inside obstacle space

    """

    distance = c_robot + r_robot

    flag1, flag2, flag3, flag4 = (True, True, True, True)

    # For Circle
    if (x_new - 90) ** 2 + (y_new - 70) ** 2 - (35 + distance) ** 2 < 0:
        flag1 = False

    # For Ellipse
    if ((x_new - 246) / (60 + distance)) ** 2 + ((y_new - 145) / (30 + distance)) ** 2 - 1 < 0:
        flag2 = False

    # For C-shape
    if (y_new > 230 - distance) and (y_new < 280 + distance) and (x_new > 200 - distance) and (x_new < 230 + distance):

        if (x_new <= 200) and (y_new >= 280) and ((x_new - 200) ** 2 + (y_new - 280) ** 2 >= distance ** 2):
            flag3 = True

        elif (x_new <= 200) and (y_new <= 230) and ((x_new - 200) ** 2 + (y_new - 230) ** 2 >= distance ** 2):
            flag3 = True

        elif (x_new >= 230) and (y_new >= 280) and ((x_new - 230) ** 2 + (y_new - 280) ** 2 >= distance ** 2):
            flag3 = True

        elif (x_new >= 230) and (y_new <= 230) and ((x_new - 230) ** 2 + (y_new - 230) ** 2 >= distance ** 2):
            flag3 = True

        elif (x_new >= 230) and (y_new <= 270) and (y_new >= 270 - distance):
            if (x_new - 230) ** 2 + (y_new - 270) ** 2 >= distance ** 2:
                flag3 = True
            else:
                flag3 = False

        elif (x_new >= 230) and (y_new >= 240) and (y_new <= 240 + distance):
            if (x_new - 230) ** 2 + (y_new - 240) ** 2 >= distance ** 2:
                flag3 = True
            else:
                flag3 = False

        elif (x_new >= 210) and (x_new <= 230) and (y_new <= 270) and (y_new >= 230):
            if (x_new >= 210 + distance) and (y_new <= 270 - distance) and (y_new >= 240 + distance):
                flag3 = True
            else:
                flag3 = False

        else:
            flag3 = False

    # For rotated rectangle
    x1, y1 = (48, 108)
    x2, y2 = (x1 - 20 * np.cos(11 * np.pi / 36), y1 + 20 * np.sin(11 * np.pi / 36))
    x4, y4 = (x1 + 150 * np.cos(7 * np.pi / 36), y1 + 150 * np.sin(7 * np.pi / 36))
    x3, y3 = (x4 - 20 * np.cos(11 * np.pi / 36), y4 + 20 * np.sin(11 * np.pi / 36))

    m_12 = (y2 - y1) / (x2 - x1)
    m_23 = (y3 - y2) / (x3 - x2)
    m_34 = (y4 - y3) / (x4 - x3)
    m_41 = (y1 - y4) / (x1 - x4)

    c_12 = y1 - m_12 * x1
    c_23 = y2 - m_23 * x2
    c_34 = y3 - m_34 * x3
    c_41 = y4 - m_41 * x4

    l_12 = lambda x, y: m_12 * x - y + c_12
    l_23 = lambda x, y: m_23 * x - y + c_23
    l_34 = lambda x, y: m_34 * x - y + c_34
    l_41 = lambda x, y: m_41 * x - y + c_41

    theta_12 = np.arctan(m_12)
    theta_23 = np.arctan(m_23)
    theta_34 = np.arctan(m_34)
    theta_41 = np.arctan(m_41)

    M_12, C_12 = (m_12, c_12 - distance * (np.cos(theta_12) + m_12 * np.sin(theta_12)))
    M_23, C_23 = (m_23, c_23 + distance * (np.cos(theta_23) + m_23 * np.sin(theta_23)))
    M_34, C_34 = (m_34, c_34 + distance * (np.cos(theta_34) + m_34 * np.sin(theta_34)))
    M_41, C_41 = (m_41, c_41 - distance * (np.cos(theta_41) + m_41 * np.sin(theta_41)))

    L_12 = lambda x, y: M_12 * x - y + C_12
    L_23 = lambda x, y: M_23 * x - y + C_23
    L_34 = lambda x, y: M_34 * x - y + C_34
    L_41 = lambda x, y: M_41 * x - y + C_41

    if (C_12 * L_12(x_new, y_new) < 0) and (C_23 * L_23(x_new, y_new) > 0) and (C_34 * L_34(x_new, y_new) > 0) and \
            (C_41 * L_41(x_new, y_new) < 0):

        if (c_12 * l_12(x_new, y_new) >= 0) and (c_41 * l_41(x_new, y_new) >= 0) and \
                ((x_new - x1) ** 2 + (y_new - y1) ** 2 >= distance ** 2):
            flag4 = True

        elif (c_23 * l_23(x_new, y_new) <= 0) and (c_12 * l_12(x_new, y_new) >= 0) and \
                ((x_new - x2) ** 2 + (y_new - y2) ** 2 >= distance ** 2):
            flag4 = True

        elif (c_34 * l_34(x_new, y_new) <= 0) and (c_23 * l_23(x_new, y_new) <= 0) and \
                ((x_new - x3) ** 2 + (y_new - y3) ** 2 >= distance ** 2):
            flag4 = True

        elif (c_41 * l_41(x_new, y_new) >= 0) and (c_34 * l_34(x_new, y_new) <= 0) and \
                ((x_new - x4) ** 2 + (y_new - y4) ** 2 >= distance ** 2):
            flag4 = True

        else:
            flag4 = False

    flag = flag1 and flag2 and flag3 and flag4

    return flag


def find_index(node_state):
    x_pt, y_pt, beta = node_state  # beta belongs to [0 to 2*np.pi)

    x_calc = x_pt - int(x_pt)
    y_calc = y_pt - int(y_pt)

    # ----------------------- For x_pt -----------------------

    if 0.5 * threshold_distance <= x_calc < 1.5 * threshold_distance:
        x_index = 2 * int(int(x_pt) + 0.5) + 1

    else:
        x_index = 2 * int(np.round(x_pt))

    # ----------------------- For y_pt ----------------

    if 0.5 * threshold_distance <= y_calc < 1.5 * threshold_distance:
        y_index = 2 * int(int(y_pt) + 0.5) + 1

    else:
        y_index = 2 * int(np.round(y_pt))

    # ----------------------- For beta ----------------

    if 0.5 * threshold_angle <= beta < 1.5 * threshold_angle:
        beta_index = 1

    elif 1.5 * threshold_angle <= beta < 2.5 * threshold_angle:
        beta_index = 2

    elif 2.5 * threshold_angle <= beta < 3.5 * threshold_angle:
        beta_index = 3

    elif 3.5 * threshold_angle <= beta < 4.5 * threshold_angle:
        beta_index = 4

    elif 4.5 * threshold_angle <= beta < 5.5 * threshold_angle:
        beta_index = 5

    elif 5.5 * threshold_angle <= beta < 6.5 * threshold_angle:
        beta_index = 6

    elif 6.5 * threshold_angle <= beta < 7.5 * threshold_angle:
        beta_index = 7

    elif 7.5 * threshold_angle <= beta < 8.5 * threshold_angle:
        beta_index = 8

    elif 8.5 * threshold_angle <= beta < 9.5 * threshold_angle:
        beta_index = 9

    elif 9.5 * threshold_angle <= beta < 10.5 * threshold_angle:
        beta_index = 10

    elif 10.5 * threshold_angle <= beta < 11.5 * threshold_angle:
        beta_index = 11

    else:
        beta_index = 0

    node_index = x_index * len(Y) * len(angles) + y_index * len(angles) + beta_index

    return node_index, x_index, y_index, beta_index


def a_star(parent_index, length, alpha):
    x_parent, y_parent = node_pts[parent_index]
    parent_angle = nodes_angle[parent_index]
    parent_cost = nodes_cost[parent_index]

    # ------------------------------------------ Performing action sets ------------------------------------------------

    x_actions = np.zeros(12)
    y_actions = np.zeros(12)
    alpha_actions = np.zeros(12)

    for j in range(len(alpha_actions)):
        alpha_actions[j] = (parent_angle + j * alpha) % (2 * np.pi)
        x_actions[j] = x_parent + length * np.cos(alpha_actions[j])
        y_actions[j] = y_parent + length * np.sin(alpha_actions[j])

        is_valid = workspace_check(x_actions[j], y_actions[j]) and obstacle_space(x_actions[j], y_actions[j])

        if not is_valid:
            continue

        neighbor_state = np.array([x_actions[j], y_actions[j], alpha_actions[j]])

        neighbor_index = find_index(neighbor_state)[0]

        if neighbor_index in visited_nodes_index:
            continue

        if nodes_cost[neighbor_index] > parent_cost + length:
            nodes_cost[neighbor_index] = parent_cost + length
            goal_cost[neighbor_index] = np.sqrt(np.sum((neighbor_state[:2] - goal_state[:2]) ** 2))
            track.update({neighbor_index: parent_index})
            node_pts[neighbor_index] = neighbor_state[:2]
            nodes_angle[neighbor_index] = neighbor_state[2]
            unvisited_nodes_index.append(neighbor_index)

    node_flag[parent_index] = 1
    unvisited_nodes_index.remove(parent_index)
    visited_nodes_index.append(parent_index)
    return 0


# ---------------------- Taking User Input for Start Point and checking for its validity -------------------------------

print('\nEnter Start location (X_pt): ')
X_start = float(input())
print('Enter Start location (Y_pt): ')
Y_start = float(input())

if workspace_check(X_start, Y_start):
    if not obstacle_space(X_start, Y_start):
        print('\n\nInvalid Start Point as it is in Obstacle/ Clearance Space\n\n')
        sys.exit('Exiting ....')

else:
    print('\n\nInvalid Start Point as it is not in Workspace\n\n')
    sys.exit('Exiting ....')

print('Enter Start angle (in degrees): ')
angle_start = (float(input()) % 360.0) * (np.pi / 180)  # converted to radian

start_state = np.array([X_start, Y_start, angle_start])

# ---------------------- Taking User Input for End Point and checking for its validity ---------------------------------

print('\nEnter Goal location (X_pt): ')
X_goal = float(input())
print('Enter Goal location (Y_pt): ')
Y_goal = float(input())

if workspace_check(X_goal, Y_goal):
    if not obstacle_space(X_goal, Y_goal):
        print('\n\nInvalid Goal Point as it is in Obstacle/ Clearance Space\n\n')
        sys.exit('Exiting ....')

else:
    print('\n\nInvalid Goal Point as it is not in Workspace\n\n')
    sys.exit('Exiting ....')

print('Enter Goal angle (in degrees): ')
angle_goal = (float(input()) % 360.0) * (np.pi / 180)  # converted to radian

goal_state = np.array([X_goal, Y_goal, angle_goal])

# ----------------------------------------------------------------------------------------------------------------------

threshold_distance = 0.5
threshold_angle = np.pi * (30 / 180)
length_vector = 7
goal_threshold_radius = 1.5

X = np.linspace(0, 400, int(400 / threshold_distance) + 1)
Y = np.linspace(0, 300, int(300 / threshold_distance) + 1)
angles = np.linspace(0, 2 * np.pi, int(2 * np.pi / threshold_angle), endpoint=False)

node_flag = np.zeros((len(X) * len(Y) * len(angles)))
node_pts = np.zeros((len(X) * len(Y) * len(angles), 2))
nodes_cost = np.zeros(len(X) * len(Y) * len(angles)) + np.inf
goal_cost = np.zeros(len(X) * len(Y) * len(angles)) + np.inf
nodes_angle = np.zeros(len(X) * len(Y) * len(angles))

start_index = find_index(start_state)[0]
goal_index = find_index(goal_state)[0]

print('\n\nStart Node Index: ', start_index)
print('Goal Node Index: ', goal_index)

node_pts[start_index] = start_state[:2]
nodes_cost[start_index] = 0.0  # assigning cost of start node to zero
goal_cost[start_index] = np.sqrt(np.sum((start_state[:2] - goal_state[:2]) ** 2))
nodes_angle[start_index] = start_state[2]

visited_nodes_index = []
unvisited_nodes_index = [start_index]
track = {}

start_time = time.time()
iterator = 0
goal_flag = 0

X_start_next = X_start + length_vector * np.cos(angle_start)
Y_start_next = Y_start + length_vector * np.sin(angle_start)

if workspace_check(X_start_next, Y_start_next):
    if not obstacle_space(X_start_next, Y_start_next):
        print('\n\nInvalid starting vector as its head is in Obstacle/ Clearance Space\n\n')
        sys.exit('Exiting ....')

else:
    print('\n\nInvalid starting vector as it is not in Workspace\n\n')
    sys.exit('Exiting ....')

start_next_state = np.array([X_start_next, Y_start_next, angle_start])
start_next_index = find_index(start_next_state)[0]

print('Start Vector Node Index: ', start_next_index)

node_pts[start_next_index] = start_next_state[:2]
nodes_cost[start_next_index] = length_vector
goal_cost[start_next_index] = np.sqrt(np.sum((start_next_state[:2] - goal_state[:2]) ** 2))
nodes_angle[start_next_index] = start_next_state[2]

node_flag[start_index] = 1
unvisited_nodes_index.remove(start_index)
visited_nodes_index.append(start_index)
unvisited_nodes_index.append(start_next_index)
track.update({start_next_index: start_index})
iterator += 1

if goal_cost[start_next_index] <= goal_threshold_radius:
    goal_flag = 1

else:
    print('\n\nSolving.........')
    print('\nIteration # \t Time (mins.)\n')

while goal_flag == 0:

    if iterator % 5000 == 0 and iterator != 0:
        mid_time = (time.time() - start_time) / 60
        print(' {0} \t\t {1:1.3f}'.format(iterator, mid_time))

    temp_cost = nodes_cost[unvisited_nodes_index] + goal_cost[unvisited_nodes_index]
    temp = np.argmin(temp_cost)

    next_node_index = unvisited_nodes_index[temp]

    if goal_cost[next_node_index] <= goal_threshold_radius:
        goal_flag = 1
        iterator += 1
        end_time = time.time()
        total_time = (end_time - start_time) / 60
        print('\n\nNumber of iterations taken to reach goal state: {}'.format(iterator))
        print('\nTime taken to find optimal (shortest) path: {0:1.3f} min'.format(total_time))

        node_flag[next_node_index] = 1
        unvisited_nodes_index.remove(next_node_index)
        visited_nodes_index.append(next_node_index)
        print('\n\nRobot reached within the threshold of goal node ...!')
        print('\nCurrent node number for robot:', next_node_index)
        print('Location (x, y):', node_pts[next_node_index])
        print('Cost:', nodes_cost[next_node_index])
        break

    goal_flag = a_star(next_node_index, length_vector, threshold_angle)
    iterator += 1

# Visited Node Exploration
x_explore = []
y_explore = []
u_explore = []
v_explore = []

fname1 = './Visited_Nodes_Mobile_Robot_A-star.txt'
myfile1 = open(fname1, "w")
myfile1.write('Node Index \t X \t Y \t Angle (degrees) \t Cumulative Cost\n')

for i in range(1, len(visited_nodes_index)):
    child_node = visited_nodes_index[i]
    parent_node = track[child_node]
    x_temp, y_temp = node_pts[parent_node]
    final_angle = (180 / np.pi) * nodes_angle[parent_node]
    final_cost = nodes_cost[parent_node]
    u_temp, v_temp = node_pts[child_node]
    x_explore.append(x_temp)
    y_explore.append(y_temp)
    u_explore.append(u_temp)
    v_explore.append(v_temp)
    myfile1.write('{0}   \t {1:1.3f} \t {2:1.3f} \t {3:1.1f}  \t {4:1.1f}\n'.format(parent_node, x_temp, y_temp,
                                                                                    final_angle, final_cost))
    if i == len(visited_nodes_index) - 1:
        final_node_index = visited_nodes_index[-1]
        myfile1.write('{0}   \t {1:1.3f} \t {2:1.3f} \t {3:1.1f}  \t {4:1.1f}\n'.format(final_node_index,
                                                                                        node_pts[final_node_index][0],
                                                                                        node_pts[final_node_index][1],
                                                                                        (180 / np.pi) * nodes_angle[
                                                                                            final_node_index],
                                                                                        nodes_cost[final_node_index]))

# Optimal solution trajectory
back_track = []


def traj(child):
    if child != start_index:
        back_track.append(child)
        parent = track[child]
        return traj(parent)

    else:
        back_track.append(start_index)
        return back_track[::-1]


trajectory = traj(visited_nodes_index[-1])

x_solution = []
y_solution = []
u_solution = []
v_solution = []

fname2 = './Solution_Path_Mobile_Robot_A-star.txt'
myfile2 = open(fname2, "w")
myfile2.write('Time taken to solve:\t{0:1.3f} minutes\n'.format(total_time))
myfile2.write('\nRequired Solution trajectory\n')
myfile2.write('\nNode Index \t X \t Y \t Angle (degrees) \t Cumulative Cost\n')

for i in range(1, len(trajectory)):
    child_node = trajectory[i]
    parent_node = trajectory[i - 1]
    x_temp, y_temp = node_pts[parent_node]
    final_angle = (180 / np.pi) * nodes_angle[parent_node]
    final_cost = nodes_cost[parent_node]
    u_temp, v_temp = node_pts[child_node]
    x_solution.append(x_temp)
    y_solution.append(y_temp)
    u_solution.append(u_temp)
    v_solution.append(v_temp)
    myfile2.write('{0}   \t {1:1.3f} \t {2:1.3f} \t {3:1.1f}  \t {4:1.1f}\n'.format(parent_node, x_temp, y_temp,
                                                                                    final_angle, final_cost))
    if i == len(trajectory) - 1:
        final_node_index = trajectory[-1]
        myfile2.write('{0}   \t {1:1.3f} \t {2:1.3f} \t {3:1.1f}  \t {4:1.1f}\n'.format(final_node_index,
                                                                                        node_pts[final_node_index][0],
                                                                                        node_pts[final_node_index][1],
                                                                                        (180 / np.pi) * nodes_angle[
                                                                                            final_node_index],
                                                                                        nodes_cost[final_node_index]))

# --------------------------------------- Visualization starts from here -----------------------------------------------

print('\n\n### Creating Visualization ###')

start_time_plot = time.time()

plt.style.use('seaborn-pastel')

fig = plt.figure()

ax = plt.axes(xlim=(0, 400), ylim=(0, 300))  # Defining Workspace limits

# For Plotting Circle threshold for goal node
x_goal_circle = np.linspace(X_goal - goal_threshold_radius, X_goal + goal_threshold_radius, 2000)
y_goal_circle1 = Y_goal + (goal_threshold_radius ** 2 - (x_goal_circle - X_goal) ** 2) ** 0.5
y_goal_circle2 = Y_goal - (goal_threshold_radius ** 2 - (x_goal_circle - X_goal) ** 2) ** 0.5
ax.plot(x_goal_circle, y_goal_circle1, 'k.', markersize=20)
ax.plot(x_goal_circle, y_goal_circle2, 'k.', markersize=20)

# For Plotting Circle
x_circle = np.linspace(55, 125, 2000)
y_circle1 = 70 + (35 ** 2 - (x_circle - 90) ** 2) ** 0.5
y_circle2 = 70 - (35 ** 2 - (x_circle - 90) ** 2) ** 0.5
ax.plot(x_circle, y_circle1, 'b.', markersize=0.15)
ax.plot(x_circle, y_circle2, 'b.', markersize=0.15)

# For Plotting Ellipse
x_ellipse = np.linspace(186, 306, 2000)
y_ellipse1 = 145 + 30 * (1 - ((x_ellipse - 246) / 60) ** 2) ** 0.5
y_ellipse2 = 145 - 30 * (1 - ((x_ellipse - 246) / 60) ** 2) ** 0.5
ax.plot(x_ellipse, y_ellipse1, 'b.', markersize=0.15)
ax.plot(x_ellipse, y_ellipse2, 'b.', markersize=0.15)

# For C-Shape (assuming uniform thickness)
ax.axhline(y=280, xmin=0.50, xmax=0.575, color='blue')
ax.axvline(x=200, ymin=23 / 30, ymax=14 / 15, color='blue')
ax.axhline(y=230, xmin=0.50, xmax=0.575, color='blue')
ax.axvline(x=230, ymin=23 / 30, ymax=0.80, color='blue')
ax.axhline(y=240, xmin=0.525, xmax=0.575, color='blue')
ax.axvline(x=210, ymin=0.8, ymax=0.9, color='blue')
ax.axhline(y=270, xmin=0.525, xmax=0.575, color='blue')
ax.axvline(x=230, ymin=0.9, ymax=14 / 15, color='blue')

# For rotated rectangle
x1_, y1_ = (48, 108)
x2_, y2_ = (x1_ - 20 * np.cos(11 * np.pi / 36), y1_ + 20 * np.sin(11 * np.pi / 36))
x4_, y4_ = (x1_ + 150 * np.cos(7 * np.pi / 36), y1_ + 150 * np.sin(7 * np.pi / 36))
x3_, y3_ = (x4_ - 20 * np.cos(11 * np.pi / 36), y4_ + 20 * np.sin(11 * np.pi / 36))
ax.plot([x1_, x2_], [y1_, y2_], 'b-')
ax.plot([x2_, x3_], [y2_, y3_], 'b-')
ax.plot([x3_, x4_], [y3_, y4_], 'b-')
ax.plot([x1_, x4_], [y1_, y4_], 'b-')

quiver1 = ax.quiver(x_explore[0], y_explore[0], u_explore[0], v_explore[0], units='xy', scale_units='xy', scale=40,
                    color='green')


def animate(frame_number):
    """

        In this function, animation is carried out.

        Parameters
        ----------
        frame_number : int type, here frame number serves as an index for the images


        Returns
        -------
        None

    """

    global quiver1

    if 0 <= frame_number <= 148:  # will run for frame_number = [0, 148]
        quiver1.remove()
        first = 0
        last = step1 * (frame_number + 1)
        x = x_explore[first:last]
        y = y_explore[first:last]
        u = u_explore[first:last]
        v = v_explore[first:last]
        quiver1 = ax.quiver(x, y, u, v, units='xy', scale_units='xy', scale=40, color='green', alpha=0.9)
        return quiver1,

    elif frame_number == 149:  # will run for frame_number = 149 only
        quiver1.remove()
        x = x_explore
        y = y_explore
        u = u_explore
        v = v_explore
        quiver1 = ax.quiver(x, y, u, v, units='xy', scale_units='xy', scale=40, color='green', alpha=0.9)
        return quiver1,

    elif 150 <= frame_number <= 198:  # will run for frame_number = [150, 198]
        first = 0
        last = step2 * (frame_number - 149)
        x = x_solution[first:last]
        y = y_solution[first:last]
        u = u_solution[first:last]
        v = v_solution[first:last]
        quiver1 = ax.quiver(x, y, u, v, units='xy', scale_units='xy', scale=40, color='red')
        return quiver1,

    else:  # will run for frame_number = 199 only
        x = x_solution
        y = y_solution
        u = u_solution
        v = v_solution
        quiver1 = ax.quiver(x, y, u, v, units='xy', scale_units='xy', scale=40, color='red')
        return quiver1,


node_explore_frames = 150
solution_traj_frames = 50
total_frames = node_explore_frames + solution_traj_frames

step1 = int(len(x_explore) / node_explore_frames)
step2 = int(len(x_solution) / solution_traj_frames)

animation = FuncAnimation(fig, animate, frames=total_frames, interval=30, blit=True, repeat=False)

animation.save('Mobile Robot Visualization (A-star).mp4', dpi=300)

plt.close()

end_time_plot = time.time()
total_time_plot = (end_time_plot - start_time_plot) / 60
print('\n\nTime taken for making visualization: {0:1.3f} min'.format(total_time_plot))
