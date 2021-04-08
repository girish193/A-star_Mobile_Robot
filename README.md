# A-star_Mobile_Robot

## Run Code
Open the file "Mobile Robot (A-star).py" in an IDE (Spyder, VS Code etc) of your choice. Enter valid input such that point does not lie in obstacle space or outside of workspace. Also enter a valid angle for the starting vector. For invalid inputs it exits with invalid input prompt. The video visualization can be found in 'Mobile Robot Visualization (A-star).mp4'.

## Description
This code aims at implementing A* algorithm for a mobile robot to solve a given map. All the nodes corresponding with different points (x,y) on the map are explored until a goal is found.

## Dependencies

a) python -version 3

b) numpy

c) sys

d) matplotlib

e) time

## Function Descriptions
### 1) workspace_check
In this functiom the given value is checked to see if it lies in the workspace.

### 2) obstacle_space
In this function the given value is checked against obstacles and true is returned if it lies outside obstacle space and also takes into consideration the clearance needed for the mobile robot. Obstacles which are given are C shaped polygon, rounded rectangle, ellipse, and circle.

### 3) find_index
For a given (x, y, angle) node_state, it finds its corresponding node index. 

### 4) a_star
In this function the initial input values (x, y, theta) are taken and action sets (based on angle between vectors & vector magnitude) are performed to generate next set of moves. Only valid actions are stored in an 'unvisited_nodes_index' list. Finally, the node_flag is updated to one for the parent_node index. 

## NOTE :

-- angle between action set vectors = 30 degrees

-- action set vector magnitude = 7 units

-- Threshold Distance = 0.5 units

-- Threshold Angle = 30 degrees

-- Goal Threshold Radius = 1.5 units
 
Using Matplotlib, animation is generated. Firstly, animation for node exploration is generated and followed by optimal path trajectory's animation. 150 frames for node exploration and 50 frames for solution trajectory are used as default values for the animation. The resulting animation is also stored in Mobile Robot Visualization (A-star).mp4 file.

If you get error while trying to save the animation in .mp4 file format, installation of FFMPEG may be required. [https://ffmpeg.org/download.html]
