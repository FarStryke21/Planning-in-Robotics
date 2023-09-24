/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <stdio.h>
#include <math.h>
#include <set>
#include <vector>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
#define PATHFLAG true
#define track 0

struct Node {
    int x, y;   // Coordinates of the node
    int g, h, f; // Cost values for path calculation

    // Constructor to initialize the node with x, y, g, and h
    Node(int _x, int _y, int _g, int _h, int _f) : x(_x), y(_y), g(_g), h(_h), f(_f) {}

    bool operator<(const Node& other) const {
        return f < other.f; // You can use any comparison that defines the order you want
    }
};

bool isValid(int x, int y, double* map, int collision_thresh, int x_size, int y_size) 
{
    return x >= 0 && y >= 0 && x < x_size && y < y_size && map[GETMAPINDEX(x, y, x_size, y_size)] < collision_thresh;
}

int calculateHeuristic(int x, int y, int target_x, int target_y) 
{
    return fmax(abs(x - target_x), abs(y - target_y));
}

void planner(
    double*	map,            // Location of Map
    int collision_thresh,   // Collision Threshold
    int x_size,             // X Size of Map
    int y_size,             // Y Size of Map
    int robotposeX,         // Robot Current X Position
    int robotposeY,         // Robot Current Y Position
    int target_steps,       // Number of Steps in Target Trajectory
    double* target_traj,    // Target Trajectory
    int targetposeX,        // Target Current X Position
    int targetposeY,        // Target Current Y Position
    int curr_time,          // Current Timestep
    double* action_ptr      // Action to be returned
    )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    // int goalposeX = (int) target_traj[target_steps-1];     
    // int goalposeY = (int) target_traj[target_steps-1+target_steps];
    targetposeX = (int) target_traj[curr_time];
    targetposeY = (int) target_traj[curr_time+target_steps];

    printf("Timestamp: %d\n", curr_time);
    printf("robot: %d %d;\n", robotposeX, robotposeY);
    printf("target: %d %d;\n", targetposeX, targetposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);.
    
    int target_track_x = (int) target_traj[curr_time+1];
    int target_track_y = (int) target_traj[curr_time+1+target_steps]; 
    // action_ptr[0] = robotposeX;
    // action_ptr[1] = robotposeY;

    // Implement A star algorithm here
    std::set<Node> openSet;
    std::set<Node> closedSet;

    int init_f = 0;
    int init_g = calculateHeuristic(robotposeX, robotposeY, target_track_x, target_track_y);
    openSet.insert(Node(robotposeX, robotposeY, init_f, init_g, init_f+init_g));
    // printf("Insert Complete\n");
    while (!openSet.empty()) 
    {
        // printf("In While Loop\n");
        Node current = *openSet.begin();
        openSet.erase(openSet.begin());
        
        closedSet.insert(current);
        
        if (current.x == target_track_x && current.y == target_track_y) {
            printf("Target Found. Tracing Back.\n");
            std::vector<std::pair<int, int>> path;
            while (current.x != robotposeX || current.y != robotposeY) {
                path.push_back({current.x, current.y});
                current = *closedSet.find(Node(current.x, current.y, 0, 0, 0));
            }
            path.push_back({robotposeX, robotposeY});
            std::reverse(path.begin(), path.end());
            std::pair<int, int> secondPair = path[2];
            printf("New Position: %d %d;\n", secondPair.first, secondPair.second);
            action_ptr[0] = secondPair.first;
            action_ptr[1] = secondPair.second;
        }
        
        for (int dir = 0; dir < NUMOFDIRS; dir++) 
        {
            int new_x = current.x + dX[dir];
            int new_y = current.y + dY[dir];
            // printf("Checking : %d, %d\n", new_x, new_y);
            if (isValid(new_x, new_y, map, collision_thresh, x_size, y_size))
            {
                // printf("Valid\n");
                Node neighbor(new_x, new_y, 0, 0, 0);
                int tentative_g = current.g + 1; // Assuming constant cost of movement
                
                if (closedSet.find(neighbor) != closedSet.end() && tentative_g >= neighbor.g) {
                    continue;
                }
                
                if (openSet.find(neighbor) == openSet.end() || tentative_g < neighbor.g) {
                    neighbor.g = tentative_g;
                    neighbor.h = calculateHeuristic(new_x, new_y, target_track_x, target_track_y);
                    neighbor.f = neighbor.g + neighbor.h;
                    
                    openSet.insert(neighbor);
                }
            }
        }
    }
    
    return;
}