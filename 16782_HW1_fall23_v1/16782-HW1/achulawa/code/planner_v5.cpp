/*=================================================================
 *
 * planner.cpp
 * Octal heuristics and highly optimised code
 *=================================================================*/

#include <stdio.h>
#include <math.h>
#include <set>
#include <vector>
#include "planner.h"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

bool PATHFLAG = true;
int track = 1;
std::vector<std::pair<int, int>> path;

struct Node {
    int x, y;
    int g, h, f;
    Node* parent;

    Node(int _x, int _y, int _g, int _h, int _f, Node* _parent) : x(_x), y(_y), g(_g), h(_h), f(_f), parent(_parent) {}
    Node() : x(0), y(0), g(0), h(0), f(0), parent(nullptr) {}

    bool operator<(const Node& other) const {
        if (f==other.f) {
            // If f values are equal, compare by h values as a secondary criterion
            if(h==other.h){
                // If h values are equal, compare by g values as a tertiary criterion
                if(g==other.g){
                    // If g values are equal, compare by y values as a quaternary criterion
                    if(y==other.y){
                        // If g values are equal, compare by x values as a centnary criterion
                        return x < other.x;
                    }
                    return y < other.y;
                }   
                return g < other.g;
            }
            return h < other.h; 
        }
        return f < other.f; 
    }
};

bool isValid(int x, int y, int* map, int collision_thresh, int x_size, int y_size) {
    return x > 0 && y > 0 && x < x_size && y < y_size && (map[GETMAPINDEX(x, y, x_size, y_size)]) < collision_thresh;
}

int calculateHeuristic(int x, int y, int target_x, int target_y, int* map, int collision_thresh, int x_size, int y_size, int target_steps, int* target_traj) {
    return 2* int(1.4142*MIN(abs(x - target_x), abs(y - target_y)) + abs(abs(x - target_x) - abs(y - target_y)));
}

std::pair<std::vector<std::pair<int, int>>, int> A_star(
    int*	map,            // Location of Map
    int collision_thresh,   // Collision Threshold
    int x_size,             // X Size of Map
    int y_size,             // Y Size of Map
    int startposeX,         // Robot Current X Position
    int startposeY,         // Robot Current Y Position
    int target_steps,       // Number of Steps in Target Trajectory
    int* target_traj,    // Target Trajectory
    int goalposeX,          // Target Current X Position
    int goalposeY           // Target Current Y Position 
)
{
    printf("A* called...\n");
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    std::set<Node> openSet;
    std::set<Node> closedSet;
    std::vector<std::pair<int, int>> path;

    int g = 0;
    int h = calculateHeuristic(startposeX, startposeY, goalposeX, goalposeY, map, collision_thresh, x_size, y_size, target_steps, target_traj);
    openSet.insert(Node(startposeX, startposeY, g, h, g+h, nullptr));

    while (!openSet.empty()) {
        Node* current = new Node(*(openSet.begin()));   // Get the node with the lowest f value
        openSet.erase(openSet.begin());                 // Remove the node from the open set
        closedSet.insert(*current);                     // Add the node to the closed set
        // printf("Current: %d %d | Open Set = %d | Closed Set = %d\n", current->x, current->y, openSet.size(), closedSet.size());
        if (current->x == goalposeX && current->y == goalposeY) {
            // printf("Path found!\n");
            int cost = current->g;
            while (current->parent != nullptr) {
                path.push_back({current->x, current->y});
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return std::make_pair(path, cost);
       }

       for(int dir = 0; dir < NUMOFDIRS; dir++){
            int newx = current->x + directions[dir].first;
            int newy = current->y + directions[dir].second;
            // printf("New: %d %d\n", newx, newy);
            if (isValid(newx, newy, map, collision_thresh, x_size, y_size)) {
                int g = current->g + 1;
                int h = calculateHeuristic(newx, newy, goalposeX, goalposeY, map, collision_thresh, x_size, y_size, target_steps, target_traj);
                Node* neighbor = new Node(newx, newy, g, h , g+h, current);
                if (std::find_if(closedSet.begin(), closedSet.end(), [&](const Node& node) {return node.x == neighbor->x && node.y == neighbor->y;}) != closedSet.end()) {
                    continue;
                }
                // printf("Element not in Closed Set...\n");
                if(std::find_if(openSet.begin(), openSet.end(), [&](const Node& node) {return node.x == neighbor->x && node.y == neighbor->y;}) == openSet.end()) {
                    // printf("Element not in Open Set...\n");
                    Node* old_neighbor = new Node(*(std::find_if(openSet.begin(), openSet.end(), [&](const Node& node) {return node.x == neighbor->x && node.y == neighbor->y && neighbor->f < node.f;})));
                    if (neighbor->f < old_neighbor->f) {
                        openSet.erase(*old_neighbor);
                        openSet.insert(*neighbor);
                    }
                }
                else{
                    // printf("Element exists in Open Set...\n");
                }
            }
        }
        
    }
    return std::make_pair(path, 0);
}

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];

    printf("Timestamp: %d | Robot %d, %d | Target: %d, %d\n ", curr_time, robotposeX, robotposeY, targetposeX, targetposeY);
    printf("goal: %d %d;\n", goalposeX, goalposeY);
    
    if (robotposeX == goalposeX && robotposeY == goalposeY){
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    
    if (PATHFLAG){
        PATHFLAG = false;
        path = A_star(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, goalposeX, goalposeY).first;
        printf("Path Size: %d\n", path.size());
        action_ptr[0] = path[0].first;
        action_ptr[1] = path[0].second;
        return;
    }
    else{
        if(track < path.size()){
            action_ptr[0] = path[track].first;
            action_ptr[1] = path[track].second;
            track++;
        }
        else{
             action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
        }
        return;
    }

    
    // goalposeX = (int) target_traj[curr_time+1];
    // goalposeY = (int) target_traj[curr_time+1+target_steps];
    // path = A_star(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, goalposeX, goalposeY).first;
    // action_ptr[0] = path[0].first;
    // action_ptr[1] = path[0].second;
    // return;
}