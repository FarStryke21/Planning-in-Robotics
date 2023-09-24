/*=================================================================
 *
 * planner.cpp
 * Weighted Euclidean distance heuristic and penalised travel cost
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

struct NodeComparator {
    bool operator() (const Node& lhs, const Node& rhs) const {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
};

bool isValid(int x, int y, double* map, int collision_thresh, int x_size, int y_size) {
    return x >= 0 && y >= 0 && x < x_size && y < y_size && (1*map[GETMAPINDEX(x, y, x_size, y_size)]) < collision_thresh;
}

int calculateHeuristic(int x, int y, int target_x, int target_y) {
    // return 10*(abs(x - target_x) + abs(y - target_y));
    return (int) 10*sqrt(pow(x - target_x, 2) + pow(y - target_y, 2));
}

bool compareByFValue(const Node& lhs, const Node& rhs) {
    return lhs.f < rhs.f;
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
    ){

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    int goalposeT = curr_time+1;

    printf("Timestamp: %d\n", curr_time);
    printf("robot: %d %d;\n", robotposeX, robotposeY);
    printf("target: %d %d;\n", targetposeX, targetposeY);
    printf("goal: %d %d;\n", goalposeX, goalposeY);
    
    if (robotposeX == goalposeX && robotposeY == goalposeY){
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    
    if (PATHFLAG){
        PATHFLAG = false;
        std::vector<std::pair<int, int>> directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
        std::set<Node> openSet;
        std::set<Node> closedSet;
        NodeComparator comparator;

        int g = 0;
        int h = calculateHeuristic(robotposeX, robotposeY, goalposeX, goalposeY);
        openSet.insert(Node(robotposeX, robotposeY, g, h, g+h, nullptr));

        while (!openSet.empty()) {
            Node* current = new Node(*(openSet.begin()));   // Get the node with the lowest f value
            openSet.erase(openSet.begin());                 // Remove the node from the open set
            closedSet.insert(*current);                     // Add the node to the closed set

            // printf("Current node : (%d, %d) | Open Set : %d | Closed Set : %d\n" , (*current).x, (*current).y, openSet.size(), closedSet.size());
            if ((*current).x == goalposeX && (*current).y == goalposeY){  // If the current node is the target node
                printf("Target node found\n");
                while ((*current).parent->x != robotposeX || (*current).parent->y != robotposeY){
                    // printf("Path : (%d, %d)\n", (*current).x, (*current).y);
                    path.push_back(std::make_pair((*current).x, (*current).y));
                    current = (*current).parent;
                }
                path.push_back(std::make_pair((*current).x, (*current).y));
                std::reverse(path.begin(), path.end());
                // Print the path
                for (int i = 0; i < path.size(); i++){
                    printf("Path : (%d, %d)\n", path[i].first, path[i].second);
                }
                break;
            }

            for (int dir = 0; dir < NUMOFDIRS; dir++) {
                int new_x = (*current).x + directions[dir].first;
                int new_y = (*current).y + directions[dir].second;

                if (isValid(new_x, new_y, map, collision_thresh, x_size, y_size)) {
                    // Create a new node neighbor with the current node as the parent
                    g = (*current).g + int(map[GETMAPINDEX(new_x, new_y, x_size, y_size)]/collision_thresh);
                    h = calculateHeuristic(new_x, new_y, goalposeX, goalposeY);
                    Node neighbor(new_x, new_y, g, h, g + h, current);

                    // Loop through the closed set to check if the neighbor is already in it by using the comparator
                    bool found = false;
                    for (const auto& node : closedSet) {
                        if (comparator(node, neighbor) == true) {
                            found = true;
                            break;
                        }
                    }
                    if (found){
                        // printf("found in closed set\n");
                        continue;
                    }

                    found = false;
                    for (const auto& node : openSet) {
                        if (comparator(node, neighbor) == true) {
                            found = true;
                            if(neighbor.f >= node.f){
                                // printf("Node Exists but neighbour has a better path (%d, %d): %d\n", neighbor.x, neighbor.y, neighbor.f);
                                openSet.erase(node);
                                openSet.insert(neighbor);
                                break;
                            }
                            break;
                        }
                    }
                    if (!found){
                        openSet.insert(neighbor);
                    }              
                }
            }
        }
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
}