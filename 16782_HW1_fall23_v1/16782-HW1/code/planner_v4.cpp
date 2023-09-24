/*=================================================================
 *
 * planner.cpp
 * Heuristic of Chebyshev function
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
int track = 0;
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

struct path_element {
    std::vector<std::pair<int, int>> path;
    int cost;
    path_element(std::vector<std::pair<int, int>> _path, int _cost) : path(_path), cost(_cost) {}
    path_element() : path(), cost(0) {}
    bool operator<(const path_element& other) const {
        return cost < other.cost;
    }
};

struct NodeComparator {
    bool operator() (const Node& lhs, const Node& rhs) const {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
};

bool isValid(int x, int y, double* map, int collision_thresh, int x_size, int y_size) {
    return x > 0 && y > 0 && x < x_size && y < y_size && (map[GETMAPINDEX(x, y, x_size, y_size)]) < collision_thresh;
}

int calculateHeuristic(int x, int y, int target_x, int target_y) {
    // return 10*(abs(x - target_x) + abs(y - target_y));
    // return (int) 100*sqrt(pow(x - target_x, 2) + pow(y - target_y, 2));
    // return A_star(map, collision_thresh, x_size, y_size, target_x, target_y, target_steps, target_traj, x, y).second;
    return 100*std::max(abs(x - target_x), abs(y - target_y));
    // int delX = abs(x - target_x);
    // int delY = abs(y - target_y);
    // return 10*int(1.414*MIN(delX, delY) + MAX(delX, delY) - MIN(delX, delY));
}

bool compareByFValue(const Node& lhs, const Node& rhs) {
    return lhs.f < rhs.f;
}


std::pair<std::vector<std::pair<int, int>>, int> A_star(
    double*	map,            // Location of Map
    int collision_thresh,   // Collision Threshold
    int x_size,             // X Size of Map
    int y_size,             // Y Size of Map
    int startposeX,         // Robot Current X Position
    int startposeY,         // Robot Current Y Position
    int target_steps,       // Number of Steps in Target Trajectory
    double* target_traj,    // Target Trajectory
    int goalposeX,          // Target Current X Position
    int goalposeY           // Target Current Y Position
)
{
    // printf("A* called...\n");
    std::vector<std::pair<int, int>> directions = {{0, 0}, {-1, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    std::set<Node> openSet;
    std::set<Node> closedSet;
    NodeComparator comparator;
    std::vector<std::pair<int, int>> path;
    openSet.insert(Node(startposeX, startposeY, 0, calculateHeuristic(startposeX, startposeY, goalposeX, goalposeY), calculateHeuristic(startposeX, startposeY, goalposeX, goalposeY), nullptr));

    while (!openSet.empty()) {
        Node* current = new Node(*(openSet.begin()));   // Get the node with the lowest f value
        openSet.erase(openSet.begin());                 // Remove the node from the open set
        closedSet.insert(*current);                     // Add the node to the closed set

        // printf("Current node : (%d, %d) | Open Set : %d | Closed Set : %d\n" , (*current).x, (*current).y, openSet.size(), closedSet.size());
        if ((*current).x == goalposeX && (*current).y == goalposeY){  // If the current node is the target node
            int final_cost = (*current).g;
            // printf("Target node found | Final Cost = %d\n", final_cost);
            while ((*current).parent->x != startposeX || (*current).parent->y != startposeY){
                // printf("Path : (%d, %d)\n", (*current).x, (*current).y);
                path.push_back(std::make_pair((*current).x, (*current).y));
                current = (*current).parent;
            }
            path.push_back(std::make_pair((*current).x, (*current).y));
            std::reverse(path.begin(), path.end());
            // Print the path
            std::pair<std::vector<std::pair<int, int>>, int> result(path, final_cost);
            return result;
            break;
        }

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int new_x = (*current).x + directions[dir].first;
            int new_y = (*current).y + directions[dir].second;

            if (isValid(new_x, new_y, map, collision_thresh, x_size, y_size)) {
                // Create a new node neighbor with the current node as the parent
                int tentaive_cost = 1;
                if (map[GETMAPINDEX(new_x, new_y, x_size, y_size)] > 0){
                    tentaive_cost = 1000;
                }
                int _g = (*current).g + tentaive_cost;
                int _h = calculateHeuristic(new_x, new_y, goalposeX, goalposeY);
                Node neighbor(new_x, new_y, _g, _h, _g + _h, current);
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
    // return std::make_pair(path, 0);
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
    
    std::set<path_element> pathSet;
    // printf("Timestamp: %d\n", curr_time);
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("target: %d %d;\n", targetposeX, targetposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);
    
    int i = 1;
    int goalposeX;
    int goalposeY;
    if (PATHFLAG){
        while (i < target_steps/4){
            if(goalposeX == target_traj[target_steps-i] && goalposeY == target_traj[target_steps-i+target_steps]){
                ++i;
                continue;
            }
            goalposeX = (int) target_traj[target_steps-i];
            goalposeY = (int) target_traj[target_steps-i+target_steps];
            printf("Target Node %d: (%d,%d)\n", i, goalposeX, goalposeY);
            std::pair<std::vector<std::pair<int, int>>, int> result;
            result = A_star(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, goalposeX, goalposeY);
            int path_length = result.first.size();
            path_element* path_element_ptr = new path_element(result.first, result.second);
            pathSet.insert(*path_element_ptr);
            printf("Path Cost %d: %d | Target Node : (%d,%d) | Path Set Size : %d\n", i, result.second, result.first[path_length-1].first, result.first[path_length-1].second, pathSet.size());
            ++i;
            if (result.first.size() > target_steps ){
                break;
            }
            
        }
        PATHFLAG=false;
        path = (*(pathSet.begin())).path;
        printf("Chosen Path Target Node : (%d,%d)\n", path[path.size()-1].first, path[path.size()-1].second);
    }
    
    if(track < path.size()){
        action_ptr[0] = path[track].first;
        action_ptr[1] = path[track].second;
        track++;
        return;
    }
    else{
        action_ptr[0] = path[track-1].first;
        action_ptr[1] = path[track-1].second;
        return;
    }
    return;
}