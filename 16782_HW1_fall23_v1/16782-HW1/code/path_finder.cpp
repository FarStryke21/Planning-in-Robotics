// A star Algorithm

#include "path_finder.h"

#include <stdio.h>
#include <math.h>
#include <set>
#include <vector>
#include <algorithm>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct Node 
{
    int x, y;
    int g, h, f;
    std::pair<int, int> parent;

    Node(int _x, int _y, int _g, int _h, int _f, std::pair<int, int> _parent) : x(_x), y(_y), g(_g), h(_h), f(_f), parent(_parent) {}
    Node() : x(0), y(0), g(0), h(0), f(0), parent({-1, -1}) {}

    bool operator<(const Node& other) const 
    {
        if (f==other.f) {
            // If f values are equal, compare by h values as a secondary criterion
            if(h==other.h)
            {
                // If h values are equal, compare by g values as a tertiary criterion
                if(g==other.g)
                {
                    // If g values are equal, compare by y values as a quaternary criterion
                    if(y==other.y)
                    {
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

struct NodeComparator 
{
    bool operator() (const Node& lhs, const Node& rhs) const 
    {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
};

bool isValid(int x, int y, double* map, int collision_thresh, int x_size, int y_size) 
{
    return x >= 0 && y >= 0 && x < x_size && y < y_size && map[GETMAPINDEX(x, y, x_size, y_size)] < collision_thresh;
}

int calculateHeuristic(int x, int y, int target_x, int target_y) 
{
    return 1*(abs(x - target_x) + abs(y - target_y)); // Manhattan Distance
}

bool compareByFValue(const Node& lhs, const Node& rhs) 
{
    return lhs.f < rhs.f;
}

std::vector<std::pair<int, int>> A_star(
    double* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int startX,
    int startY,
    int targetposeX,
    int targetposeY
) 
{
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    std::set<Node> openSet;
    std::set<Node> closedSet;
    NodeComparator comparator;

    int g = 0;
    int h = calculateHeuristic(startX, startY, targetposeX, targetposeY);
    
    openSet.insert(Node(startX, startY, g, h, g+h, {-1, -1}));
    while (!openSet.empty()) 
    {
        Node current = *(openSet.begin());  // Get the node with the lowest f value
        openSet.erase(openSet.begin());     // Remove the node from the open set

        closedSet.insert(current);          // Add the node to the closed set
        // printf("Current node : (%d, %d) | Address : %p| Open Set : %d | Closed Set : %d\n" , current.x, current.y, (void*)&current, openSet.size(), closedSet.size());
        if (current.x == targetposeX && current.y == targetposeY)  // If the current node is the target node
        {
            // printf("Target node found\n");
            std::vector<std::pair<int, int>> path;
            while (current.x != startX || current.y != startY)
            {
                // printf("Parent Node : (%d, %d)\n", current.parent.first, current.parent.second);
                path.push_back({current.x, current.y});
                openSet.insert(closedSet.begin(), closedSet.end());
                //Search in the open and closed sets for the parent node
                for (const auto& node : openSet) 
                {
                    if (node.x == current.parent.first && node.y == current.parent.second) 
                    {
                        current = node;
                        break;
                    }
                }
            }
            //  reverse the path vector
            std::reverse(path.begin(), path.end());
            // printf("Next Node : (%d, %d) | Target at : (%d, %d)\n", current.x, current.y, targetposeX, targetposeY);
            // action_ptr[0] = current.x;
            // action_ptr[1] = current.y;
            return path;
        }

        for (int dir = 0; dir < NUMOFDIRS; dir++) 
        {
            int new_x = current.x + directions[dir].first;
            int new_y = current.y + directions[dir].second;

            if (isValid(new_x, new_y, map, collision_thresh, x_size, y_size)) 
            {
                // Create a new node neighbor with the current node as the parent
                g = current.g + 1; //map[GETMAPINDEX(new_x, new_y, x_size, y_size)];
                h = calculateHeuristic(new_x, new_y, targetposeX, targetposeY);
                Node neighbor(new_x, new_y, g, h, g + h, {current.x, current.y});

                // Loop through the closed set to check if the neighbor is already in it by using the comparator
                bool found = false;
                for (const auto& node : closedSet) 
                {
                    if (comparator(node, neighbor) == true) 
                    {
                        found = true;
                        break;
                    }
                }
                if (found)
                {
                    // printf("found in closed set\n");
                    continue;
                }

                found = false;
                for (const auto& node : openSet) 
                {
                    if (comparator(node, neighbor) == true) 
                    {
                        found = true;
                        if(neighbor.f >= node.f)
                        {
                            // printf("Node Exists but neighbour has a better path (%d, %d): %d\n", neighbor.x, neighbor.y, neighbor.f);
                            openSet.erase(node);
                            openSet.insert(neighbor);
                            break;
                        }
                        break;
                    }
                }
                if (!found)
                {
                    // printf("Node does not exist and is being added (%d, %d): %d\n", neighbor.x, neighbor.y, neighbor.f);
                    openSet.insert(neighbor);
                }              
            }
        }
    }
    return {}; // No path found
}