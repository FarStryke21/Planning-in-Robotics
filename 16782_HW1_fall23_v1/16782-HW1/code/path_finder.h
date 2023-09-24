#ifndef PATH_FINDER_H
#define PATH_FINDER_H
#include <vector>

std::vector<std::pair<int, int>> A_star(
    double* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int startX,
    int startY,
    int targetposeX,
    int targetposeY
); 

#endif