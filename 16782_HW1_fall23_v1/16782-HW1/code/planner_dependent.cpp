/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include "path_finder.h"

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
    targetposeX = (int) target_traj[curr_time];
    targetposeY = (int) target_traj[curr_time+target_steps];

    int goalposeX = (int) target_traj[target_steps];
    int goalposeY = (int) target_traj[target_steps+target_steps];


    printf("Timestamp: %d\n", curr_time);
    printf("robot: %d %d;\n", robotposeX, robotposeY);
    printf("target: %d %d;\n", targetposeX, targetposeY);
    
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    
    std::pair<int, int> step;
    if (PATHFLAG)
    {
        printf("Planning...\n");
        path = A_star(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY);
        printf("Path Size: %d\n", path.size());
        PATHFLAG = false;
        step = path[0];
        printf("Step: %d %d\n", step.first, step.second);
        action_ptr[0] = step.first;
        action_ptr[1] = step.second;
    }
    else
    {
        //Extract the path from the path vector
        if(track < path.size() - 1)
        {
            step = path[track];
            action_ptr[0] = step.first;
            action_ptr[1] = step.second;
            track++;
        }
        else
        {
            // step = path[track-1];
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
        }
 
    }
    return;
}