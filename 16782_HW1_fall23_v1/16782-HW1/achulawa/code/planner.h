#ifndef PLANNER_H
#define PLANNER_H

// Declare the plan function
void planner(
    int*	map,
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
    );
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
);

#endif // PLANNER_H