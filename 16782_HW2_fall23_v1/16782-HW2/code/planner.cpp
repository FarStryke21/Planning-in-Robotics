/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;
using namespace std;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            // cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//**************************************MY FUNCTIONS*****************************************************************//
struct Node {
    double* angles;
    Node* parent;
	double cost;
	vector<Node*> neighbors;
	Node(double* angles, Node* parent) : angles(angles), parent(parent) { cost = std::numeric_limits<double>::infinity();}
	Node(double* angles, Node* parent, double cost) : angles(angles), parent(parent), cost(cost) {}
	Node(double* angles, vector<Node*> neighbors) : angles(angles), neighbors(neighbors) {}
};

// Helper function to generate a random valid configuration
double* getRandomConfiguration(int numofDOFs) {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 2 * M_PI);

    double* config = new double[numofDOFs];
    for (int i = 0; i < numofDOFs; i++) {
        config[i] = dis(gen);
    }
    return config;
}

double* getRandomConfiguration_Biased(int numofDOFs, double* goalConfig) {
    double tuning_param = 0.35;
	double* config = new double[numofDOFs];
	double value = ((double) rand() / (RAND_MAX));
	if (value < tuning_param)
	{
		for (int i = 0; i < numofDOFs; i++) 
		{
			config[i] = goalConfig[i];
		}
	}
	else
	{
		random_device rd;
		mt19937 gen(rd());
		uniform_real_distribution<> dis(0.0, 2 * M_PI);
		for (int i = 0; i < numofDOFs; i++) 
		{
			config[i] = dis(gen);
		}
	}
    return config;
}

// Function to find the nearest neighbor in the tree to a given configuration
Node* nearestNeighbor(double* sample, vector<Node*>& tree, int numofDOFs) {
    Node* nearest = nullptr;
    double minDistance = numeric_limits<double>::max();

    for (Node* node : tree) {
        double distance = 0;
        for (int i = 0; i < numofDOFs; i++) 
		{ 
            double diff = sample[i] - node->angles[i];
            distance += diff * diff;
        }
        if (distance < minDistance) 
		{
            minDistance = distance;
            nearest = node;
        }
    }

    return nearest;
}

// Function to extend the tree towards a random sample
Node* extendTree(double* randomSample, vector<Node*>& tree, double stepSize, int numofDOFs) {
    double* angles = new double[numofDOFs];
	Node* nearest = nearestNeighbor(randomSample, tree, numofDOFs);

    for (int i = 0; i < numofDOFs; i++) {
        angles[i] = nearest->angles[i] + stepSize * (randomSample[i] - nearest->angles[i]);
    }
	Node* newNode = new Node{angles, nearest};
    return newNode;
}

void extendTree_RRTConnect(std::vector<Node*>& treeFrom, std::vector<Node*>& treeTo, double stepSize, int numofDOFs, double* map, int x_size, int y_size) {
    // Generate a random sample
    // double* randomSample = getRandomConfiguration_Biased(numofDOFs, treeTo.back()->angles);
	double* randomSample = getRandomConfiguration(numofDOFs);

    // Use extendTree to extend treefrom towards randomSample
	Node* newNode = extendTree(randomSample, treeFrom, stepSize, numofDOFs);

	// If newNode has valid configuration, add it to treeFrom, else exit
	if(IsValidArmConfiguration(newNode->angles, numofDOFs, map, x_size, y_size))
	{
		treeFrom.push_back(newNode);

		// find the nearest node in treeTo to the new node
		Node* nearest = nearestNeighbor(newNode->angles, treeTo, numofDOFs);

		// from treeTo_newNode, interpolate towards newNode in step sizes until either the new node is reached or an obstacle is hit
		double* angles = new double[numofDOFs];
		for (int i = 0; i < numofDOFs; i++) 
		{
			angles[i] = nearest->angles[i];
		}
		Node* treeTo_newNode = new Node{angles, nearest};
		while(!equalDoubleArrays(treeTo_newNode->angles, newNode->angles, numofDOFs))
		{	
			double* new_angles = new double[numofDOFs];
			for (int i = 0; i < numofDOFs; i++) 
			{
				new_angles[i] = treeTo_newNode->angles[i] + stepSize * (newNode->angles[i] - nearest->angles[i]);
			}	
			if(!IsValidArmConfiguration(new_angles, numofDOFs, map, x_size, y_size))
				break;
			else
			{
				for (int i = 0; i < numofDOFs; i++) 
				{
					treeTo_newNode->angles[i] = new_angles[i];
				}
			}
		}
		// Node* treeTo_newNode = new Node{angles, nearest};
		treeTo.push_back(treeTo_newNode);

		return;
	}
	else
	{
		return;
	}	
}

// Function to check if the edge is valid
bool isValidEdge(Node* node1, Node* node2, double* map, int x_size, int y_size) 
{
	double* angles1 = node1->angles;
	double* angles2 = node2->angles;

	double x0 = ((double)x_size)/2.0;
	double y0 = 0;
	for (int i = 0; i < 5; i++) 
	{
		double x1 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles1[i]);
		double y1 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles1[i]);
		double x2 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles2[i]);
		double y2 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles2[i]);

		if (!IsValidLineSegment(x1, y1, x2, y2, map, x_size, y_size)) 
		{
			return false;
		}

		x0 = x1;
		y0 = y1;
	}

	return true;
}

// Function to find nodes within a certain radius of a given node
vector<Node*> findNearNodes(vector<Node*>& tree, Node* newNode, double radius) 
{
    vector<Node*> nearNodes;
    for (Node* node : tree) 
	{
        if (distance(node->angles, newNode->angles) <= radius) 
		{
            nearNodes.push_back(node);
        }
    }
    return nearNodes;
}

// Function to compute the Euclidean distance between two configurations
double distance(double* angles1, double* angles2, int numofDOFs) 
{
    double distanceSquared = 0;
    for (int i = 0; i < numofDOFs; i++) 
	{
        double diff = angles1[i] - angles2[i];
        distanceSquared += diff * diff;
    }
    return sqrt(distanceSquared);
}

// Function to find the five nearest neighbour of the given node
vector<Node*> findNearestNeighbors(Node* node, vector<Node*>& roadmap, int k, int numofDOFs) 
{
	vector<Node*> nearestNeighbors;
	vector<double> distances;

	for (Node* neighbor : roadmap) 
	{
		if (neighbor != node) 
		{
			double distance = 0;
			for (int i = 0; i < numofDOFs; i++) 
			{
				double diff = node->angles[i] - neighbor->angles[i];
				distance += diff * diff;
			}
			distances.push_back(distance);
		}
	}
	sort(distances.begin(), distances.end());
	for (int i = 0; i < k; i++) 
	{
		for (Node* neighbor : roadmap) 
		{
			if (neighbor != node) 
			{
				double distance = 0;
				for (int i = 0; i < 5; i++) 
				{
					double diff = node->angles[i] - neighbor->angles[i];
					distance += diff * diff;
				}
				if (distance == distances[i]) 
				{
					nearestNeighbors.push_back(neighbor);
				}
			}
		}
	}
	return nearestNeighbors;
}

// Function to generate PRM roadmap
vector<Node*> generatePRMRoadmap(int numNodes, int k, double* map, int x_size, int y_size, int numofDOFs) 
{
    vector<Node*> roadmap;

	// printf("Generating PRM roadmap Nodes...\n");
    for (int i = 0; i < numNodes; ++i) 
	{
		// if (i % 10000 == 0)
		// {
		// 	printf("Iteration %d\n", i);
		// }
        double* randomSample = getRandomConfiguration(numofDOFs);
        if (IsValidArmConfiguration(randomSample, numofDOFs, map, x_size, y_size)) 
		{
            Node* newNode = new Node{randomSample, nullptr};
            roadmap.push_back(newNode);
        }
    }

	// printf("Generating PRM roadmap Edges...\n");
    for (Node* node : roadmap) 
	{
        vector<Node*> nearestNeighbors = findNearestNeighbors(node, roadmap, k, numofDOFs);
        for (Node* neighbor : nearestNeighbors) 
		{
            if (isValidEdge(node, neighbor, map, x_size, y_size)) 
			{
                node->neighbors.push_back(neighbor);
                neighbor->neighbors.push_back(node);
            }
        }
    }

    return roadmap;
}

// The Dijkstra algorithm
vector<Node*> dijkstra(Node* startNode, Node* goalNode) 
{
	vector<Node*> path;
	vector<Node*> openSet;
	vector<Node*> closedSet;

	startNode->cost = 0;
	openSet.push_back(startNode);

	while (!openSet.empty()) {
		Node* currentNode = openSet[0];
		int currentIndex = 0;
		for (int i = 0; i < openSet.size(); i++) {
			if (openSet[i]->cost < currentNode->cost) {
				currentNode = openSet[i];
				currentIndex = i;
			}
		}

		openSet.erase(openSet.begin() + currentIndex);
		closedSet.push_back(currentNode);

		if (currentNode == goalNode) {
			Node* current = currentNode;
			while (current != nullptr) {
				path.push_back(current);
				current = current->parent;
			}
			reverse(path.begin(), path.end());
			return path;
		}

		for (Node* neighbor : currentNode->neighbors) {
			if (find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) {
				continue;
			}

			double newCost = currentNode->cost + distance(currentNode->angles, neighbor->angles);
			if (newCost < neighbor->cost) {
				neighbor->cost = newCost;
				neighbor->parent = currentNode;
				if (find(openSet.begin(), openSet.end(), neighbor) == openSet.end()) {
					openSet.push_back(neighbor);
				}
			}
		}
	}
	return path;
}

// Implement A star algorithm for the roadmap search
vector<Node*> aStar(Node* startNode, Node* goalNode) 
{
	vector<Node*> path;
	vector<Node*> openSet;
	vector<Node*> closedSet;

	startNode->cost = 0;
	openSet.push_back(startNode);

	while (!openSet.empty()) {
		Node* currentNode = openSet[0];
		int currentIndex = 0;
		for (int i = 0; i < openSet.size(); i++) {
			if (openSet[i]->cost < currentNode->cost) {
				currentNode = openSet[i];
				currentIndex = i;
			}
		}

		openSet.erase(openSet.begin() + currentIndex);
		closedSet.push_back(currentNode);

		if (currentNode == goalNode) {
			Node* current = currentNode;
			while (current != nullptr) {
				path.push_back(current);
				current = current->parent;
			}
			reverse(path.begin(), path.end());
			return path;
		}

		for (Node* neighbor : currentNode->neighbors) {
			if (find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) {
				continue;
			}

			double newCost = currentNode->cost + distance(currentNode->angles, neighbor->angles);
			if (newCost < neighbor->cost) {
				neighbor->cost = newCost;
				neighbor->parent = currentNode;
				if (find(openSet.begin(), openSet.end(), neighbor) == openSet.end()) {
					openSet.push_back(neighbor);
				}
			}
		}
	}
	return path;
}

// Function to find path using PRM roadmap
vector<Node*> findPathPRM(Node* startNode, Node* goalNode) 
{
    // vector<Node*> path = dijkstra(startNode, goalNode);
	vector<Node*> path = aStar(startNode, goalNode);
    return path;
}

// Implement Short Cutting for the path where the path is a vector of double*. Return the shortcut path as a vector of double*, shortcutting the path by removing nodes that are not necessary
vector<double*> shortCut(vector<double*>& path, double* map, int x_size, int y_size, int numofDOFs) 
{
	vector<double*> shortcutPath;
	shortcutPath.push_back(path[0]);
	int i = 1;
	while(i <  path.size() - 2)
	{
		double* config1 = path[i-1];
		double* config2 = path[i + 1];
		while(isValidEdge(new Node{config1, nullptr}, new Node{config2, nullptr}, map, x_size, y_size))
		{
			i++;
			config2 = path[i + 1];
		}
		shortcutPath.push_back(config2);
	}
	shortcutPath.push_back(path[path.size() - 1]);
	return shortcutPath;
}

// Function to generate 40 valid sample confgurations of the arm of given DOF.
double** generateValidSamples(int numofDOFs, double* armgoal_anglesV_rad, double* map, int x_size, int y_size) {
	double** validSamples = new double*[40];
	int count = 0;
	while (count < 40) {
		double* randomSample = getRandomConfiguration(numofDOFs);
		if (IsValidArmConfiguration(randomSample, numofDOFs, map, x_size, y_size)) {
			validSamples[count] = randomSample;
			count++;
		}
	}
	// Print valid samples seperated by commas
	for (int i = 0; i < 40; i++) {
		if (i%2 != 0)
			printf("\"");
		else
			printf("[\"./map2.txt\", \"");
		for (int j = 0; j < numofDOFs; j++) 
		{
			printf("%f", validSamples[i][j]);
			if (j != numofDOFs - 1)
				printf(",");
		}
		if (i%2 != 0)
			printf("\"],\n");
		else
			printf("\", ");
	}
	return validSamples;
}
//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	printf("planner call...\n");
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
	printf("the distance between the configuration is %f\n", distance);
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
	printf("the number of samples is %d\n", numofsamples);
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    printf("planner: plan is of length=%d\n", *planlength);
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    // Initialize tree with the start configuration
    Node* startNode = new Node{armstart_anglesV_rad, nullptr};
    vector<Node*> tree = {startNode};

	// Set a flag
	int flag = 0;

    // Set a step size for extending the tree
    double stepSize = 0.15; 

    // Define the maximum number of iterations (you may adjust this based on your needs)
    int maxIterations = 30000;

    while(flag == 0 && tree.size() < maxIterations) {
        // Generate a random sample
		// printf("Iteration %ld\n", tree.size());
        // double* randomSample = getRandomConfiguration(numofDOFs);
		
		double* randomSample = getRandomConfiguration_Biased(numofDOFs, armgoal_anglesV_rad);
		// printf("Random Sample Generated\n");
		Node* newNode = extendTree(randomSample, tree, stepSize, numofDOFs);
		// printf("Tree Extended\n");

		if(IsValidArmConfiguration(newNode->angles, numofDOFs, map, x_size, y_size)) 
		{	
			// Print the angles of the current nodes

			// printf("Valid configuration!| Tree: %ld | Angles : [%f, %f, %f, %f, %f]\n", tree.size(), newNode->angles[0], newNode->angles[1], newNode->angles[2], newNode->angles[3], newNode->angles[4]);
			tree.push_back(newNode);
			// printf("Valid Configuration -> Node Added to Tree\n");
			if (equalDoubleArrays(newNode->angles, armgoal_anglesV_rad, numofDOFs)) {
				// Add the goal configuration to the tree
				flag = 1;
				Node* goalNode = new Node{armgoal_anglesV_rad, newNode};
				tree.push_back(goalNode);
				break;
			}
		}
		// printf("Iteration Complete!\n");
	}

	// Find the path from the goal configuration to the start configuration
	if(flag == 1)
	{
		// printf("Path found!\n");
		vector<double*> path;
		Node* currentNode = tree.back();
		while (currentNode != nullptr) {
			path.push_back(currentNode->angles);
			currentNode = currentNode->parent;
		}
		reverse(path.begin(), path.end());

		// Set the plan and plan length
		*plan = new double*[path.size()];
		for (int i = 0; i < path.size(); i++) {
			(*plan)[i] = path[i];
		}
		*planlength = path.size();
		
	}
	else
	{
		printf("Trash Tree -> Restarting!\n");
		plannerRRT(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
		// *plan = NULL;
		// *planlength = 0;
		// printf("No path found!\n");
	}

}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	// Initialize tree with the start configuration
    Node* startNode = new Node{armstart_anglesV_rad, nullptr};
    std::vector<Node*> treeStart = {startNode};
    Node* goalNode = new Node{armgoal_anglesV_rad, nullptr};
    std::vector<Node*> treeGoal = {goalNode};

    // Set a step size for extending the tree
    double stepSize = 0.05;

    // Define the maximum number of iterations (you may adjust this based on your needs)
    int maxIterations = 100000;
	int i = 0;
    while(i < maxIterations) 
	{
		// printf("Iteration %d | Start tree : %ld | Goal Tree : %ld \n", i, treeStart.size(), treeGoal.size());
        // Alternate between extending trees
        if (i % 2 == 0) {
            extendTree_RRTConnect(treeStart, treeGoal, stepSize, numofDOFs, map, x_size, y_size);
			// printf("Start Tree Extended | Start tree : %ld | Goal Tree : %ld \n", treeStart.size(), treeGoal.size());
		
        } 
		else {
            extendTree_RRTConnect(treeGoal, treeStart, stepSize, numofDOFs, map, x_size, y_size);
			// printf("Goal Tree Extended | Start tree : %ld | Goal Tree : %ld \n", treeStart.size(), treeGoal.size());
        }

        // Check if trees have connected
        if (equalDoubleArrays(treeStart.back()->angles, treeGoal.back()->angles, numofDOFs)) 
		{
            // create two paths, one from the start tree and one from the goal tree. Flip the goal tree path, and concatenate the two paths after removing the duplicate node and linking the pointer of the last element of start tree path to the last element of the goal tree path
			vector<double*> pathStart;
			Node* currentNode = treeStart.back();
			while (currentNode != nullptr) {
				pathStart.push_back(currentNode->angles);
				currentNode = currentNode->parent;
			}
			reverse(pathStart.begin(), pathStart.end());

			vector<double*> pathGoal;
			currentNode = treeGoal.back();
			while (currentNode != nullptr) 
			{
				pathGoal.push_back(currentNode->angles);
				currentNode = currentNode->parent;
			}
			// Remove the duplicate node
			pathGoal.erase(pathGoal.begin());

			// Concatenate the two paths
			pathStart.insert(pathStart.end(), pathGoal.begin(), pathGoal.end());

			// Set the plan and plan length
			*plan = new double*[pathStart.size()];
			for (int i = 0; i < pathStart.size(); i++) 
			{
				(*plan)[i] = pathStart[i];
			}
			*planlength = pathStart.size();
			return;	
        }

		i++;
    }

    // No path found
    *plan = NULL;
    *planlength = 0;
    // printf("No path found!\n");
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	// Initialize tree with the start configuration
    Node* startNode = new Node{armstart_anglesV_rad, nullptr};
    vector<Node*> tree = {startNode};

    // Set a flag
    int flag = 0;

    // Set a step size for extending the tree
    double stepSize = 0.25; 

    // Define the maximum number of iterations (you may adjust this based on your needs)
    int maxIterations = 5000;

    while(flag == 0 && tree.size() < maxIterations) {
        // Generate a random sample
        // printf("Iteration %ld\n", tree.size());
        double* randomSample = getRandomConfiguration_Biased(numofDOFs, armgoal_anglesV_rad);
		// double* randomSample = getRandomConfiguration(numofDOFs);
        Node* newNode = extendTree(randomSample, tree, stepSize, numofDOFs);

        if(IsValidArmConfiguration(newNode->angles, numofDOFs, map, x_size, y_size)) 
        {   
            // Print the angles of the current nodes

            // printf("Valid configuration!| Tree: %ld | Angles : [%f, %f, %f, %f, %f]\n", tree.size(), newNode->angles[0], newNode->angles[1], newNode->angles[2], newNode->angles[3], newNode->angles[4]);
            tree.push_back(newNode);

            // Rewire the tree
            for (Node* nearNode : findNearNodes(tree, newNode, stepSize)) {
                double newCost = nearNode->cost + distance(nearNode->angles, newNode->angles);
                if (newCost < newNode->cost) {
                    newNode->parent = nearNode;
                    newNode->cost = newCost;
                }
            }

            if (equalDoubleArrays(newNode->angles, armgoal_anglesV_rad, numofDOFs)) {
                // Add the goal configuration to the tree
                flag = 1;
                Node* goalNode = new Node{armgoal_anglesV_rad, newNode};
                tree.push_back(goalNode);
                break;
            }
        }
    }

    // Find the path from the goal configuration to the start configuration
    if(flag == 1)
    {
		// printf("Path found!\n");
        vector<double*> path;
        Node* currentNode = tree.back();
        while (currentNode != nullptr) {
            path.push_back(currentNode->angles);
            currentNode = currentNode->parent;
        }
        reverse(path.begin(), path.end());
		// Shortcutting the path
		// vector<double*> shortCutPath = shortCut(path, map, x_size, y_size, numofDOFs);

        // Set the plan and plan length
        *plan = new double*[path.size()];
        for (int i = 0; i < path.size(); i++) {
            (*plan)[i] = path[i];
        }
        *planlength = path.size();
        
    }
    else
    {
		printf("Trash Tree -> Restarting!\n");
		plannerRRTStar(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
        // *plan = NULL;
        // *planlength = 0;
        // printf("No path found!\n");
    }
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	int numNodes = 5000; // Number of nodes to generate
    int k = 3; // Number of nearest neighbors to consider

	// Call the random sample generator (Used to generare the random samples for the report)
	// double** randomSample = generateValidSamples(4, armgoal_anglesV_rad, map, x_size, y_size);

    // Generate PRM roadmap
    vector<Node*> roadmap = generatePRMRoadmap(numNodes, k, map, x_size, y_size, numofDOFs);

    // Find start and goal nodes
    Node* startNode = nearestNeighbor(armstart_anglesV_rad, roadmap, numofDOFs);
    Node* goalNode = nearestNeighbor(armgoal_anglesV_rad, roadmap, numofDOFs);

    // Find path
    vector<Node*> path = findPathPRM(startNode, goalNode);

	// Create a node from the start and goal position and add them to the path
	Node* start = new Node{armstart_anglesV_rad, nullptr};
	Node* goal = new Node{armgoal_anglesV_rad, nullptr};
	path.insert(path.begin(), start);
	path.push_back(goal);

    if (!path.empty()) 
	{
		// printf("Path found!\n");
		// Convert the path to a vector of double*
		vector<double*> p;
		for (int i = 0; i < path.size(); i++) 
		{
            p.push_back(path[i]->angles);
        }
		
		*plan = new double*[p.size()];
        for (int i = 0; i < p.size(); i++) {
            (*plan)[i] = p[i];
        }
        *planlength = p.size();
    } 
	else 
	{
		printf("No path found!\n");
        // *plan = NULL;
        // *planlength = 0; 
    }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {	
		// printf("Calling plannerPRM ...\n");
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
		// printf("Calling plannerRRT ...\n");
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
		// printf("Calling plannerRRTConnect ...\n");
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
		// printf("Calling plannerRRTStar ...\n");
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}