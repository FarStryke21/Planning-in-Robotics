import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

plannerList = ["RRT", "RRTCONNECT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    # problems = [["./map1.txt", "1.570796,0.785398,1.570796,0.785398,1.570796",
    #                             "0.392699,2.356194,3.141592,2.8274328,4.712388"],
    #         ["./map2.txt", "0.392699,2.356194,3.141592",
    #                             "1.570796,0.785398,1.570796"]]

    problems = [
                ["./map2.txt", "0.392699,2.356194,3.141592","1.570796,0.785398,1.570796"],
                ["./map2.txt", "1.466340,5.202395,1.304561,2.949626", "0.366624,3.276792,1.708619,1.312873"],
                ["./map2.txt", "0.770085,0.293235,2.938311,0.849313", "1.749751,5.371942,2.247162,2.483775"],
                ["./map2.txt", "1.600868,2.103580,0.893306,5.165508", "1.563135,2.679784,0.827199,6.021880"],
                ["./map2.txt", "0.896909,2.103018,2.219823,1.667896", "0.337269,2.607728,2.000279,5.313906"],
                ["./map2.txt", "1.165005,1.467201,3.638635,4.938026", "1.291006,2.507072,5.498993,5.428735"],
                ["./map2.txt", "1.825788,5.225387,1.464571,0.575427", "1.260163,1.933199,1.376783,3.970613"],
                ["./map2.txt", "1.683710,1.705942,5.741788,2.514218", "0.094765,6.254705,2.780871,0.380473"], 
                ["./map2.txt", "1.557288,2.126027,1.110754,6.129017", "0.685105,5.839448,1.119969,3.625207"],
                ["./map2.txt", "0.503520,5.880797,2.563045,3.419112", "0.366601,1.742353,1.350295,3.749208"],
                ["./map2.txt", "1.155461,2.360845,5.352844,0.941554", "0.809280,3.366588,0.277391,1.896677"],
                ["./map2.txt", "1.257181,0.923621,2.050745,1.559712", "0.632250,2.560300,3.313542,3.573848"],
                ["./map2.txt", "1.571282,1.558336,3.879547,3.279791", "1.646669,1.233930,5.431218,5.785156"],
                ["./map2.txt", "0.919034,1.080997,6.125651,4.493142", "0.175985,1.834523,5.085469,3.165996"],
                ["./map2.txt", "1.036066,1.488801,2.827454,3.037292", "0.104368,1.580905,3.448028,0.109133"],
                ["./map2.txt", "0.674531,2.709350,0.787241,2.850385", "1.453937,5.730944,3.121371,6.134127"],
                ["./map2.txt", "1.619552,0.746130,2.460286,3.225942", "0.478839,2.188059,5.235496,0.305292"],
                ["./map2.txt", "0.722020,2.213984,1.705407,0.525902", "0.263619,3.321129,1.029257,0.837670"],
                ["./map2.txt", "1.575816,2.218053,0.265569,4.291518", "0.695574,1.428588,1.780402,5.629848"],
                ["./map2.txt", "1.507219,1.962573,0.799230,4.876514", "0.649638,5.841130,0.970132,4.122333"]    
                ]

    scores = []
    for aPlanner in [0, 1, 2, 3]:
        print("\nTESTING " + plannerList[aPlanner] + "\n")
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            print("EXECUTING: " + str(commandPlan))
            commandVerify = "./verifier.out {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            print("EXECUTING: " + str(commandVerify))
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                ### Visualize their results
                commandViz = "python3 visualizer.py grader_out/tmp.txt --gifFilepath=grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                # commandViz += " --incPrev=1"
                subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./planner.out", "grader_out/grader_results.csv")