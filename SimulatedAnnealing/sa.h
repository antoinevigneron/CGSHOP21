#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <numeric>
#include <fstream>
#include <jsoncpp/json/json.h>

using namespace std;

class SA{
    private:
        //  INPUT PARAMETERS
        double Tmin;
        double Tmax;
        int niter;
        int ncycles;
        double grid_t_coef;
        int grid_xy_margin;
        double f_stretch;
        double f_tighten;
        int random_seed;
        double write_interval;
        // END OF INPUT PARAMETERS

	   
	   // number of moves that increased the score and
	   // were accepted in this cycle
        int number_rejected=0;  
        int number_accepted=0;


        vector<int> start_x;	// starting position of a robot
        vector<int> start_y;
        vector<int> target_x;   // target position of a robot
        vector<int> target_y;
        vector<int> obstacle_x;   // position of  an obstacle
        vector<int> obstacle_y;

        int grid_x;   // size of the 3D grid
        int grid_y;
        int grid_t;

        int num_rb;   // number of robots
        int num_ob;   //  number of obstacles

        int shape; // not used

        vector<vector<vector<char>>> Grid3D;      // main grid
        vector<vector<vector<char>>> Grid3Dbest;  // records best solution so far
        vector<vector<vector<short>>> Grid3D2;    // grid with distance information
        vector<vector<char>> interface;          // middle-slice when making a stretching move
        vector<vector<int>> grid;            // auxiliary grid, used for debugging and IOs
        vector<int> wxmin;     // current window coordinates when performing a stretch operation
        vector<int> wxmax;
        vector<int> wymin;
        vector<int> wymax;

        vector<char> prev_traj;
        int prev_traj_x,prev_traj_y;

        int makespan, total_moves, best_makespan, best_total_moves;    // scores
        double score, best_score;
        vector<int> completion_time;   // completion times of the robots
        vector<int> number_of_moves;   // numbers of moves of the robots

        float elapse_t=0.0;    // elapsed time

	public:
        SA(){};
        ~SA(){};

	    double alpha=5;     // parameter for the stretch operation size

        string input;    // input instance file name
        string output;   // output file name, no longer used
        string solution;  // input solution file name  
        int Objective;  // 0 for MAX, 1 for SUM

        void Optimize();  // main procedure that does simulated annealing
        void CopyBest();  // saves a copy of the best solution so far
        void UserParameter(string);   // loads the parameter file
        void PrintParameters();   
        void ReadData();    // reads the instance file
        void RemoveRobot(int,int,int,int,int);   // Removes a robot from the grid
        void UnRemoveRobot(int,int,int);    // Reinserts the robot
        int ComputeCompletionTime(int);     // Computes all the completion times
        int CheckGrid3D();  		// for debugging, checks consistency of data structure
        void ComputeScores(void);	// computes all the scores
        void RandomBounds(int &, int &);    // computes tmin and tmax for stretching
        double ComputeScore();       // computes the current value of the objective function
  
        void Tighten(int, int ,int);    // tightening operation
        void Stretch(int, int ,int);    // stretching operation
        void Position(int, int, int &, int &);  // position of a robot at a given time
        void FillGrid3D(int, int, int,int,int);   // main procedure for inserting a robot into the grid
        void FillGrid3D_back(int, int, int,int,int);  // same but backward 
        void FillGrid3D2(int, int, int,int,int);   //  inserts a robot while minimizing its number of moves
        void FillGrid3D_back2(int, int, int,int,int);  
        void PrintTrajectory(int);    //  for debugging, prints the trajectory of a robot
        void CleanGrid();
        void RandomMove(double);  // generates a random move for the simulated annealing

        void WriteScore();
        void WriteFile2();   // writes the output solution file
        void WriteVisual2();  // not currently used; for displaying the solution 
        void ReadSolution();  // reads the input solution file

        void PrintGrid3D();      // for debugging
        void PrintInterface();   // for debugging
};

int Move_x(int , char );     // moves in direction d
int Move_y(int , char );
int Unmove_x(int ,char );    // moves backwards from direction d
int Unmove_y(int , char );




