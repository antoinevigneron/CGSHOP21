#include <iostream>
#include <jsoncpp/json/json.h>
#include <queue>
#include <fstream>
#include <iomanip>


//#include <stdio.h>
//#include <stdlib.h>
//#include <vector>
#include <algorithm>
//#include <cmath>
//#include <unordered_map>
//#include <map>
//#include <set>
//#include <ctime>
//#include <functional>
#include <numeric>
//#include <sys/types.h>
//#include <dirent.h>
//#include <cstring>
//#include <iterator>

using namespace std;

class LocalSearch{
    private:
        vector<int> start_x;
        vector<int> start_y;
        vector<int> target_x;
        vector<int> target_y;
        vector<int> obstacle_x;
        vector<int> obstacle_y;

        int grid_x;
        int grid_y;
        int grid_t;

        int num_rb;
        int num_ob;

        int shape; // Do we need it?

        vector<vector<vector<char>>> Grid3D;
        vector<vector<vector<char>>> Grid3D0;
        vector<vector<vector<char>>> Grid3Dbest;
        vector<vector<vector<short>>> Grid3D2;

        int achieved_moves=0;

        int makespan =0;
        int total_moves=0;

        int makespan_best=0;
        int total_moves_best=0;

      	vector<int> completion_time;
      	vector<int> number_of_moves;

        float elapse_t=0.0;

    public:
        LocalSearch(){};
        ~LocalSearch(){};

        string input;
        string output;
        string solution;
        int Objective;
        int RunningTime;
        int OutputTimeLapse;

        void ReadData();
        void ReadSolution();
        void MoveOneRobot(int);
        void MoveOneRobot2(int);
        void RemoveRobot(int);
        void Optimize();
        bool Movable(int &,int&, vector<vector<int>> &);
        int ComputeCompletionTime(int);
        int CheckGrid3D();
        void CopyBest();

        void WriteFile();
        void WriteVisual();
        void WriteScore();

        void PrintGrid3D();            // ANTOINE
};

int Move_x(int x, char d);     // ANTOINE
int Move_y(int y, char d);
int Unmove_x(int x,char d);
int Unmove_y(int y, char d);




