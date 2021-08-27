#include <iostream>
#include <jsoncpp/json/json.h>
#include <queue>
#include <fstream>
#include <iomanip>
using namespace std;

struct Wall{
    int origin_dist = -100;
    int value = -100;
    int x = -100;
    int y = -100;
};

struct my_compare{
    bool operator()(Wall* &lhs, Wall* &rhs){
        if(rhs->origin_dist < 0){
            return false;
        }
        if(lhs->origin_dist <0 and rhs->origin_dist >0){
            return true;
        }
        return lhs->origin_dist > rhs->origin_dist;
    }
};

class Dist{
    public:
        int rb=-1;
        int dist=-1;
        int u = -1;
        int v = -1; /* gy1-y */
        vector<int> Path;
        vector<pair<short,short>> pxy;
};

struct dist_compare{
    bool operator()(Dist* &lhs, Dist* &rhs){
        return lhs->dist > rhs->dist;
    }
};

class Feasible{
    private:
        vector<int> start_x;
        vector<int> start_y;
        vector<int> target_x;
        vector<int> target_y;
        vector<int> obstacle_x;
        vector<int> obstacle_y;

        int shape;
        int Layer;
        vector<pair<short,short>> U;
        vector<bool> F;
        int grid_x;
        int grid_y;
        int gy1;

        int num_rb;
        int num_ob;
        char heuristic;
        int bfs_count=0;

        vector<Dist*> dg;
        vector<vector<pair<short,short>>> Dist_Grid;
        vector<vector<vector<char>>> Grid3D;  // ANTOINE
        vector<vector<vector<short>>> Grid3D2;

        vector<int> move_order;
        vector<int> move_order1;

        vector<int> AllManhattan;
        int Achieved_sum=0;
        int Achieved_ms=0;
    public:
        bool use_Grid3D2;
        bool minimize_makespan;
        Feasible(){};
        ~Feasible(){};

        string input;
        string output;
        string Objective;
        string starttime;
        string elapse_t;

        void ReadData();
        void UF();
        void UF2();
        void FindClosestU();
        /* change the search area: grid_x, grid_y */
        void GridInitialize();
        void MoveOrder();

        void MoveOneRobot(int,int,int,int,int,int); // ANTOINE
        void MoveOneRobot1(int,int,int,int,int,int); // ANTOINE
        void Simultaneous();    // ANTOINE
        void Simultaneous1();    // ANTOINE

        void BFSdist(vector<vector<pair<short,short>>>&, int, int);

        void WriteFile();
        void WriteVisual();
        void WriteScore();

        int Value(vector<vector<pair<short,short>>>& G,int x, int y){
            /* Manhattan distance */
            return G[y][x].first+G[y][x].second;
        }
        int First(vector<vector<pair<short,short>>>& G,int x, int y){
            /* Distance between x-coordi*/
            return G[y][x].first;
        }
        int Second(vector<vector<pair<short,short>>>& G,int x, int y){
            /* Distance between y-coordi*/
            return G[y][x].second;
        }
        void Set(vector<vector<pair<short,short>>>& G,int x, int y, int a, int b){
            G[y][x] = make_pair(a,b);
        }

        void PrintGrid(vector<vector<pair<short,short>>>&);
        void PrintGrid3D();
        void PrintGrid3D2();
        void PrintGridWall();
        void PrintMoveOrder();
        void PrintMoveOrder1();
};

int Move_x(int x, char d);     // ANTOINE
int Move_y(int y, char d);
int Unmove_x(int x,char d);
int Unmove_y(int y, char d);




