#include "ls.h"

int main(int argc, char** argv) {

    UNISTCG21_LS ls;
    srand (time(NULL));

    int i=1;
    if (argc<4 or argc>6){
		cout << "Syntax: ls [input_instance.json][input_solution.json][MAX or SUM][Running Time][Time between two output files]" << endl;
		exit(1);
    }
    cout << "input instance: " << argv[1] << endl;
    ls.input=argv[1];
    ls.ReadData();

    if (argv[3]==string("MAX")) {
    	ls.Objective=0;
    }
    else if(argv[3]==string("SUM")){
    	ls.Objective=1;
    }
    else{
    	cout << "the 3rd argument should be MAX or SUM" << endl;
    	exit(1);
    }
    cout << "Objective: " << ls.Objective << " " << argv[3] << endl;

    cout << "input solution: " << argv[2] << endl;
    ls.solution=argv[2];
    ls.ReadSolution();

	if (argc==4){
		ls.RunningTime=10000000;
		ls.OutputTimeLapse=60;
	} else if (argc==5){
	    ls.RunningTime = atoi(argv[4]);
		ls.OutputTimeLapse=60;
	} else if (argc==6){
		ls.RunningTime = atoi(argv[4]);
    	ls.OutputTimeLapse = atoi(argv[5]);
    }

    struct tm curr_tm;
    time_t curr_time = time(nullptr);
    localtime_r(&curr_time, &curr_tm);
    cout <<"\nStarts at "<<curr_tm.tm_mday<<"/"<<curr_tm.tm_mon+1<<"_";
    cout <<curr_tm.tm_hour<<":"<<curr_tm.tm_min<<":"<<curr_tm.tm_sec<<endl;
    cout <<"Time between two outputs= "<< ls.OutputTimeLapse <<endl;

   	ls.Optimize();
    return 0;
}
