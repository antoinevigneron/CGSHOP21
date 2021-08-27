#include "sa.h"



int main(int argc, char** argv) {

    SA sa;
    //srand (time(NULL));
    
    
    int i=1;
    if (argc<4 or argc>5){
		cout << "Syntax: sa [input_instance.json][input_solution.json][MAX or SUM][parameter_file.par]" << endl;
		exit(1);
    }
    cout << "input instance: " << argv[1] << endl;
    sa.input=argv[1];
    sa.ReadData();

    if (argv[3]==string("MAX")) {
    	sa.Objective=0;
    }
    else if(argv[3]==string("SUM")){
    	sa.Objective=1;
    }
    else{
    	cout << "the 3rd argument should be MAX or SUM" << endl;
    	exit(1);
    }
    cout << "Objective: " << sa.Objective << " " << argv[3] << endl;

	string paramfile;
	if(argc==5){
		paramfile=argv[4];		
	}
	else if(sa.Objective==0){
		paramfile="defaultMAX.par";
	}	
	else if(sa.Objective==1){
		paramfile="defaultSUM.par";
	}
	cout << "parameter file: " << paramfile << endl;
	sa.UserParameter(paramfile);
	sa.PrintParameters();
	

    cout << "input solution: " << argv[2] << endl;
    sa.solution=argv[2];
    sa.ReadSolution();

    struct tm curr_tm;
    time_t curr_time = time(nullptr);
    localtime_r(&curr_time, &curr_tm);
    cout <<" "<<curr_tm.tm_mday<<"/"<<curr_tm.tm_mon+1<<"_";
    cout <<curr_tm.tm_hour<<":"<<curr_tm.tm_min<<":"<<curr_tm.tm_sec<<endl;

   	sa.Optimize();
    return 0;
}
