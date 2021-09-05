#include "feasible.h"

int main(int argc, char** argv) {

    UNISTCG21_FS fs;
    srand (time(NULL));

    int i=1;
    if (argc<3 or argc>4){
		cout << "Syntax: fs [input_instance.json][MAX or SUM]" << endl;
		exit(1);
    }
    cout << "input instance: " << argv[1] << endl;
    fs.input=argv[1];

    if (argv[2]==string("MAX")) {
        fs.Objective=0;
    	fs.use_Grid3D2 = false;
    	fs.minimize_makespan = true;
    }
    else if(argv[2]==string("SUM")){
        fs.Objective=1;
    	fs.use_Grid3D2 = true;
	    fs.minimize_makespan = false;
    }
    else{
    	cout << "the 3rd argument should be MAX or SUM" << endl;
    	exit(1);
    }

    cout << "Objective: " << fs.Objective << " " << argv[2] << endl;

    fs.ReadData();

    struct tm curr_tm;
    time_t curr_time = time(nullptr);

    localtime_r(&curr_time, &curr_tm);
    fs.starttime = " "+to_string(curr_tm.tm_mday)+"/"+to_string(curr_tm.tm_mon+1)+"_";
    fs.starttime = fs.starttime+to_string(curr_tm.tm_hour)+":"+to_string(curr_tm.tm_min)+":"+to_string(curr_tm.tm_sec);
    clock_t s=(int) clock();

    fs.Simultaneous();

    s=(float)(clock()-s)/CLOCKS_PER_SEC;
    fs.elapse_t=to_string(s)+" s";
    cout<<"Running time "<< fs.elapse_t <<endl;
    fs.WriteFile();
    //fs.WriteVisual();
    return 0;
}
