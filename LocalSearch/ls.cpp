#include "ls.h"


int Move_x(int x, char d){      // ANTOINE: moves (x,y) in direction d, returns x-coordinate
    if (d==1) return x+1;        //             Now direction 5 (instead of 0) means: do not move
    if (d==2) return x-1;
    return x;
}
int Move_y(int y, char d){      // ANTOINE: moves (x,y) in direction d, returns y-coordinate
    if (d==3) return y-1;
    if (d==4) return y+1;
    return y;
}
int Unmove_x(int x, char d){      // ANTOINE: moves (x,y) in direction opposite to d, returns x-coordinate
    if (d==1) return x-1;
    if (d==2) return x+1;
    return x;
}
int Unmove_y(int y, char d){      // ANTOINE: moves (x,y) in direction opposite to d, returns y-coordinate
    if (d==3) return y+1;
    if (d==4) return y-1;
    return y;
}
void UNISTCG21_LS::MoveOneRobot(int r){     // ANTOINE: minimizes completion time of robot r
    int sx=start_x[r];
    int sy=start_y[r];
    int tx=target_x[r];
    int ty=target_y[r];
    RemoveRobot(r);

    Grid3D[0][sx][sy]=15;                              //10+d means current robot has previous move d
    for(int t=0;t<grid_t-1;t++){
        int t2=t+1;
        for(int x=1;x<grid_x-1;x++){         // handles current robot at x,y does not move
            for(int y=1; y<grid_y-1; y++){
                char g=Grid3D[t][x][y];
                if(g<=5)
                    continue;
                char g2=Grid3D[t2][x][y];
                if(g2==0)
                    Grid3D[t2][x][y]=15;
            }
        }
        for(int x=1;x<grid_x-1;x++){          // handles current robot at x,y   moves
            for(int y=1; y<grid_y-1; y++){
                char g=Grid3D[t][x][y];
                if(g<=5){
                    continue;
                }
                for(char d=1;d<5;d++){
                    int x2=Move_x(x,d);
                    int y2=Move_y(y,d);
                    char g2=Grid3D[t2][x2][y2];
                    if(g2!=0)                                          // next cell not free
                        continue;
                    char g3=Grid3D[t][x2][y2];
                    if((g3>=1) && (g3<=5) && (d!=g3) )                        // swarm condition 1
                        continue;
                    char g4=Grid3D[t2][x][y];
                    if(g4>=1 && g4<=5){                                                   // swarm condition 2
                        int x3=Unmove_x(x,d);
                        int y3=Unmove_y(y,d);
                        if (Grid3D[t][x3][y3]!=d)
                            continue;
                    }
                    Grid3D[t2][x2][y2]=10+d;
                }
            }
        }
    }
    if(Grid3D[grid_t-1][tx][ty]<11){
        cout << "***** ERROR MoveOneRobot 1" << endl;
        cout << "robot=" << r << " g=" << int(Grid3D[grid_t-1][tx][ty]);
        cout << " sx= " << sx << " sy=" << sy  << " tx= " << tx << " ty=" << ty << endl;
        cout << "grid_t=" << grid_t << " completion time=" << completion_time[r] << endl;
        PrintGrid3D();
        exit(0);
    }
    number_of_moves[r]=0;
    char d1,d2;
    for(int t=grid_t-1; t>=0; t--){               // write path for robot r in Grid3D
        d1=Grid3D[t][tx][ty]-10;
        if (t==grid_t-1){
            Grid3D[t][tx][ty]=5;
        }
        else{
            Grid3D[t][tx][ty]=d2;
        }
        tx=Unmove_x(tx,d1);
        ty=Unmove_y(ty,d1);
        if (d1!=5)
            number_of_moves[r]++;
        d2=d1;
    }
    if(sx!=tx || sy != ty){
        cout << "***** ERROR MoveOneRobot 2" << endl;
        exit(0);
    }
    for (int t=0;t<grid_t;t++){                     // clean the grid
        for(int x=1;x<grid_x-1;x++){
            for(int y=1;y<grid_y-1;y++){
                if(Grid3D[t][x][y]>=10){
                    Grid3D[t][x][y]=0;
                }
            }
        }
    }
}

void UNISTCG21_LS::MoveOneRobot2(int r){     // ANTOINE: minimizes completion time of robot r
	int sx=start_x[r];
	int sy=start_y[r];
	int tx=target_x[r];
	int ty=target_y[r];
	RemoveRobot(r);

    Grid3D[0][sx][sy]=15;                              //10+d means current robot has previous move d
    Grid3D2[0][sx][sy]=0;
    for(int t=0;t<grid_t-1;t++){
        int t2=t+1;
        for(int x=1;x<grid_x-1;x++){         // handles current robot at x,y does not move
            for(int y=1; y<grid_y-1; y++){
                char g=Grid3D[t][x][y];
                if(g<=5)
                    continue;
                char g2=Grid3D[t2][x][y];
                if(g2==0){
                    Grid3D[t2][x][y]=15;
                    Grid3D2[t2][x][y]=Grid3D2[t][x][y];
                }
            }
        }
        for(int x=1;x<grid_x-1;x++){          // handles current robot at x,y   moves
            for(int y=1; y<grid_y-1; y++){
                char g=Grid3D[t][x][y];
                if(g<=5){
                    continue;
                }
                for(char d=1;d<5;d++){
                    int x2=Move_x(x,d);
                    int y2=Move_y(y,d);
                    char g2=Grid3D[t2][x2][y2];
                    if(g2<0 or 1<=g2 and g2<=5) //next cell not free
                        continue;
                    else{
                        char g3=Grid3D[t][x2][y2];
                        if((g3>=1) && (g3<=5) && (d!=g3) )                        // swarm condition 1
                            continue;
                        char g4=Grid3D[t2][x][y];
                        if(g4>=1 && g4<=5){                                                   // swarm condition 2
                           int x3=Unmove_x(x,d);
                           int y3=Unmove_y(y,d);
                           if (Grid3D[t][x3][y3]!=d)
                                continue;
                        }
                        if(g2>10){
                            if(Grid3D2[t2][x2][y2] > Grid3D2[t][x][y]+1){
                                Grid3D[t2][x2][y2]=10+d;
                                Grid3D2[t2][x2][y2]=Grid3D2[t][x][y]+1;
                            }
                        }
                        else if(g2==0){
                            if(Grid3D2[t2][x2][y2]==-1){
                                Grid3D[t2][x2][y2]=10+d;
                                Grid3D2[t2][x2][y2]=Grid3D2[t][x][y]+1;
                            }
                        }
                    }
                }
            }
        }
    }
    if(Grid3D[grid_t-1][tx][ty]<11){
        cout << "***** ERROR MoveOneRobot2 1" << endl;
        cout << "robot=" << r << " g=" << int(Grid3D[grid_t-1][tx][ty]);
        cout << " sx= " << sx << " sy=" << sy  << " tx= " << tx << " ty=" << ty << endl;
        cout << "grid_t=" << grid_t << " completion time=" << completion_time[r] << endl;
        PrintGrid3D();
        exit(0);
    }
    number_of_moves[r]=0;
    char d1,d2;
    for(int t=grid_t-1; t>=0; t--){               // write path for robot r in Grid3D
        d1=Grid3D[t][tx][ty]-10;
        if (t==grid_t-1){
            Grid3D[t][tx][ty]=5;
        }
        else{
            Grid3D[t][tx][ty]=d2;
        }
        tx=Unmove_x(tx,d1);
        ty=Unmove_y(ty,d1);
        if (d1!=5 and d1!=0)
            number_of_moves[r]++;
        d2=d1;
    }
    if(sx!=tx || sy != ty){
        cout << "***** ERROR MoveOneRobot2 2" << endl;
        exit(0);
    }
    for (int t=0;t<grid_t;t++){                     // clean the grid
        for(int x=1;x<grid_x-1;x++){
            for(int y=1;y<grid_y-1;y++){
                if(Grid3D[t][x][y]>=10){
                    Grid3D[t][x][y]=0;
                }
                Grid3D2[t][x][y]=-1;
            }
        }
    }
}

int UNISTCG21_LS::ComputeCompletionTime(int r){			// Returns the completion time of robot r
	int t=grid_t-1;
	while(Grid3D[t][target_x[r]][target_y[r]]==5 && t>0)
		t--;
	completion_time[r]=t+1;
	return t;
}

void UNISTCG21_LS::RemoveRobot(int r){			// Removes trajectory of robot r from Grid3D
   int x=start_x[r];
   int y=start_y[r];
   for(int t=0; t<grid_t; t++){
        char d;
        d=Grid3D[t][x][y];
        if(Grid3D[t][x][y]<1 || Grid3D[t][x][y]>5){
        	cout << "***** ERROR RemoveRobot" << endl;
        	exit(0);
        }
        Grid3D[t][x][y]=0;
        x=Move_x(x,d);
        y=Move_y(y,d);
    }
}

void UNISTCG21_LS::CopyBest(){
    Grid3Dbest.clear();
    Grid3Dbest.assign(Grid3D.begin(),Grid3D.end());
    makespan_best= makespan;
    total_moves_best=total_moves;
}

void UNISTCG21_LS::Optimize(){
    cout << "----------------------- Optimize -------------------------" << endl;
    completion_time.resize(num_rb);

    vector<int> permutation(num_rb,0);
    for(int r=0; r< num_rb; r++)
        permutation[r]=r;

    Grid3D0=Grid3D;
    clock_t t0=clock();
    clock_t tB=clock();
    bool first_output=true;
    bool ever_update=false;
    bool not_written=false;

    bool improvement = true;
    while(true){
        for (int r=0; r<num_rb; r++)
            ComputeCompletionTime(r);
        bool flag = true;
        while(flag){
            flag = false;
            random_shuffle(permutation.begin(),permutation.end());
            for(int i=0; i<num_rb; i++){
                int r=permutation[i];
                int temp=completion_time[r];
                int temp2=number_of_moves[r];
                if(!Objective)
                    MoveOneRobot(r);
                else
                    MoveOneRobot2(r);

                ComputeCompletionTime(r);
                if(!Objective and completion_time[r] < temp)
                    flag = true;
                else if(Objective and number_of_moves[r] < temp2)
                    flag = true;

                grid_t=1+(*max_element(completion_time.begin(),completion_time.end()));
                achieved_moves=accumulate(number_of_moves.begin(),number_of_moves.end(),0);

                if((!Objective and grid_t<makespan) or (Objective and achieved_moves <total_moves)){
                    makespan=grid_t;
                    total_moves=achieved_moves;
                    ever_update=true;
                    cout << "!updates! " << elapse_t <<" s";
                    cout << " makespan= " << makespan << " sum= " << total_moves << endl;

                    CopyBest();
                    if(first_output){
                        WriteFile();
                        tB=clock();
                        first_output=false;
                    }
                    else
                    	not_written=true;   // ANTOINE
              	}
              	if(not_written and (float)(clock()-tB)/CLOCKS_PER_SEC > OutputTimeLapse){
                	WriteFile();
                    tB=clock();
                    not_written=false;  // ANTOINE
                }
                elapse_t=(float)(clock()-t0)/CLOCKS_PER_SEC;
                if(elapse_t>=RunningTime){
                    cout <<"!----------------------------------------------------\n";
                    cout<<"Exit by User after Running time "<< RunningTime <<" (sec)\n";
                    if(ever_update) WriteFile();
                    else cout<<"Nothing updated\n";
                    exit(0);
                }
            } // End of for loop
        } // End of inner while loop
        // No improvements, so start from beginning
        cout <<"------- start from beginning --------" <<  elapse_t <<" s" << " makespan= " << makespan << " sum= " << total_moves << endl;
        grid_t=Grid3D0.size();
        Grid3D.clear();
        Grid3D.assign(Grid3D0.begin(),Grid3D0.end());
        number_of_moves.clear();
        number_of_moves.assign(number_of_moves0.begin(),number_of_moves0.end());
    } // End of outer while loop
}

int UNISTCG21_LS::CheckGrid3D(){					// checks whether the data in Grid3D is valid.
	int count=0;					// returns number of robots.
	for (int t=0; t<grid_t; t++){	// checks number of robots in each slice
		int count_temp=0;
		for (int x=0; x<grid_x; x++)
			for (int  y=0; y< grid_y; y++){
				char d=Grid3D[t][x][y];
				if ( d>=1 && d<=5 ){
					count_temp++;
				}
				if ( (d<-1) || (d>5) ){
					cout << "************** ERROR CheckGrid3D  1" << endl;
					exit(0);
				}
			}
		if (t>0 && count != count_temp){
			cout << "************** ERROR CheckGrid3D  2" << endl;
			exit(0);
		}
		count=count_temp;
	}
	bool flag=true;
	for (int r=0; r<num_rb; r++){	// checks starting end target points
		int sx=start_x[r];			// may need to implement swarm condition 2
		int sy=start_y[r];
		int sg=Grid3D[0][sx][sy];
		int tx=target_x[r];
		int ty=target_y[r];
		int tg=Grid3D[grid_t-1][tx][ty];

		if( (sg==0 && tg !=0) || (sg!=0 && tg==0) ){
			cout << "************** ERROR CheckGrid3D  3" << endl;
			cout << "robot " << r << " sx=" << sx << " sy=" << sy << " sg=" << int(sg);
			cout << " tx=" << tx << " ty=" << ty << " tg=" << int(sg) << endl;
			exit(0);
		}
		if ( sg==0 && flag==false){
			cout << "************** ERROR CheckGrid3D  4" << endl;
			exit(0);
		}
		if ( sg==0 && flag==true){
			flag=false;
		}
		else{
			if (sg<1 || sg>5 || tg!=5){
				cout << "************** ERROR CheckGrid3D  5" << endl;
				exit(0);
			}
		}
	}
	for (int t=0; t< grid_t-1; t++)				// checks trajectory
		for (int x=0; x<grid_x; x++)
			for (int y=0; y<grid_y; y++){
				int d=Grid3D[t][x][y];
				if (d==0 || d==-1)
					continue;
				int x2=Move_x(x,d);
				int y2=Move_y(y,d);
				int t2=t+1;
				int d2=Grid3D[t2][x2][y2];
				if (d2<1 ){
					cout << "************** ERROR CheckGrid3D  6" << endl;
					exit(0);
				}
				if (d==5)
					continue;
				int d3=Grid3D[t][x2][y2];
				if (d3!=0 && d3!=d){								// swarm condition 1
					cout << "************** ERROR CheckGrid3D  7" << endl;
					cout << "d=" << d << " d3=" << d3 << " x=" << x << " y=" << y << " t=" << t ;
					cout << " x2=" << x2 << " y2=" << y2 << " d2=" << d2 << " d3=" << d3 << endl;
					PrintGrid3D();
					exit(0);
				}
			}
	return count;
}

void UNISTCG21_LS::ReadData(){
    cout << "----------------------- ReadData -------------------------" << endl;
    short tx = 2;
    short ty = 2;
    int gx = 3;
    int gy = 3;
    Json::Value root;
    Json::Reader reader;

    string path=input;
    path = input;
    ifstream json(path,ifstream::binary);
    reader.parse(json, root);

    Json::Value meta = root["meta"];
    num_rb = meta["number_of_robots"].asInt();

    clock_t clt = (int)clock();
    start_x.assign(num_rb,0);
    start_y.assign(num_rb,0);
    int count=0;
    for(auto& value:root["starts"]){
        start_x.at(count)=value[0].asInt();
        start_y.at(count)=value[1].asInt();
        count++;
    }
    target_x.assign(num_rb,0);
    target_y.assign(num_rb,0);
    count=0;
    for(auto& value:root["targets"]){
        target_x.at(count)=value[0].asInt();
        target_y.at(count)=value[1].asInt();
        count++;
    }
    count=0;
    for(auto& value:root["obstacles"]){
        count++;
    }

    num_ob = count;
    count=0;
    if(num_ob !=0){
        obstacle_x.assign(num_ob,0);
        obstacle_y.assign(num_ob,0);
        for(auto& value:root["obstacles"]){
            obstacle_x.at(count)=value[0].asInt();
            obstacle_y.at(count)=value[1].asInt();
            count++;
        }
    }
    printf("reading data takes %0.4f seconds\n",(float)(clock()-clt)/CLOCKS_PER_SEC);
}


void UNISTCG21_LS::WriteFile(){
    string filename;
    if(!Objective)
        filename = "MAX"+to_string(makespan_best)+".json";
    else
        filename = "SUM"+to_string(total_moves_best)+".json";

    ofstream solution(filename);

    string instance_name;
	instance_name = input.substr(0, input.size() - 14);

    solution<<"{\n";
    solution<<"\"instance\": \""<< instance_name <<"\",\n";
    solution<<"\"steps\" :[\n";

    vector<int> rx(num_rb,0);
    vector<int> ry(num_rb,0);
    for(int i=0; i<num_rb;i++){
        rx[i]=start_x[i];
        ry[i]=start_y[i];
    }

    char d;
    int final_move = 0;
    vector<int> final_m(num_rb,0);
    for(int t=0;t<makespan_best;t++){
        solution<<"{";
        int flag=0;
        for(int i=0;i<num_rb;i++){
            d=Grid3Dbest[t][rx[i]][ry[i]];
            rx[i]=Move_x(rx[i],d);
            ry[i]=Move_y(ry[i],d);
            if (d==5){
                continue;
            }
            if(d!=0){
                if (flag==1){
                    solution << ", ";
                }
                flag=1;
                solution<<"\""<<i<<"\":";
                if(d==1){
                    final_m[i]++;
                    final_move++;
                    solution<<"\"E\" ";
                }
                else if(d==2){
                    final_m[i]++;
                    final_move++;
                    solution<<"\"W\" ";
                }
                else if(d==3){
                    final_m[i]++;
                    final_move++;
                    solution<<"\"S\" ";
                }
                else if(d==4){
                    final_m[i]++;
                    final_move++;
                    solution<<"\"N\" ";
                }
            }
        }
        if (t<makespan_best-1){
            solution<<"},\n";
        }
        else{
            solution<<"}\n";
        }
    }

    solution<<"]\n";
    solution<<"}\n";
    solution.close();
    if( total_moves_best != final_move ){
        cout <<"total_moves_best = "<< total_moves_best <<" final_move = "<< final_move <<endl;
        cout << accumulate(final_m.begin(),final_m.end(),0)<<endl;
        cout << accumulate(number_of_moves.begin(),number_of_moves.end(),0)<<endl;
        exit(0);
    }
}

void UNISTCG21_LS::WriteVisual(){
    string filename;
    if(!Objective)
        filename = "visual_MAX"+to_string(makespan_best)+".js";
    else
        filename = "visual_SUM"+to_string(total_moves_best)+".js";

    ofstream result(filename);
    result << "var start = [\n";
    int i=0;
    for(;i<num_rb-1;i++){
        result << "{id:" << i+1 <<", x:";
        result << start_x[i] << ", y:"<< start_y[i]<<"},\n";
    }
    result << "{id:" << num_rb <<", x:";
    i=num_rb-1;
    result << start_x[num_rb-1] << ", y:"<< start_y[num_rb-1]<<"}\n];\n\n";
    result << "var end = [\n";
    for(int i=0;i<num_rb-1;i++){
        result << "{id:" << i+1 <<", x:";
        result << target_x[i] << ", y:"<< target_y[i]<<"},\n";
    }
    result << "{id:" << num_rb <<", x:";
    result << target_x[num_rb-1] << ", y:"<< target_y[num_rb-1]<<"}\n];\n\n";

    result << "var obstacle = [\n";
    if(num_ob!=0){
        for(int i=0;i<num_ob-1;i++){
            result << "{id: 'o', x:";
            result << obstacle_x[i] << ", y:"<< obstacle_y[i]<<"},\n";
        }
        result << "{id: 'o', x:";
        result << obstacle_x[num_ob-1] << ", y:"<< obstacle_y[num_ob-1]<<"}\n];\n\n";
    }
    else{
        result <<"];\n\n";
    }
    result << "var steps = [\n";

    Grid3D.clear();
    Grid3D.assign(Grid3Dbest.begin(),Grid3Dbest.end());
    vector<int> rx(num_rb,0);                                     //// ANTOINE: start
    vector<int> ry(num_rb,0);
    for(int r=0;r<num_rb;r++){
        rx[r]=start_x[r];
        ry[r]=start_y[r];
    }
    for(int t=0;t<makespan_best;t++){
        result << "[\n";
        for(int r=0;r<num_rb;r++){
            int a,b;
            a=rx[r]; b=ry[r];
            rx[r]=Move_x(a,Grid3D[t][a][b]);
            ry[r]=Move_y(b,Grid3D[t][a][b]);
            result <<"{id:"<<r+1<<", x: " << rx[r] << " ,y: " << ry[r] << "},\n";
        }
        result << "],\n";
    }
    result << "];\n";                                            /// ANTOINE: end


    result << "var distance =[]\n";
    result << "var data = {\n";
    result <<"\"start\" : start,\n";
    result <<"\"end\" : end,\n";
    result <<"\"obstacle\" : obstacle,\n";
    result <<"\"steps\" : steps,\n";
    result <<"\"distance\" : distance,\n";
    result <<"\"width\" :"<< grid_x <<",\n";
    result <<"\"height\" :"<< grid_y <<" \n};\n";
    result <<"$(window).on('load', function(){\n";
    result <<"if(!controller && $(\"#canvas\").length && $(\"#controls\").length){\n";
    result <<"  var controller = new RearrangingSquaresController";
    int width = (grid_x+1)*40;
    int height = (grid_y+4)*40;
    result <<"(data, \"canvas\", \"controls\","<< width <<"," << height<<");\n }\n});";
    result.close();
}


void UNISTCG21_LS::ReadSolution(){
    cout << "----------------------- ReadSolution -------------------------" << endl;
    clock_t s = (int) clock();
    Json::Value root;
    Json::Reader reader;

    ifstream json(solution,ifstream::binary);
    if (json.fail()){
        cout << "ReadSolution cannot read input file" << endl;
        exit(0);
    }
    reader.parse(json, root);
    Json::Value Steps = root["steps"];

    vector<int> robot_x(start_x);
    vector<int> robot_y(start_y);
    int xmin=*min_element(robot_x.begin(),robot_x.end());		// Compute xmin and xmax
    int xmax=*max_element(robot_x.begin(),robot_x.end());
    int ymin=*min_element(robot_y.begin(),robot_y.end());
    int ymax=*max_element(robot_y.begin(),robot_y.end());
    if (num_ob>0){
        xmin=min(xmin,*min_element(obstacle_x.begin(),obstacle_x.end()));
        xmax=max(xmax,*max_element(obstacle_x.begin(),obstacle_x.end()));
        ymin=min(ymin,*min_element(obstacle_y.begin(),obstacle_y.end()));
        ymax=max(ymax,*max_element(obstacle_y.begin(),obstacle_y.end()));
    }

    grid_t=1;
    vector<int> each_robot_move(num_rb,0);
    for(auto it = Steps.begin();it !=Steps.end();it++){
        grid_t++;
        for(auto jt = it->begin();jt != it->end();jt++){
            int r = stoi(jt.key().asString());
            int d;
            if(jt->asString() == "E"){
                d=1;
                each_robot_move.at(r)++;
            }
            else if(jt->asString() == "W"){
                d=2;
                each_robot_move.at(r)++;
            }
            else if(jt->asString() == "S"){
                d=3;
                each_robot_move.at(r)++;
            }
            else if(jt->asString() == "N"){
                d=4;
                each_robot_move.at(r)++;
            }
            else
                d=5;
            robot_x[r]=Move_x(robot_x[r],d);
            robot_y[r]=Move_y(robot_y[r],d);
            xmin=min(xmin,robot_x[r]);
            xmax=max(xmax,robot_x[r]);
            ymin=min(ymin,robot_y[r]);
            ymax=max(ymax,robot_y[r]);
        }
    }

    for(int temp:each_robot_move){
        total_moves+=temp;
    }
    makespan = grid_t-1;
    int tx=-xmin+1;            // Translate the grid
    int ty=-ymin+1;
    grid_x=tx+xmax+2;
    grid_y=ty+ymax+2;
    for(int i=0; i<num_ob;i++){
        obstacle_x[i]+=tx;
        obstacle_y[i]+=ty;
    }
    for(int r=0; r<num_rb; r++){
        start_x[r]+=tx;
        start_y[r]+=ty;
        target_x[r]+=tx;
        target_y[r]+=ty;
    }

    cout<<"Initial makespan= "<< makespan << " sum= "<< total_moves <<endl;
    //cout << "grid_x=" << grid_x << " grid_y = " << grid_y << " tx=" << tx << " ty=" << ty << " grid_t=" << grid_t-1 << " *******************" << endl;
    Grid3D.resize(grid_t);           	// Fill the 3D grid
    if(!Objective){
        for(int t=0; t<grid_t; t++){
            Grid3D[t].resize(grid_x);
            for(int x=0; x<grid_x; x++)
                Grid3D[t][x].resize(grid_y,0);
            for(int k=0;k<num_ob;k++)
                Grid3D[t][obstacle_x[k]][obstacle_y[k]]=-1;
            for(int x=0;x<grid_x;x++){
                Grid3D[t][x][0]=-1;
                Grid3D[t][x][grid_y-1]=-1;
            }
            for(int y=0;y<grid_y;y++){
                Grid3D[t][0][y]=-1;
                Grid3D[t][grid_x-1][y]=-1;
            }
        }
    }
    else{
        Grid3D2.resize(grid_t);
        for(int t=0; t<grid_t; t++){
            Grid3D[t].resize(grid_x);
            Grid3D2[t].resize(grid_x);
            for(int x=0; x<grid_x; x++){
                Grid3D[t][x].resize(grid_y,0);
                Grid3D2[t][x].resize(grid_y,-1);
            }
            for(int k=0;k<num_ob;k++)
                Grid3D[t][obstacle_x[k]][obstacle_y[k]]=-1;
            for(int x=0;x<grid_x;x++){
                Grid3D[t][x][0]=-1;
                Grid3D[t][x][grid_y-1]=-1;
                Grid3D2[t][x][0]=-2;
                Grid3D2[t][x][grid_y-1]=-2;
            }
            for(int y=0;y<grid_y;y++){
                Grid3D[t][0][y]=-1;
                Grid3D[t][grid_x-1][y]=-1;
                Grid3D2[t][0][y]=-2;
                Grid3D2[t][grid_x-1][y]=-2;
            }
        }
    }
    for(int r=0;r<num_rb;r++){
        Grid3D[grid_t-1][target_x[r]][target_y[r]]=5;        // d corresponds to robot with next move d in {1,2,3,4,5}
    robot_x[r]=start_x[r];
    robot_y[r]=start_y[r];
    }
    int t=0;
    number_of_moves.assign(num_rb,0);
    for(auto it = Steps.begin();it !=Steps.end();it++){
        for(int r=0; r<num_rb; r++)
            Grid3D[t][robot_x[r]][robot_y[r]]=5;
        for(auto jt = it->begin();jt != it->end();jt++){
            int r = stoi(jt.key().asString());
            int d;
            if(jt->asString() == "E")
                d=1;
            else if(jt->asString() == "W")
                d=2;
            else if(jt->asString() == "S")
                d=3;
            else if(jt->asString() == "N")
                d=4;
            else
                d=5;
            if(d!=5)
                number_of_moves[r]++;
            Grid3D[t][robot_x[r]][robot_y[r]]=d;
            robot_x[r]=Move_x(robot_x[r],d);
            robot_y[r]=Move_y(robot_y[r],d);
        }
        t++;
    }
    number_of_moves0.assign(number_of_moves.begin(),number_of_moves.end());
    root.clear();
    Steps.clear();
    Steps.resize(0);
}


void UNISTCG21_LS::WriteScore(){
    struct tm curr_tm;
    time_t curr_time = time(nullptr);
    localtime_r(&curr_time, &curr_tm);
    string scorefile;
    if(!Objective)
        scorefile = "score_MAX";
    else
        scorefile = "score_SUM";
    ofstream score(scorefile+".txt", ios::app);
    score <<" "<<curr_tm.tm_mday<<"/"<<curr_tm.tm_mon+1<<"_";
    score <<curr_tm.tm_hour<<":"<<curr_tm.tm_min<<":"<<curr_tm.tm_sec;
    score <<" "<< elapse_t <<" s";
    score <<" rb: "<< num_rb<<" ob: "<<num_ob;
    score <<" "<<total_moves_best;
    score <<" "<< makespan_best <<endl;
    score.close();
}

void UNISTCG21_LS::PrintGrid3D(){
    cout << "-----------------------------------------------------------------------------" << endl;
    cout << "number of slices = " << Grid3D.size() << endl;
    cout << "-----------------------------------------------------------------------------" << endl;
    for(int t=0; t< Grid3D.size(); t++){
        cout << "slice " << t << endl;
        for(int j=grid_y-1;j>=0;j--){
            for(int i=0;i<grid_x;i++){
                if(int(Grid3D[t][i][j]==0)){
                   cout <<" "<<setw(2)<<"-";
                }
                else if(int(Grid3D[t][i][j]==-1)){
                   cout <<" "<<setw(2)<<"X";
                }
                else{
                   cout <<" "<<setw(2) << int(Grid3D[t][i][j]);
                }
            }
            cout <<endl;
        }
        cout << "-----------------------------------------------------------------------------" << endl;
    }
}
