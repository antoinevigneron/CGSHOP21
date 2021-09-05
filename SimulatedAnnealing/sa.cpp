#include "sa.h"

void UNISTCG21_SA::CopyBest(){
    Grid3Dbest.clear();
    Grid3Dbest.assign(Grid3D.begin(),Grid3D.end());
    best_makespan=makespan;
    best_total_moves=total_moves;
    best_score=score;
}


void UNISTCG21_SA::UserParameter(string filename){
    ifstream para(filename, ifstream::binary);
    if (para.fail()){
        cout << "cannot read parameter file" << endl;
        exit(0);
    }
    string prm_str;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> Tmin;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> Tmax;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> niter;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> ncycles;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> f_stretch;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> f_tighten;
    getline(para, prm_str);
	getline(para, prm_str);
    stringstream(prm_str) >> grid_t_coef;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> grid_xy_margin;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> random_seed;
    getline(para, prm_str);
    getline(para, prm_str);
    stringstream(prm_str) >> write_interval;
}


void UNISTCG21_SA::PrintParameters(){
    cout << "----------------------- Parameter ------------------------" << endl;
    cout << "| Tmin=" <<  Tmin <<endl;
    cout << "| Tmax=" << Tmax <<endl;
    cout << "| niter=" << niter <<endl;
    cout << "| ncycles=" << ncycles<<endl;
    cout << "| f_stretch=" << f_stretch <<endl;
    cout << "| f_tighten=" << f_tighten <<endl;
    cout << "| ___________________________\n";
    cout << "| grid_x=" << grid_x <<endl;
    cout << "| grid_y=" << grid_y <<endl;
    cout << "| grid_t=" << grid_t<<endl;
    cout << "| grid_t_coef=" << grid_t_coef <<endl;
    cout << "| grid_xy_margin=" << grid_xy_margin << endl;
    cout << "| ___________________________\n";
    cout << "| Objective=" << Objective << endl;
    cout << "| alpha=" << alpha << endl;
    cout << "| random seed=" << random_seed << endl;
    cout << "| write_interval=" << write_interval << endl;
    cout << "| ___________________________\n";
}

void UNISTCG21_SA::Optimize(){
	CopyBest();
	srand(random_seed);
    cout << "----------------------- Optimize -------------------------" << endl;
    best_score=ComputeScore();
    best_makespan=makespan;
    best_total_moves=total_moves;
    double sigma = (double)niter / (20 - log(Tmax - Tmin));
    double Tmax0=Tmax;
    clock_t tpr=clock();
    clock_t t0=clock();
    int enlarge=1;
    elapse_t=0.0;
    bool ever_update=false;
    for (int cycle = 1; cycle <= ncycles; cycle++) {
        number_accepted=0;
        number_rejected=0;
        for (int index = 1; index <= niter; index++) {
            double i = index - 1;
            double T = (Tmax - Tmin) * exp(-i / sigma) + Tmin;
            // CheckGrid3D();
            RandomMove(T);
            elapse_t=(float)(clock()-t0)/CLOCKS_PER_SEC;
            if (score<best_score){
                CopyBest();
                ever_update=true;
                cout << "!updates! "<< elapse_t << " s";
                cout << " makespan: " << best_makespan << " sum: " << best_total_moves;
                cout << " #iter: "<< index << " #cycle: " << cycle;
                cout << " score: "<< ComputeScore();
                if (Objective==0 and grid_t>1+int(grid_t_coef*double(best_makespan))){
                    grid_t=1+int(grid_t_coef*double(makespan));
                    cout << " resized grid_t to " << grid_t << endl;
                }
                else cout<<endl;
            }
            if (ever_update and ((double)(clock()-tpr)/CLOCKS_PER_SEC>write_interval)){
            	WriteFile();
            	tpr=clock();
            	ever_update=false;
            }
     	}
        cout << (float)(clock()-t0)/CLOCKS_PER_SEC << " s ";
        cout << "Cycle " << cycle << " Tmin=" << Tmin << " Tmax=" << Tmax;
        cout << " #rejected=" << number_rejected << " #accepted=" << number_accepted;
        cout << " makespan=(" << makespan << "/"  << best_makespan<<")"<< " sum=(" <<total_moves<<"/"<<best_total_moves<<")";
        cout << " score= " << score << endl;
        Tmax -= (Tmax0 - Tmin) / (double)ncycles;
    }
    if(ever_update){
        WriteFile();
        //WriteVisual();
    }
}

double UNISTCG21_SA::ComputeScore(){
    if (Objective==1){
        return score=double(total_moves)/double(num_rb);
    }
    //  return score=double(makespan);
    int count=-1;
    for (int r=0; r< num_rb; r++)
    	if (completion_time[r]==makespan)
    		count++;
    if (count<0 or count>= num_rb){
    	cout << "error ComputeScore()" << endl;
    	exit(1);
    }
    score=double(makespan)+(double)count/double(num_rb);
    // cout << "makespan: " << makespan << " score: " << setprecision(8) << score << " count:  "  << count <<  endl;
    return score;
}

void UNISTCG21_SA::RandomMove(double T){
    static bool firsttime=true;
    static double r1;
    if(firsttime==true){
        double tot= f_stretch+f_tighten;
        r1=f_stretch/tot;
        cout << "r1= "<< r1<<endl;
        firsttime=false;
    }

    int r=rand()%num_rb;
    int bakm=makespan;
    int bakt=total_moves;
    int bakc=completion_time[r];
    int bakn=number_of_moves[r];
    double baks=ComputeScore();
    int tmin,tmax;
    double rndc=(double)rand() / ((double)RAND_MAX + 1);
    if(rndc<=r1){
        RandomBounds(tmin,tmax);
        Stretch(r,tmin,tmax);
    }
    else{
        tmin=0; tmax=grid_t-1;
        Tighten(r,tmin,tmax);
    }
    if (ComputeScore()<=baks){
        return;
    }
    double rand01=(double)rand() / ((double)RAND_MAX + 1);
    double acceptance_prob=exp((baks-score)/T);
    if (rand01<=acceptance_prob){
        number_accepted++;
        return;
    }
    number_rejected++;
    UnRemoveRobot(r,tmin,tmax);
    makespan=bakm; total_moves=bakt; completion_time[r]=bakc; number_of_moves[r]=bakn;
}

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

void UNISTCG21_SA::Position(int r, int t, int &x, int &y){  // returns the position (x,y) of robot r at time t
    if(r >= num_rb){
        cout <<"-----------------\n";
        cout <<"r = "<< r <<", num_rb = "<< num_rb <<endl;
        exit(0);
    }
    if(t>grid_t){
        cout <<"-----------------\n";
        cout <<"t = "<< t <<", grid_t = "<< grid_t <<endl;
        exit(0);
    }

    x=start_x[r];
    y=start_y[r];
    int a;// hyeyun : remove ambiguity
    int b;// hyeyun : remove ambiguity
    for(int i=0;i<t;i++){
        //int a=x;// hyeyun : remove ambiguity
        //int b=y;// hyeyun : remove ambiguity
        a=x;
        b=y;
        if(a>=grid_x or a<0){
            PrintGrid3D();
            cout <<"-----------------\n";
            cout <<"a = "<< a <<", grid_x = "<< grid_x <<endl;
            exit(0);
        }
        if(b>=grid_y or b<0){
            PrintGrid3D();
            cout <<"-----------------\n";
            cout <<"b = "<< b <<", grid_y = "<< grid_y <<endl;
            exit(0);
        }
        x=Move_x(a,Grid3D[i][a][b]);
        y=Move_y(b,Grid3D[i][a][b]);
    }
}

void UNISTCG21_SA::RandomBounds(int &tmin, int &tmax){
    int k=grid_t;
    double x = ((double)rand()+1) / ((double)RAND_MAX + 1);
    k=(int)(alpha*sqrt(1/x));
    k=min(grid_t-1,k);
    tmin=rand()%(grid_t-k);
    tmax=tmin+k;
    //delta_counting[k]++;//hyeyun
}



void UNISTCG21_SA::FillGrid3D(int r, int tmin, int tmax,int sx, int sy){ // Fills grid3D with values 11,...,15
    //cout <<" FG3D,";
    Grid3D[tmin][sx][sy]=15;
    wxmin[tmin]=sx;	// Coordinates of the current window when traversing the grid
    wxmax[tmin]=sx;
    wymin[tmin]=sy;
    wymax[tmin]=sy;
    int t=tmin;
    while(t!=tmax){
        int t2=t+1;
        int xmin=wxmin[t];	int xmax=wxmax[t];
        int ymin=wymin[t];	int ymax=wymax[t];
        int xmin2=xmin; 	int xmax2=xmax;
        int ymin2=ymin; 	int ymax2=ymax;
        for(int x=xmin;x<=xmax;x++){         // handles current robot at x,y does not move
            for(int y=ymin; y<=ymax; y++){
                char g=Grid3D[t][x][y];
                if(g<=5)
                    continue;
                char g2=Grid3D[t2][x][y];
                if(g2==0)
                    Grid3D[t2][x][y]=15;
            }
        }
        for(int x=xmin;x<=xmax;x++){          // handles current robot at x,y   moves
            for(int y=ymin; y<=ymax; y++){
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
                    xmin2=min(xmin2,x2); xmax2=max(xmax2,x2); ymin2=min(ymin2,y2); ymax2=max(ymax2,y2);
                }
            }
        }
        wxmin[t2]=xmin2; wxmax[t2]=xmax2; wymin[t2]=ymin2; wymax[t2]=ymax2;
        t=t2;
    }
}
void UNISTCG21_SA::FillGrid3D_back(int r, int tmax, int tm,int sx, int sy){ // Fills grid3D backwards from tmax to tmin
    //cout <<" FG3D_b";
    Grid3D[tmax][sx][sy]=15;
    wxmin[tmax]=sx;	// Coordinates of the current window when traversing the grid
    wxmax[tmax]=sx;
    wymin[tmax]=sy;
    wymax[tmax]=sy;
    int t=tmax;
    while(t!=tm){
        int t2=t-1;
        int xmin=wxmin[t];	int xmax=wxmax[t];
        int ymin=wymin[t];	int ymax=wymax[t];
        int xmin2=xmin; 	int xmax2=xmax;
        int ymin2=ymin; 	int ymax2=ymax;
        for(int x=xmin;x<=xmax;x++){         // handles current robot at x,y does not move
            for(int y=ymin; y<=ymax; y++){
                char g=Grid3D[t][x][y];
                if(g<=5)
                    continue;
                char g2=Grid3D[t2][x][y];
                if(g2==0)
                    Grid3D[t2][x][y]=15;
            }
        }
        for(int x=xmin;x<=xmax;x++){          // handles current robot at x,y   moves
            for(int y=ymin; y<=ymax; y++){
                char g=Grid3D[t][x][y];
                if(g<=5){
                    continue;
                }
                for(char d=1;d<5;d++){
                    int x2=Unmove_x(x,d);
                    int y2=Unmove_y(y,d);
                    char g2=Grid3D[t2][x2][y2];
                    if(g2!=0)                                          // next cell not free
                        continue;
                    char g3=Grid3D[t2][x][y];
                    if((g3>=1) && (g3<=5) && (d!=g3) )                        // swarm condition 1
                        continue;
                    char g4=Grid3D[t][x2][y2];
                    if(g4>=1 && g4<=5){                                                   // swarm condition 2
                        int x3=Unmove_x(x2,d);
                        int y3=Unmove_y(y2,d);
                        if (Grid3D[t2][x3][y3]!=d)
                            continue;
                    }
                    Grid3D[t2][x2][y2]=10+d;
                    xmin2=min(xmin2,x2); xmax2=max(xmax2,x2); ymin2=min(ymin2,y2); ymax2=max(ymax2,y2);
                }
            }
        }
        wxmin[t2]=xmin2; wxmax[t2]=xmax2; wymin[t2]=ymin2; wymax[t2]=ymax2;
        t=t2;
    }
}
void UNISTCG21_SA::FillGrid3D2(int r, int tmin, int tmax,int sx, int sy){ // Fills grid3D with values 11,...,15
    Grid3D[tmin][sx][sy]=15;
    Grid3D2[tmin][sx][sy]=0;

    wxmin[tmin]=sx;	// Coordinates of the current window when traversing the grid
    wxmax[tmin]=sx;
    wymin[tmin]=sy;
    wymax[tmin]=sy;
    int t=tmin;
    while(t!=tmax){
        int t2=t+1;
        int xmin=wxmin[t];	int xmax=wxmax[t];
        int ymin=wymin[t];	int ymax=wymax[t];
        int xmin2=xmin; 	int xmax2=xmax;
        int ymin2=ymin; 	int ymax2=ymax;
        for(int x=xmin;x<=xmax;x++){         // handles current robot at x,y does not move
            for(int y=ymin; y<=ymax; y++){
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
        for(int x=xmin;x<=xmax;x++){          // handles current robot at x,y   moves
            for(int y=ymin; y<=ymax; y++){
                char g=Grid3D[t][x][y];
                if(g<=5){
                    continue;
                }
                for(char d=1;d<5;d++){
                    int x2=Move_x(x,d);
                    int y2=Move_y(y,d);
                    char g2=Grid3D[t2][x2][y2];
                    if(g2<0 or 1<=g2 and g2<=5) // next cell not free
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
                                xmin2=min(xmin2,x2); xmax2=max(xmax2,x2); ymin2=min(ymin2,y2); ymax2=max(ymax2,y2);
                            }
                        }
                        else if(g2==0){
                            if(Grid3D2[t2][x2][y2]==-1){
                                Grid3D[t2][x2][y2]=10+d;
                                Grid3D2[t2][x2][y2]=Grid3D2[t][x][y]+1;
                                xmin2=min(xmin2,x2); xmax2=max(xmax2,x2); ymin2=min(ymin2,y2); ymax2=max(ymax2,y2);
                            }
                        }
                    }
                }
            }
        }
        wxmin[t2]=xmin2; wxmax[t2]=xmax2; wymin[t2]=ymin2; wymax[t2]=ymax2;
        t=t2;
    }
}
void UNISTCG21_SA::FillGrid3D_back2(int r, int tmax, int tm,int sx, int sy){ // Fills grid3D backwards from tmax to tmin
    Grid3D[tmax][sx][sy]=15;
    Grid3D2[tmax][sx][sy]=0;
    wxmin[tmax]=sx;	// Coordinates of the current window when traversing the grid
    wxmax[tmax]=sx;
    wymin[tmax]=sy;
    wymax[tmax]=sy;
    int t=tmax;
    while(t!=tm){
        int t2=t-1;
        int xmin=wxmin[t];	int xmax=wxmax[t];
        int ymin=wymin[t];	int ymax=wymax[t];
        int xmin2=xmin; 	int xmax2=xmax;
        int ymin2=ymin; 	int ymax2=ymax;
        for(int x=xmin;x<=xmax;x++){         // handles current robot at x,y does not move
            for(int y=ymin; y<=ymax; y++){
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
        for(int x=xmin;x<=xmax;x++){          // handles current robot at x,y   moves
            for(int y=ymin; y<=ymax; y++){
                char g=Grid3D[t][x][y];
                if(g<=5){
                    continue;
                }
                for(char d=1;d<5;d++){
                    int x2=Unmove_x(x,d);
                    int y2=Unmove_y(y,d);
                    char g2=Grid3D[t2][x2][y2];
                    if(g2<0 or 1<=g2 and g2<=5)
                        continue;
                    else{
                        char g3=Grid3D[t2][x][y];
                        if((g3>=1) && (g3<=5) && (d!=g3) )                        // swarm condition 1
                            continue;
                        char g4=Grid3D[t][x2][y2];
                        if(g4>=1 && g4<=5){                                                   // swarm condition 2
                            int x3=Unmove_x(x2,d);
                            int y3=Unmove_y(y2,d);
                            if (Grid3D[t2][x3][y3]!=d)
                                continue;
                        }
                        if(g2>10){
                            if(Grid3D2[t2][x2][y2] > Grid3D2[t][x][y]+1){
                                Grid3D[t2][x2][y2]=10+d;
                                Grid3D2[t2][x2][y2]=Grid3D2[t][x][y]+1;
                                xmin2=min(xmin2,x2); xmax2=max(xmax2,x2); ymin2=min(ymin2,y2); ymax2=max(ymax2,y2);
                            }
                        }
                        else if(g2==0){
                            if(Grid3D2[t2][x2][y2]==-1){
                                Grid3D[t2][x2][y2]=10+d;
                                Grid3D2[t2][x2][y2]=Grid3D2[t][x][y]+1;
                                xmin2=min(xmin2,x2); xmax2=max(xmax2,x2); ymin2=min(ymin2,y2); ymax2=max(ymax2,y2);
                            }
                        }
                    }
                }
            }
        }
        wxmin[t2]=xmin2; wxmax[t2]=xmax2; wymin[t2]=ymin2; wymax[t2]=ymax2;
        t=t2;
    }
}

void UNISTCG21_SA::Stretch(int r, int tmin, int tmax){
    //cout <<" Stch : ";
    int sx,tx,sy,ty,dt;
    int number_of_moves0=number_of_moves[r];
    int is_max_makespan=makespan-completion_time[r];
    int tm=tmin+1+(rand()%(tmax-tmin-1));

    Position(r,tmin,sx,sy);
    Position(r,tmax,tx,ty);
    dt=Grid3D[tmax][tx][ty];
    RemoveRobot(r,tmin,tmax,sx,sy);

    if (Objective==1)
        FillGrid3D2(r,tmin,tm,sx,sy);
    else
        FillGrid3D(r,tmin,tm,sx,sy);
    for(int x=wxmin[tm];x<=wxmax[tm];x++)
        for(int y=wymin[tm];y<=wymax[tm];y++){
            char g=Grid3D[tm][x][y];
            interface[x][y]=g;
            if (g>10)
                Grid3D[tm][x][y]=0;
        }
    if (Objective==1){
        for(int x=wxmin[tm];x<=wxmax[tm];x++)
            for(int y=wymin[tm];y<=wymax[tm];y++)
                Grid3D2[tm][x][y]=-1;
    }
    int wx1=wxmin[tm]; int wx2=wxmax[tm];
    int wy1=wymin[tm]; int wy2=wymax[tm];
    if (Objective==1)
        FillGrid3D_back2(r,tmax,tm,tx,ty);
    else
        FillGrid3D_back(r,tmax,tm,tx,ty);

    wx1=max(wx1,wxmin[tm]); wx2=min(wx2,wxmax[tm]);
    wy1=max(wy1,wymin[tm]); wy2=min(wy2,wymax[tm]);

    int xm,ym;
    //cout << "w " << wx1 << " " << wx2 << " " << wy1 << " " << wy2 << endl;
    int debugi;
    for(debugi=0;debugi<1000000000;debugi++){
        xm=wx1+(rand()%(wx2-wx1+1));
        ym=wy1+(rand()%(wy2-wy1+1));
        if(interface[xm][ym]>10 && Grid3D[tm][xm][ym]>10)
            break;
    }
    //cout << "xm= " << xm << " ym=" << ym << endl;
    if (debugi==1000000000){
        cout << "***** ERROR Stretch 0" << endl;
        cout << " sx=" << sx << " sy=" << sy;
        cout << " xm=" << xm << " ym=" << ym;
        PrintInterface();
        PrintGrid3D();
        exit(0);
    }

    int xtemp=xm;
    int ytemp=ym;
    int ctemp=tmin;
    bool rmove=false;
    char d1,d2; //hyeyun : remove ambiguity
    for(int t=tm-1; t>=tmin; t--){               // write path for robot r in Grid3D
        //char d1,d2; //hyeyun: remove ambiguity
        // cout << xtemp << "," << ytemp << endl;
        if (t==tm-1){
            d2=interface[xm][ym]-10;
            xtemp=Unmove_x(xtemp,d2);
            ytemp=Unmove_y(ytemp,d2);
        }
        d1=Grid3D[t][xtemp][ytemp]-10;
        if (d2<1 || d2>5){
        cout << "***** ERROR Stretch 1" << endl;
        cout << " t= " << t << " xtemp=" << xtemp << " ytemp=" << ytemp;
        cout << " sx=" << sx << " sy=" << sy;
        cout << " xm=" << xm << " ym=" << ym;
        cout << " d1=" << int(d1) << " d2=" << int(d2) <<endl;
        PrintInterface();
        PrintGrid3D();
        exit(0);
        }
        Grid3D[t][xtemp][ytemp]=d2;
        if(d2!=5){
            number_of_moves[r]++;
            if(rmove==false){
                ctemp=t;
                rmove=true;
            }
        }
        if(t==tmin){
            if(xtemp!=sx || ytemp!=sy){
                cout << "***** ERROR Stretch 2" << endl;
                cout << " t= " << t << " xtemp= " << xtemp << " ytemp= " << ytemp;
                cout << " sx= " << sx << " sy= " << sy;
                cout << " d1= " << int(d1) << " d2=" << int(d2) <<endl;
                PrintInterface();
                PrintGrid3D();
                exit(0);
            }
        }
        xtemp=Unmove_x(xtemp,d1);
        ytemp=Unmove_y(ytemp,d1);
        d2=d1;
    }
    xtemp=xm; ytemp=ym;
    char d; //hyeyun : remove ambiguity
    for(int t=tm;t<tmax;t++){
        d=Grid3D[t][xtemp][ytemp]; //hyeyun : remove ambiguity
        //char d=Grid3D[t][xtemp][ytemp]; //hyeyun :remove ambiguity
        if (d<=10){
            cout << "***** ERROR Stretch 3" << endl;
            cout << t << " d=" << int(d) <<endl;
            PrintInterface();
            PrintGrid3D();
            exit(0);
        }
        d-=10;
        Grid3D[t][xtemp][ytemp]=d;
        xtemp=Move_x(xtemp,d);
        ytemp=Move_y(ytemp,d);
        if(d!=5){
            number_of_moves[r]++;
            rmove=true;
            ctemp=t;
        }
    }
    Grid3D[tmax][tx][ty]=dt;
    if (dt!=5)
        number_of_moves[r]++;
    if(tmin < completion_time[r] && completion_time[r]<=tmax)
        if(rmove==true)
            completion_time[r]=ctemp+1;
        else
            ComputeCompletionTime(r);
    if(completion_time[r] <= tmin && rmove==true)
        completion_time[r]=ctemp+1;
    if(tx!=xtemp || ty != ytemp){
        cout << "***** ERROR Stretch 4" << endl;
        exit(0);
    }
    if (Objective==1){
        for (int t=tmin;t<=tmax;t++){                     // clean the grid
            for(int x=wxmin[t];x<=wxmax[t];x++){
                for(int y=wymin[t];y<=wymax[t];y++){
                    if(Grid3D[t][x][y]>=10)
                        Grid3D[t][x][y]=0;
                    Grid3D2[t][x][y]=-1;
                }
            }
        }
    }else{
        for (int t=tmin;t<=tmax;t++){                     // clean the grid
            for(int x=wxmin[t];x<=wxmax[t];x++){
                for(int y=wymin[t];y<=wymax[t];y++){
                    if(Grid3D[t][x][y]>=10)
                        Grid3D[t][x][y]=0;
                }
            }
        }
    }
    total_moves+=number_of_moves[r]-number_of_moves0;
    if (is_max_makespan==0)
        makespan=*max_element(completion_time.begin(),completion_time.end());
    makespan=max(makespan,completion_time[r]);
    //cout <<" ---\n";
}

void UNISTCG21_SA::Tighten(int r, int tmin, int tmax){
    //cout <<" Tt : ";
    int sx,tx,sy,ty,dt;
    int number_of_moves0=number_of_moves[r];
    int is_max_makespan=makespan-completion_time[r];

    Position(r,tmin,sx,sy);
    Position(r,tmax,tx,ty);
    dt=Grid3D[tmax][tx][ty];
    RemoveRobot(r,tmin,tmax,sx,sy);

    if (Objective==1)
        FillGrid3D2(r,tmin,tmax,sx,sy);
    else
        FillGrid3D(r,tmin,tmax,sx,sy);

    if(Grid3D[tmax][tx][ty]<11){
      cout << "***** ERROR MoveOneRobot 1" << endl;
      cout << "robot=" << r << " g=" << int(Grid3D[tmax][tx][ty]) << " sx= " << sx << " sy=" << sy  << " tx= " << tx << " ty=" << ty << endl;
      cout << "grid_t=" << grid_t <<  endl;
      PrintGrid3D();
      exit(0);
    }
    int xtemp=tx;
    int ytemp=ty;
    int ctemp=tmin;
    bool rmove=false;
    char d1,d2; //hyeyun: remove ambiguity
    for(int t=tmax; t>=tmin; t--){               // write path for robot r in Grid3D
        //char d1,d2; //hyeyun: remove ambiguity
        // cout << xtemp << "," << ytemp << endl;
        d1=Grid3D[t][xtemp][ytemp]-10;
        if (t==tmax)
            d2=dt;
        Grid3D[t][xtemp][ytemp]=d2;
        if(d2!=5){
            number_of_moves[r]++;
            if(rmove==false){
                ctemp=t;
                rmove=true;
            }
        }
        xtemp=Unmove_x(xtemp,d1);
        ytemp=Unmove_y(ytemp,d1);
        d2=d1;
    }
    if(tmin < completion_time[r] && completion_time[r]<=tmax)
        if(rmove==true)
            completion_time[r]=ctemp+1;
        else
            ComputeCompletionTime(r);
    if(completion_time[r] <= tmin && rmove==true)
        completion_time[r]=ctemp+1;
    if(sx!=xtemp || sy != ytemp){
        cout << "***** ERROR MoveOneRobot 2" << endl;
        exit(0);
    }
    if (Objective==1){
        for (int t=tmin;t<=tmax;t++){                     // clean the grid
            for(int x=wxmin[t];x<=wxmax[t];x++){
                for(int y=wymin[t];y<=wymax[t];y++){
                    if(Grid3D[t][x][y]>=10)
                        Grid3D[t][x][y]=0;
                    Grid3D2[t][x][y]=-1;
                }
            }
        }
    }else{
        for (int t=tmin;t<=tmax;t++){                     // clean the grid
            for(int x=wxmin[t];x<=wxmax[t];x++){
                for(int y=wymin[t];y<=wymax[t];y++){
                    if(Grid3D[t][x][y]>=10)
                        Grid3D[t][x][y]=0;
                }
            }
        }
    }
    total_moves+=number_of_moves[r]-number_of_moves0;
    if (is_max_makespan==0)
        makespan=*max_element(completion_time.begin(),completion_time.end());
    makespan=max(makespan,completion_time[r]);
    //cout <<" ---\n";
}

int UNISTCG21_SA::ComputeCompletionTime(int r){			// Returns the completion time of robot r
    int t=grid_t-1;
    while(Grid3D[t][target_x[r]][target_y[r]]==5 && t>0)
        t--;
    completion_time[r]=t+1;
    return t;
}
void UNISTCG21_SA::UnRemoveRobot(int r, int tmin, int tmax){
    //cout <<" UnRemoveR\n";
    int x=prev_traj_x;
    int y=prev_traj_y;
    char d;//hyeyun : remove ambiguity
    for(int t=tmin; t<=tmax; t++){
        //char d;//hyeyun :remove ambiguity
        d=Grid3D[t][x][y];
        if(d<1 || d>5){
            cout << "***** ERROR UnRemoveRobot 1" << endl;
            exit(0);
        }
        Grid3D[t][x][y]=0;
        if(t!=tmax){
            x=Move_x(x,d);
            y=Move_y(y,d);
        }
    }
    x=prev_traj_x;
    y=prev_traj_y;
    for(int t=tmin; t<=tmax; t++){
        //char d; //hyeyun: remove ambiguity
        d=prev_traj[t];
        Grid3D[t][x][y]=d;
        if(d<1 || d>5){
            cout << "***** ERROR UnRemoveRobot 2" << endl;
            exit(0);
        }
        x=Move_x(x,d);
        y=Move_y(y,d);
    }
}
void UNISTCG21_SA::RemoveRobot(int r, int tmin, int tmax, int x, int y){			// Removes trajectory of robot r from Grid3D
    //cout <<" RR,";
    prev_traj_x=x; prev_traj_y=y;	// For UnRemoveRobot
    char d; //hyeyun: remove ambiguity
    for(int t=tmin; t<=tmax; t++){              // between time tmin and tmax (included)
        //char d; //hyeyun: remove ambiguity
        d=Grid3D[t][x][y];
        prev_traj[t]=d;					// For UnRemoveRobot
        if(d<1 || d>5){
          cout << "***** ERROR RemoveRobot" << endl;
          exit(0);
        }
        if (d!=5)
            number_of_moves[r]--;
        Grid3D[t][x][y]=0;
        if(t!=tmax){
            x=Move_x(x,d);
            y=Move_y(y,d);
        }
    }
}

void UNISTCG21_SA::ComputeScores(){
    total_moves=0;
    makespan=0;
    completion_time.resize(num_rb);
    number_of_moves.resize(num_rb);
    vector<short> rx(num_rb,0);
    vector<short> ry(num_rb,0);
    for(int r=0; r<num_rb; r++){
        rx[r]=start_x[r];
        ry[r]=start_y[r];
        number_of_moves[r]=0;
        completion_time[r]=0;
    }
    for(int t=0; t<grid_t-1; t++){
        bool flag=false;
        for(int x=0; x<grid_x; x++)
            for(int y=0; y<grid_y; y++){
                int d=Grid3D[t][x][y];
                if(d>=1 && d<=4){
                    total_moves++;
                    flag=true;
                }
            }
        if (flag==true)
            makespan=t+1;
    }
    // cout << "***** ComputeScores ****" << endl;
    // cout << "makespan=" << makespan << "  total moves=" << total_moves << endl;
    for(int t=0;t<grid_t-1;t++){
        for(int r=0; r<num_rb; r++){
            int a=rx[r];
            int b=ry[r];
            char d=Grid3D[t][a][b];
            rx[r]=Move_x(a,d);
            ry[r]=Move_y(b,d);
            if(d<1 || d> 5){
                cout << "ERROR ComputeScore 1" << endl;
                exit(0);
            }
            if (d!=5){
                completion_time[r]=t+1;
                number_of_moves[r]++;
            }
        }
    }
    /*for(int r=0;r<num_rb; r++){
      cout << "robot " << r << " completion time: " << completion_time[r];
      cout << "  number of moves: " << number_of_moves[r] << endl;
      }*/
    if (total_moves!=accumulate(number_of_moves.begin(),number_of_moves.end(),0)){
        cout << "ERROR ComputeScore 2" << endl;
        exit(0);
    }
    if (makespan!=*max_element(completion_time.begin(),completion_time.end())){
        cout << "ERROR ComputeScore 2" << endl;
        exit(0);
    }
}

int UNISTCG21_SA::CheckGrid3D(){					// checks whether the data in Grid3D is valid.
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

void UNISTCG21_SA::ReadData(){
    cout << "----------------------- ReadData -------------------------" << endl;
    Json::Value root;
    short tx = 2;
    short ty = 2;
    int gx = 3;
    int gy = 3;
    Json::Reader reader;

    ifstream json(input,ifstream::binary);
    if (json.fail()){
        cout << "ReadData cannot read input file" << endl;
        exit(0);
    }
    reader.parse(json, root);
    Json::Value meta = root["meta"];
    num_rb = meta["number_of_robots"].asInt();
    shape = meta["description"]["parameters"]["shape"][0].asInt();

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

void UNISTCG21_SA::WriteFile(){
    string filename;
    if(Objective==1){
        filename="SUM"+to_string(best_total_moves)+".json";
    }
    else{
        filename="MAX"+to_string(best_makespan)+".json";
    }
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
    for(int t=0;t<best_makespan;t++){
        solution<<"{";
        int flag=0;
        for(int i=0;i<num_rb;i++){
            //d=Grid3D[t][rx[i]][ry[i]];
            d=Grid3Dbest[t][rx[i]][ry[i]]; //hyeyun
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
                    solution<<"\"E\" ";
                }
                else if(d==2){
                    solution<<"\"W\" ";
                }
                else if(d==3){
                    solution<<"\"S\" ";
                }
                else if(d==4){
                    solution<<"\"N\" ";
                }
            }
        }
        if (t<best_makespan-1){
            solution<<"},\n";
        }
        else{
            solution<<"}\n";
        }
    }
    solution<<"]\n";
    solution<<"}\n";
    solution.close();
}

void UNISTCG21_SA::WriteVisual(){
    string filename;
    if(Objective==1){
        filename="./visual_main_/main_MIN_"+output+"_"+to_string(best_total_moves)+".js";
    }
    else{
        filename="./visual_main_/main_MAX_"+output+"_"+to_string(best_makespan)+".js";
    }

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

    vector<int> rx(num_rb,0);
    vector<int> ry(num_rb,0);
    for(int r=0;r<num_rb;r++){
        rx[r]=start_x[r];
        ry[r]=start_y[r];
    }
    for(int t=0;t<best_makespan;t++){
        result << "[\n";
        for(int r=0;r<num_rb;r++){
            int a,b;
            a=rx[r]; b=ry[r];
            //rx[r]=Move_x(a,Grid3D[t][a][b]);
            //ry[r]=Move_y(b,Grid3D[t][a][b]);
            rx[r]=Move_x(a,Grid3Dbest[t][a][b]); //hyeyun
            ry[r]=Move_y(b,Grid3Dbest[t][a][b]);
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


void UNISTCG21_SA::ReadSolution(){
    cout << "----------------------- ReadSolution -------------------------" << endl;
    clock_t s = (int) clock();
    Json::Value root;
    Json::Reader reader;

    ifstream json(solution,ifstream::binary);
    if (json.fail()){
        cout << "ReadSolution cannot read input file" << endl;
        cout << solution <<endl;
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
    vector<int>::iterator mine = robot_x.begin();
    if (num_ob>0){
        xmin=min(xmin,*min_element(obstacle_x.begin(),obstacle_x.end()));
        xmax=max(xmax,*max_element(obstacle_x.begin(),obstacle_x.end()));
        ymin=min(ymin,*min_element(obstacle_y.begin(),obstacle_y.end()));
        ymax=max(ymax,*max_element(obstacle_y.begin(),obstacle_y.end()));
    }

    grid_t=1;
    for(auto it = Steps.begin();it !=Steps.end();it++){
        grid_t++;
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
            robot_x[r]=Move_x(robot_x[r],d);
            robot_y[r]=Move_y(robot_y[r],d);
            // cout << "robot " << r << " moves to ("<< robot_x[r] << ", " << robot_y[r] << ")" << endl;
            xmin=min(xmin,robot_x[r]);
            xmax=max(xmax,robot_x[r]);
            ymin=min(ymin,robot_y[r]);
            ymax=max(ymax,robot_y[r]);
        }
    }

    int tx=-xmin+1+grid_xy_margin;            // Translate the grid
    int ty=-ymin+1+grid_xy_margin;
    grid_x=tx+xmax+2+grid_xy_margin;
    grid_y=ty+ymax+2+grid_xy_margin;
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


    int grid_t2=int(grid_t_coef*double(grid_t));
    Grid3D.resize(grid_t2);           	// Fill the 3D grid
    for(int t=0; t<grid_t2; t++){
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
    if(Objective==1){
        Grid3D2.resize(grid_t2);
        for(int t=0; t<grid_t2; t++){
            Grid3D2[t].resize(grid_x);
            for(int x=0; x<grid_x; x++)
                Grid3D2[t][x].resize(grid_y,-1);
            for(int k=0;k<num_ob;k++)
                Grid3D[t][obstacle_x[k]][obstacle_y[k]]=-1;
            for(int x=0;x<grid_x;x++){
                Grid3D2[t][x][0]=-2;
                Grid3D2[t][x][grid_y-1]=-2;
            }
            for(int y=0;y<grid_y;y++){
                Grid3D2[t][0][y]=-2;
                Grid3D2[t][grid_x-1][y]=-2;
            }
        }
    }
    for(int r=0;r<num_rb;r++){
        for(int t=grid_t-1;t<grid_t2;t++)
            Grid3D[t][target_x[r]][target_y[r]]=5;        // d corresponds to robot with next move d in {1,2,3,4,5}
    robot_x[r]=start_x[r];
    robot_y[r]=start_y[r];
    }
    int t=0;
    for(auto it = Steps.begin();it !=Steps.end();it++){
        for(int r=0; r<num_rb; r++){
            Grid3D[t][robot_x[r]][robot_y[r]]=5;
        }
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
            Grid3D[t][robot_x[r]][robot_y[r]]=d;
            robot_x[r]=Move_x(robot_x[r],d);
            robot_y[r]=Move_y(robot_y[r],d);
        }
        t++;
    }
    grid_t=grid_t2;
    wxmin.resize(grid_t);
    wxmax.resize(grid_t);
    wymin.resize(grid_t);
    wymax.resize(grid_t);
    interface.resize(grid_x);
    grid.resize(grid_x);
    for(int x=0;x<grid_x;x++){
        interface[x].resize(grid_y);
        grid[x].resize(grid_y);
        for(int y=0;y<grid_y;y++)
            if (Grid3D[0][x][y]==-1){
                interface[x][y]=-1;
                grid[x][y]=-1;
            }
            else{
                interface[x][y]=0;
                grid[x][y]=0;
            }
    }
    prev_traj.resize(grid_t);
    //cout << "grid_x=" << grid_x << " grid_y= " << grid_y << " tx=" << tx << " ty=" << ty << " grid_t=" << grid_t << " *******************" << endl;
    root.clear();
    Steps.clear();
    Steps.resize(0);

    ComputeScores();
    cout << "Initial makespan= " << makespan << " total moves= " << total_moves << endl;
}

void UNISTCG21_SA::WriteScore(){
    struct tm curr_tm;
    time_t curr_time = time(nullptr);
    localtime_r(&curr_time, &curr_tm);
    ofstream score("score.txt", ios::app);
    score <<" " <<curr_tm.tm_mday<<"/"<<curr_tm.tm_mon+1<<"_";
    score << curr_tm.tm_hour<<":"<<curr_tm.tm_min<<":"<<curr_tm.tm_sec;
    score << setprecision(4) << setw(6) << elapse_t << " s";
    score << " rb: "<< num_rb<<" ob: "<<num_ob;
    score << " " << best_total_moves;
    score << " " << best_makespan<<endl;
    score.close();
}

void UNISTCG21_SA::CleanGrid(){
    for(int x=0;x<grid_x;x++)
        for(int y=0;y<grid_y;y++)
            if (grid[x][y]!=-1)
                grid[x][y]=0;
}

void UNISTCG21_SA::PrintTrajectory(int r){
    int x=start_x[r];
    int y=start_y[r];
    for(int t=0; t<grid_t; t++){
        char d=Grid3D[t][x][y];
        grid[x][y]=t+1;
        if(Grid3D[t][x][y]<1 || Grid3D[t][x][y]>5){
            cout << "***** ERROR PrintTrajectory" << endl;
            cout << "r=" << r << " t=" << t << " x=" << x << " y=" << y << " d=" << int(d) << endl;
            PrintGrid3D();
            exit(0);
        }
        x=Move_x(x,d);
        y=Move_y(y,d);
    }
    for(int j=grid_y-1;j>=0;j--){
        for(int i=0;i<grid_x;i++){
            if(grid[i][j]==0){
                cout <<" "<<setw(3)<<"-";
            }
            else if(grid[i][j]==-1){
                cout <<" "<<setw(3)<<"X";
            }
            else{
                cout <<" "<<setw(3) << grid[i][j]-1;
            }
        }
        cout <<endl;
    }
    cout << "-----------------------------------------------------------------------------" << endl;
    CleanGrid();
}
void UNISTCG21_SA::PrintGrid3D(){
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
void UNISTCG21_SA::PrintInterface(){
    cout << "-----------------------------------------------------------------------------" << endl;
    cout << "Interface" << endl;
    cout << "-----------------------------------------------------------------------------" << endl;
    for(int j=grid_y-1;j>=0;j--){
        for(int i=0;i<grid_x;i++){
            if(interface[i][j]==0){
                cout <<" "<<setw(2)<<"-";
            }
            else if(interface[i][j]==-1){
                cout <<" "<<setw(2)<<"X";
            }
            else{
                cout <<" "<<setw(2) << int(interface[i][j]);
            }
        }
        cout <<endl;
    }
    cout << "-----------------------------------------------------------------------------" << endl;
}
