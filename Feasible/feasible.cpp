#include "feasible.h"

void Feasible::GridInitialize(){
    Dist_Grid.assign(grid_y,vector<pair<short,short>>(grid_x,make_pair(0,0)));
    if(num_ob !=0){
        for(int i=0;i<num_ob;i++){
            Set(Dist_Grid,obstacle_x[i],gy1-obstacle_y[i],-1,-1);
        }
    }
    for(int i=0;i<num_rb;i++){
        dg.push_back(new Dist);
        dg[i]->rb = i;
    }
    AllManhattan.assign(num_rb,0);
    for(int i=0;i<num_rb;i++){
        for(int k=0;k<grid_x;k++){
            for(int l=0;l<grid_y;l++){
                if(Value(Dist_Grid,k,l) > 0){
                    Set(Dist_Grid,k,l,0,0);
                }
            }
        }
        BFSdist(Dist_Grid,target_x[i],target_y[i]);
        AllManhattan[i] = Value(Dist_Grid,start_x[i],gy1-start_y[i]);
        //cout <<"----------robot #"<<i<<" = "<<"("<< start_x[i]<<", "<< start_y[i]<<")-->"<<AllManhattan[i]<<"\n";
        //PrintGrid(Dist_Grid);
        //cout <<endl;
    }
}

void Feasible::MoveOrder(){
    /* M = (0, gy1) on plan, =(0,0) on grid*/
    //BFSdist(Dist_Grid,0,gy1); /* working_SA, working_SA3*/
    BFSdist(Dist_Grid,grid_x-1,0); /* working_SA2, some instances have better result */
    priority_queue<Dist,vector<Dist*>,dist_compare> mv_order;

    /* order for S->M*/
    for(int i=0;i<num_rb;i++){
        dg[i]->dist = Value(Dist_Grid,start_x[i],gy1-start_y[i]);
        mv_order.push(dg[i]);
    }
    move_order.assign(num_rb,0);
    vector<int>::iterator it = move_order.begin();
    for(;it!=move_order.end();it++){
        *it = mv_order.top()->rb;
        mv_order.pop();
    }
    /* order for M->T*/
    for(int i=0;i<num_rb;i++){
        dg[i]->dist = Value(Dist_Grid,target_x[i],gy1-target_y[i]);
        mv_order.push(dg[i]);
    }
    move_order1.assign(num_rb,0);
    vector<int>::reverse_iterator rit = move_order1.rbegin();
    for(;rit!=move_order1.rend();rit++){
        *rit = mv_order.top()->rb;
        mv_order.pop();
    }
}

void Feasible::PrintMoveOrder(){
    cout <<"MoveOrder S-->M:\n";
    int th=1;
    for(int i:move_order){
        cout<<" "<<th<<": "<<i<<"("<<dg[i]->dist<<")";
        th++;
    }
    cout <<endl;
}

void Feasible::PrintMoveOrder1(){
    cout <<"MoveOrder M-->T:\n";
    int th=1;
    for(int i:move_order1){
        cout<<" "<<th<<": "<<i<<"("<<dg[i]->dist<<")";
        th++;
    }
    cout <<endl;
}

void Feasible::BFSdist(vector<vector<pair<short,short>>> &dist_g,int zero_x, int zero_y){
    Set(dist_g,zero_x,gy1-zero_y, -2,-2);
    for(int i=0;i<grid_x;i++){
        for(int j=0;j<grid_y;j++){
            if(Value(dist_g,i,j)>0){
                Set(dist_g,i,j,0,0);
            }
        }
    }
    queue<pair<short,short>> xy;
    vector<int> direction = {-1,1};
    for(int di: direction){
        int a = zero_x+di;
        int b = zero_y;
        if(a<grid_x and a >=0){
            if(First(dist_g,a,gy1-b) >=0){
                if(a!=zero_x or b!=zero_y){
                    xy.push(make_pair(a,b));
                    dist_g[gy1-b][a] = make_pair(1,0);
                }
            }
        }
    }
    for(int di: direction){
        int a = zero_x;
        int b = zero_y+di;
        if(b<grid_y and b >=0){
            if(First(dist_g,a,gy1-b) >=0){
                if(a!=zero_x or b!=zero_y){
                    xy.push(make_pair(a,b));
                    dist_g[gy1-b][a] = make_pair(0,1);
                }
            }
        }
    }
    int count = xy.size();
    int d=1;

    while(!xy.empty()){
        for(int di: direction){
            int a = xy.front().first + di;
            int b = xy.front().second;
            if(a<grid_x and a >=0){
                if(Value(dist_g,a,gy1-b)==0 or Value(dist_g,a,gy1-b) > d+1){
                    if(a!=zero_x or b!=zero_y){
                        xy.push(make_pair(a,b));
                        dist_g[gy1-b][a] = make_pair(First(dist_g,a-di,gy1-b)+1,Second(dist_g,a-di,gy1-b));
                    }
                }
            }
        }
        for(int di: direction){
            int a = xy.front().first;
            int b = xy.front().second + di;
            if(b<grid_y and b >=0){
                if(Value(dist_g,a,gy1-b)==0 or Value(dist_g,a,gy1-b) > d+1){
                    if(a!=zero_x or b!=zero_y){
                        xy.push(make_pair(a,b));
                        dist_g[gy1-b][a] = make_pair(First(dist_g,a,gy1-b+di),Second(dist_g,a,gy1-b+di)+1);
                    }
                }
            }
        }
        xy.pop();
        if(count ==1){
            d++;
            count = xy.size();
        }
        else{
            count--;
        }
    }
    Set(dist_g,zero_x,gy1-zero_y, 0,0);
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

void Feasible::MoveOneRobot(int tmin, int r, int sx, int sy, int tx, int ty){                // ANTOINE: auxiliary function for Simultaneous()
    //printf("\nrobot #%d: s(%d, %d) --> t(%d, %d) \n", r+1, sx,sy,tx,ty);

    Grid3D[tmin][sx][sy]=15;                              //10+d means current robot has previous move d
    for(int t=tmin+1; t<Grid3D.size(); t++){
        Grid3D[t][sx][sy]=0;
    }
    for(int t=tmin;t>=0;t++){
        int t2=t+1;
        if(t==Grid3D.size()-1){    // resizing if necessary
            Grid3D.resize(t+2);
            Grid3D[t+1].resize(grid_x);
            for(int x=0;x<grid_x;x++){
                Grid3D[t+1][x].resize(grid_y,0);
            }
            for(int x=0;x<grid_x;x++){
                for(int y=0;y<grid_y;y++){
                    int temp=Grid3D[t][x][y];
                    if(temp <= 0){
                        Grid3D[t2][x][y]=temp;
                    }
                    else if(temp <= 5){
                        Grid3D[t2][x][y]=5;
                    }
                }
            }
            Grid3D[t2][sx][sy]=0;
        }
        for(int x=0;x<grid_x;x++){         // handles current robot at x,y does not move
            for(int y=0; y<grid_y; y++){
                char g=Grid3D[t][x][y];
                if(g<=5)
                    continue;
                char g2=Grid3D[t2][x][y];
                if(g2==0)
                    Grid3D[t2][x][y]=15;
            }
        }
        for(int x=0;x<grid_x;x++){          // handles current robot at x,y   moves
            for(int y=0; y<grid_y; y++){
                char g=Grid3D[t][x][y];
                if(g<=5){
                    continue;
                }
                for(char d=1;d<5;d++){
                    int x2=Move_x(x,d);
                    int y2=Move_y(y,d);
                    if(x2<0 || y2 <0 || x2>=grid_x || y2>=grid_y)         // out of bound
                        continue;
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
                        if (x3<0 || x3>=grid_x || y3<0 || y3>= grid_y || Grid3D[t][x3][y3]!=d)
                            continue;
                    }
                    Grid3D[t2][x2][y2]=10+d;
                }
            }
        }
        if((t2==Grid3D.size()-1) && (Grid3D[t2][tx][ty]>=10)){
            t=-2; // will exit outer loop
        }
    }
    for(int t=Grid3D.size()-1; t>=tmin; t--){               // write path for robot r in Grid3D
        char d1,d2; //  remove ambiguity?
        d1=Grid3D[t][tx][ty]-10;
        if (t==Grid3D.size()-1){
            Grid3D[t][tx][ty]=5;
        }
        else{
            Grid3D[t][tx][ty]=d2;
        }
        tx=Unmove_x(tx,d1);
        ty=Unmove_y(ty,d1);
        d2=d1;
    }
    for (int t=tmin;t<Grid3D.size();t++){                     // clean the grid
        for(int x=0;x<grid_x;x++){
            for(int y=0;y<grid_y;y++){
                if(Grid3D[t][x][y]>=10){
                    Grid3D[t][x][y]=0;
                }
            }
        }
    }
}

void Feasible::MoveOneRobot1(int tmin, int r, int sx, int sy, int tx, int ty){                // ANTOINE: auxiliary function for Simultaneous()
    //printf("\nrobot #%d: s(%d, %d) --> t(%d, %d) \n", r+1, sx,sy,tx,ty);

    Grid3D[tmin][sx][sy]=15;                              //10+d means current robot has previous move d
    for(int t=tmin+1; t<Grid3D.size(); t++){
        Grid3D[t][sx][sy]=0;
    }
    for(int t=tmin;t>=0;t++){
        int t2=t+1;
        if(t==Grid3D.size()-1){    // resizing if necessary
            Grid3D.resize(t+2);
            Grid3D2.resize(t+2);
            Grid3D[t+1].resize(grid_x);
            Grid3D2[t+1].resize(grid_x);
            for(int x=0;x<grid_x;x++){
                Grid3D[t+1][x].resize(grid_y,0);
                Grid3D2[t+1][x].resize(grid_y,-1);
            }
            for(int x=0;x<grid_x;x++){
                for(int y=0;y<grid_y;y++){
                    int temp=Grid3D[t][x][y];
                    if(temp <= 0){
                        Grid3D[t2][x][y]=temp;
                    }
                    else if(temp <= 5){
                        Grid3D[t2][x][y]=5;
                    }
                }
            }
            Grid3D[t2][sx][sy]=0;
        }
        Grid3D2[t][sx][sy]=0;
        for(int x=0;x<grid_x;x++){         // handles current robot at x,y does not move
            for(int y=0; y<grid_y; y++){
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
        for(int x=0;x<grid_x;x++){          // handles current robot at x,y   moves
            for(int y=0; y<grid_y; y++){
                char g=Grid3D[t][x][y];
                if(g<=5){
                    continue;
                }
                for(char d=1;d<5;d++){
                    int x2=Move_x(x,d);
                    int y2=Move_y(y,d);
                    if(x2<0 || y2 <0 || x2>=grid_x || y2>=grid_y)         // out of bound
                        continue;
                    char g2=Grid3D[t2][x2][y2];
                    if(g2<0 or 1<=g2 and g2<=5)                          // next cell not free
                        continue;
                    else{
                        char g3=Grid3D[t][x2][y2];
                        if((g3>=1) && (g3<=5) && (d!=g3) )                        // swarm condition 1
                            continue;
                        char g4=Grid3D[t2][x][y];
                        if(g4>=1 && g4<=5){                                                   // swarm condition 2
                            int x3=Unmove_x(x,d);
                            int y3=Unmove_y(y,d);
                            if (x3<0 || x3>=grid_x || y3<0 || y3>= grid_y || Grid3D[t][x3][y3]!=d)
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
        //PrintGrid3D();
        if((t2==Grid3D.size()-1) && (Grid3D[t2][tx][ty]>=10)){
            t=-2; // will exit outer loop
        }
    }
    for(int t=Grid3D.size()-1; t>=tmin; t--){               // write path for robot r in Grid3D
        char d1,d2;
        d1=Grid3D[t][tx][ty]-10;
        if (t==Grid3D.size()-1){
            Grid3D[t][tx][ty]=5;
        }
        else{
            Grid3D[t][tx][ty]=d2;
        }
        tx=Unmove_x(tx,d1);
        ty=Unmove_y(ty,d1);
        d2=d1;
    }
    for (int t=tmin;t<Grid3D.size();t++){                     // clean the grid
        for(int x=0;x<grid_x;x++){
            for(int y=0;y<grid_y;y++){
                if(Grid3D[t][x][y]>=10){
                    Grid3D[t][x][y]=0;
                }
                Grid3D2[t][x][y]=-1;
            }
        }
    }
    Grid3D2[tmin][sx][sy]=-1;
}

void Feasible::Simultaneous(){  // ANTOINE: smaller makespan through simultaneous movement
    cout << "----------------------- Simultaneous -------------------------" << endl;
    Grid3D.resize(1);           // initializing the first slice of the 3D grid
    Grid3D[0].resize(grid_x);
    for(int i=0;i<grid_x;i++)
        Grid3D[0][i].resize(grid_y,0);
    for(int k=0;k<num_ob;k++)
        Grid3D[0][obstacle_x[k]][obstacle_y[k]]=-1;
    for(int k=0;k<num_rb;k++){
        Grid3D[0][start_x[k]][start_y[k]]=5;       // d corresponds to robot with next move d in {1,2,3,4,5}
    }

    for( int r:move_order)
        MoveOneRobot(0, r, start_x[r], start_y[r], dg[r]->u, gy1-dg[r]->v);
    int tmin=Grid3D.size()-1;
    for( int r:move_order1)
        MoveOneRobot(tmin, r, dg[r]->u, gy1-dg[r]->v,target_x[r],target_y[r]);
    //PrintGrid3D();
    //cout <<"Simultaneous ends\n";
}

void Feasible::Simultaneous1(){  // ANTOINE: smaller makespan through simultaneous movement
    cout << "----------------------- Simultaneous -------------------------" << endl;
    Grid3D.resize(1);          // initializing the first slice of the 3D grid
    Grid3D[0].resize(grid_x);
    Grid3D2.resize(1);
    Grid3D2[0].resize(grid_x);
    for(int i=0;i<grid_x;i++){
        Grid3D[0][i].resize(grid_y,0);
        Grid3D2[0][i].resize(grid_y,-1);
    }
    for(int k=0;k<num_ob;k++){
        Grid3D[0][obstacle_x[k]][obstacle_y[k]]=-1;
    }
    for(int k=0;k<num_rb;k++){
        Grid3D[0][start_x[k]][start_y[k]]=5;       // d corresponds to robot with next move d in {1,2,3,4,5}
    }

    for( int r:move_order){
        MoveOneRobot1(0, r, start_x[r], start_y[r], dg[r]->u, gy1-dg[r]->v);
    }
    int tmin=Grid3D.size()-1;
    for( int r:move_order1){
        MoveOneRobot1(tmin, r, dg[r]->u, gy1-dg[r]->v,target_x[r],target_y[r]);
    }
    //PrintGrid3D();
    //cout <<"Simultaneous ends\n";
}

void Feasible::UF(){
    /* Compute Layer */
    int temp_rbn=0;
    for(int ly=1;ly>0;ly++){
        temp_rbn += ((shape/2+1)+2*(ly-1))*4;
        if(temp_rbn >= num_rb){
            Layer = ly;
            break;
        }
    }
    U.assign(temp_rbn,make_pair(0,0));
    F.assign(temp_rbn,1);
    //cout <<"U size = "<< U.size()<<endl;
    /* Compute coordiate of middle target */
    short bottom = -2;
    short top = shape+1;
    short uid = 0;
    short prev = 0;
    short length = 0;
    for(int ly=1;ly<=Layer;ly++){
        prev += length*4;
        length = (shape/2+1)+2*(ly-1);
        for(int i=0;i<length;i++){
            U.at(uid+prev) = make_pair(bottom, bottom+1+2*i);
            U.at(uid+length+prev) = make_pair(bottom+1+2*i,top);
            U.at(uid+2*length+prev) = make_pair(top,top-1-2*i);
            U.at(uid+3*length+prev) = make_pair(top-1-2*i,bottom);
            uid++;
        }
        uid = 0;
        bottom -=2;
        top +=2;
    }
    int t = Layer*2;
    for(int i=0;i<U.size();i++){
        U.at(i).first +=t;
        U.at(i).second +=t;
    }
}

void Feasible::UF2(){ /* handle instance which grid_x != grid_y*/
    /* Compute Layer */
    int temp_rbn=0;
    cout <<"grid_x = "<< grid_x<<", grid_y = "<< grid_y<<endl;
    if(grid_x!= grid_y){
        if(grid_y%2==0 and grid_x%2==1){
            short bottom = -2;
            short top = grid_y+1;
            short uid = 0;
            short prev = 0;
            short length_y = 0;
            short length_x = 0;
            length_y = (grid_y+2)/2+1;
            length_x = (grid_x+1)/2;
            for(int ly=1;ly>0;ly++){
                if(ly>1){
                    length_y +=2;
                    length_x +=2;
                }
                temp_rbn += 2*(length_y+length_x);
                if(temp_rbn >= num_rb){
                    Layer = ly;
                    break;
                }
            }
            U.assign(temp_rbn,make_pair(0,0));
            F.assign(temp_rbn,1);
            cout <<"U size = "<< U.size()<<endl;
            length_y = (grid_y+2)/2+1;
            length_x = (grid_x+1)/2;
            for(int ly=1;ly<=Layer;ly++){
                if(ly>1){
                    prev +=2*(length_y+length_x);
                    length_y +=2;
                    length_x +=2;
                }
                for(int i=0;i<length_y;i++){
                    U.at(uid+prev) = make_pair(bottom, bottom+1+2*i);
                    U.at(uid+length_y+length_x+prev) = make_pair(grid_x-1+2*ly, top-2*i);
                    uid++;
                }
                uid=0;
                for(int i=0;i<length_x;i++){
                    U.at(uid+length_y+prev) = make_pair(bottom+2*(i+1), top);
                    U.at(uid+2*length_y+length_x+prev) = make_pair(grid_x-1+2*ly-2*(i+1),bottom);
                    uid++;
                }
                uid = 0;
                bottom -=2;
                top +=2;
            }
        }
        else if(grid_y%2==0 and grid_x%2==0){
            short bottom = -2;
            short top = grid_y+1;
            short uid = 0;
            short prev = 0;
            short length_y = 0;
            short length_x = 0;
            for(int ly=1;ly>0;ly++){
                length_y = (grid_y+2*(ly-1)+2)/2;
                length_x = (grid_x+2*(ly-1)+2)/2;
                temp_rbn += 2*(length_y+length_x);
                if(temp_rbn >= num_rb){
                    Layer = ly;
                    break;
                }
            }
            U.assign(temp_rbn,make_pair(0,0));
            F.assign(temp_rbn,1);
            cout <<"U size = "<< U.size()<<endl;
            length_y = 0;
            length_x = 0;
            for(int ly=1;ly<=Layer;ly++){
                if(ly>1){
                    prev += 2*(length_x+length_y);
                }
                length_y = (grid_y+2*(ly-1)+2)/2;
                length_x = (grid_x+2*(ly-1)+2)/2;
                for(int i=0;i<length_y;i++){
                    U.at(uid+prev) = make_pair(bottom+1, bottom+1+2*i);
                    U.at(uid+length_y+length_x+prev) = make_pair(grid_x-1+2*ly, top-1-2*i);
                    uid++;
                }
                uid=0;
                for(int i=0;i<length_x;i++){
                    U.at(uid+length_y+prev) = make_pair(bottom+1+2*i, top);
                    U.at(uid+2*length_y+length_x+prev) = make_pair(grid_x-1+2*ly-1-2*i,bottom);
                    uid++;
                }
                uid = 0;
                bottom -=2;
                top +=2;
            }
        }
    }
    int t = Layer*2;
    for(int i=0;i<U.size();i++){
        U.at(i).first +=t;
        U.at(i).second +=t;
    }
}

void Feasible::FindClosestU(){
    /* brute force */
    for(int i:move_order){
        int shortest = -1;
        int sh = -1;
        for(int s=0;s<U.size();s++){
            if(F[s]==1){
                int len = abs(start_x[i]-U[s].first)+abs(start_y[i]-U[s].second); //SA2
                //len += abs(target_x[i]-U[s].first)+abs(target_y[i]-U[s].second); // len = d(s,m)+d(m,t)
                if(len < shortest and F[s]==true or shortest == -1){
                    shortest = len;
                    sh = s;
                }
            }
        }
        dg[i]->u = U[sh].first;
        //dg[i]->v = U[sh].second;
        dg[i]->v = gy1-U[sh].second;
        F[sh] = false;
    }
}

void Feasible::ReadData(){
    cout << "----------------------- ReadData -------------------------" << endl;
    Json::Value root;
    short tx = 2;
    short ty = 2;
    int gx = 3;
    int gy = 3;
    Json::Reader reader;

    string path =input;
    //path = "/home/hyeyun/cgshop2021/twostep/Result/cgshop_2021_instances_01/instances_01/"+input+".instance.json";
    //path = "./instance_/"+input+".instance.json";
    ifstream json(path,ifstream::binary);
    reader.parse(json, root);

    Json::Value meta = root["meta"];
    num_rb = meta["number_of_robots"].asInt();
    shape = meta["description"]["parameters"]["shape"][0].asInt();

    int min_x=10000000;
    int min_y=10000000;
    int max_x=0;
    int max_y=0;

    clock_t clt = (int)clock();
    start_x.assign(num_rb,0);
    start_y.assign(num_rb,0);
    int count=0;
    for(auto& value:root["starts"]){
        start_x.at(count)=value[0].asInt();
        start_y.at(count)=value[1].asInt();
        min_x = min(min_x, start_x[count]);
        min_y = min(min_y, start_y[count]);
        max_x = max(max_x, start_x[count]);
        max_y = max(max_y, start_y[count]);
        count++;
    }
    target_x.assign(num_rb,0);
    target_y.assign(num_rb,0);
    count=0;
    for(auto& value:root["targets"]){
        target_x.at(count)=value[0].asInt();
        target_y.at(count)=value[1].asInt();
        min_x = min(min_x, target_x[count]);
        min_y = min(min_y, target_y[count]);
        max_x = max(max_x, target_x[count]);
        max_y = max(max_y, target_y[count]);
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
            min_x = min(min_x, obstacle_x[count]);
            min_y = min(min_y, obstacle_y[count]);
            max_x = max(max_x, obstacle_x[count]);
            max_y = max(max_y, obstacle_y[count]);
            count++;
        }
    }
    printf("reading data takes %0.4f seconds\n",(float)(clock()-clt)/CLOCKS_PER_SEC);

    if(min_x!=0){
        for(int i=0;i<num_rb;i++){
            start_x[i] -=min_x;
            target_x[i] -=min_x;
        }
        if(num_ob!=0){
            for(int i=0;i<num_ob;i++){
                obstacle_x[i] -=min_x;
            }
        }
    }
    if(min_y!=0){
        for(int i=0;i<num_rb;i++){
            start_y[i] -=min_y;
            target_y[i] -=min_y;
        }
        if(num_ob!=0){
            for(int i=0;i<num_ob;i++){
                obstacle_y[i] -=min_y;
            }
        }
    }

    grid_x = max_x;
    grid_y = max_y;

    if(shape!=0){
        UF();
        cout << "Layer = "<< Layer<<", tx = "<< Layer*2 <<", ty = "<< Layer*2 <<endl;
        tx = Layer*2;
        ty = Layer*2;
        for(int i=0;i<num_rb;i++){
            start_x[i] +=tx;
            start_y[i] +=ty;
            target_x[i] +=tx;
            target_y[i] +=ty;
        }
        if(num_ob !=0){
            for(int i=0;i<num_ob;i++){
                obstacle_x[i] +=tx;
                obstacle_y[i] +=ty;
            }
        }
        max_x +=tx;
        max_y +=ty;
        grid_x = max_x;
        grid_y = max_y;

        //gx = tx+1; /* Since we count from zero, we need to add 1*/
        //gy = ty+1;
        grid_x += tx+1;
        grid_y += ty+1;

        if(grid_x!=grid_y){
            if(grid_x < grid_y){
                if(grid_y%2==0){
                    grid_x = grid_y;
                }
                else{
                    grid_y++;
                    grid_x = grid_y;
                }
            }
            else{
                if(grid_x%2==0){
                    grid_y = grid_x;
                }
                else{
                    grid_x++;
                    grid_y = grid_x;
                }
            }
        }
    }
    if(shape==0){
        UF2();
        cout << "Layer = "<< Layer<<", tx = "<< Layer*2 <<", ty = "<< Layer*2 <<endl;
        tx = Layer*2;
        ty = Layer*2;
        for(int i=0;i<num_rb;i++){
            start_x[i] +=tx;
            start_y[i] +=ty;
            target_x[i] +=tx;
            target_y[i] +=ty;
        }
        if(num_ob !=0){
            for(int i=0;i<num_ob;i++){
                obstacle_x[i] +=tx;
                obstacle_y[i] +=ty;
            }
        }
        max_x +=tx;
        max_y +=ty;
        grid_x = max_x;
        grid_y = max_y;
        grid_x +=Layer*2+1;
        grid_y +=Layer*2+1;
    }
    gy1 = grid_y-1;

    cout <<"grid_x = "<< grid_x<<", grid_y = "<< grid_y<<endl;
    printf("\nDATA----------------\n");
    printf("    %d X %d grid\n", grid_x, grid_y);
    printf("    %d robots\n", num_rb);
    printf("    %d obstacles\n", num_ob);
    printf("--------------------\n");

    GridInitialize();
    MoveOrder();
    FindClosestU();
}


void Feasible::WriteFile(){                // ANTOINE: write using the 3D Grid
    vector<int> rx(num_rb,0);
    vector<int> ry(num_rb,0);
    for(int i=0; i<num_rb;i++){
        rx[i]=start_x[i];
        ry[i]=start_y[i];
    }
    Achieved_sum=0;
    char d;
    for(int t=0;t<Grid3D.size()-1;t++){
        int flag=0;
        for(int i=0;i<num_rb;i++){
            d=Grid3D[t][rx[i]][ry[i]];
            rx[i]=Move_x(rx[i],d);
            ry[i]=Move_y(ry[i],d);
            if (d==5){
                continue;
            }
            if(d!=0){
                if(d==1){
                    Achieved_sum++;
                }
                else if(d==2){
                    Achieved_sum++;
                }
                else if(d==3){
                    Achieved_sum++;
                }
                else if(d==4){
                    Achieved_sum++;
                }
            }
        }
    }
    Achieved_ms = Grid3D.size()-1;
    //ofstream solution("./output_solution_/solution_"+output+"_"+to_string(Achieved_sum)+"_"+to_string(Achieved_ms)+".json");
    //ofstream solution(Objective+"_solution_"+output+"_"+to_string(Achieved_sum)+"_"+to_string(Achieved_ms)+".json");
    string filename;
    if(Objective=="MAX")
        filename = Objective+"_"+to_string(Achieved_ms)+".json";
    else if(Objective=="SUM")
        filename = Objective+"_"+to_string(Achieved_sum)+".json";

    ofstream solution(filename);

	string instance_name;
	instance_name = input.substr(0, input.size() - 14);

    solution<<"{\n";
    solution<<"\"instance\": \""<< instance_name <<"\",\n";
    solution<<"\"steps\" :[\n";

    for(int i=0; i<num_rb;i++){
        rx[i]=start_x[i];
        ry[i]=start_y[i];
    }
    for(int t=0;t<Grid3D.size()-1;t++){
        solution<<"{";
        int flag=0;
        for(int i=0;i<num_rb;i++){
            d=Grid3D[t][rx[i]][ry[i]];
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
        if (t<Grid3D.size()-2){
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

void Feasible::WriteVisual(){            // ANTOINE: Write with simultaneous movement
    ofstream result("./visual_main_/main_"+output+"_"+to_string(Achieved_sum)+"_"+to_string(Achieved_ms)+".js");
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

    vector<int> rx(num_rb,0);                                     //// ANTOINE: start
    vector<int> ry(num_rb,0);
    for(int r=0;r<num_rb;r++){
        rx[r]=start_x[r];
        ry[r]=start_y[r];
    }
    for(int t=0;t<Grid3D.size()-1;t++){
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


void Feasible::WriteScore(){
    int SumOfAllManhattan =0;
    int MaxManhattan=0;
    for(int soam:AllManhattan){
        if(MaxManhattan < soam){
            MaxManhattan = soam;
        }
        SumOfAllManhattan +=soam;
    }
    struct tm curr_tm;
    time_t curr_time = time(nullptr);
    localtime_r(&curr_time, &curr_tm);
    ofstream score("./score_/score_"+output+".txt", ios::app);
    score <<starttime<<endl;
    score <<" "<< elapse_t<<endl;
    score <<" "<<curr_tm.tm_mday<<"/"<<curr_tm.tm_mon+1<<"_";
    score <<curr_tm.tm_hour<<":"<<curr_tm.tm_min<<":"<<curr_tm.tm_sec;
    score <<" "<< output <<" "<<grid_x<<" "<< num_rb<<" "<<num_ob;
    score <<" "<< Achieved_sum;
    score <<" "<< Achieved_ms;
    score.close();
}

void Feasible::PrintGrid(vector<vector<pair<short,short>>> &G){
    for(int i=0;i<grid_y;i++){
        for(int j=0;j<grid_x;j++){
            cout <<" "<<setw(2) <<Value(G,j,i);
        }
        cout <<endl;
    }
}

void Feasible::PrintGrid3D(){
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
void Feasible::PrintGrid3D2(){
    cout << "-----------------------------------------------------------------------------" << endl;
    cout << "Grid3D2: number of slices = " << Grid3D2.size() << endl;
    cout << "-----------------------------------------------------------------------------" << endl;
    for(int t=0; t< Grid3D2.size(); t++){
        cout << "slice " << t << endl;
        for(int j=grid_y-1;j>=0;j--){
            for(int i=0;i<grid_x;i++){
                if(int(Grid3D2[t][i][j]==-1)){
                    cout <<" "<<setw(2)<<"-";
                }
                else{
                    cout <<" "<<setw(2) << int(Grid3D2[t][i][j]);
                }
            }
            cout <<endl;
        }
        cout << "-----------------------------------------------------------------------------" << endl;
    }
}
void Feasible::PrintGridWall(){
    for(int i=0;i<grid_x;i++){
        cout<<" "<<setw(2)<<Value(Dist_Grid,i,0);
    }
    cout <<endl;
    cout <<endl;
    for(int i=0;i<grid_x;i++){
        cout<<" "<<setw(2)<<Value(Dist_Grid,i,grid_y-1);
    }
    cout <<endl;
    cout <<endl;
    for(int i=0;i<grid_y;i++){
        cout<<" "<<setw(2)<<Value(Dist_Grid,0,i);
    }
    cout <<endl;
    cout <<endl;
    for(int i=0;i<grid_y;i++){
        cout<<" "<<setw(2)<<Value(Dist_Grid,grid_x-1,i);
    }
    cout <<endl;
    cout <<endl;
    cout <<endl;
}
