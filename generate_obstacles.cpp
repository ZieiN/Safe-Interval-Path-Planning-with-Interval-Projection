//
// Created by **** on 10.12.2021.
//
#include <bits/stdc++.h>
#define MX_HEIGHT 32
#define MX_WIDTH 32
#define MXT 5000
using namespace  std;
ofstream out;
ifstream in;
set<pair<int, int>> st[MX_HEIGHT][MX_WIDTH];
set<int> st1[MX_HEIGHT][MX_WIDTH];
bool mp[MX_HEIGHT][MX_WIDTH], rsrv[MX_HEIGHT][MX_WIDTH][MXT];
int H, W;


void clr();


void generate_start_end_points() {
// Random var
    random_device rd;
    mt19937 mt(rd());
    uniform_int_distribution<int> randHeight(0, H-1);
    uniform_int_distribution<int> randWidth(0, W-1);
    uniform_int_distribution<int> randOrientation(0, 3);
    int x = randHeight(mt), y = randWidth(mt), o = randOrientation(mt);
    while(mp[x][y]){
        x = randHeight(mt);
        y = randWidth(mt);
    }
    out<<"start_cell: x= "<<x<<" y= "<<y<<" orientation= "<<o<<endl;
    mp[x][y] = true;
    while(mp[x][y]){
        x = randHeight(mt);
        y = randWidth(mt);
    }
    out<<"goal_cell: x= "<<x<<" y= "<<y<<endl;
    mp[x][y] = true;
}

void generate_obstacle(){

// Random var
    random_device rd;
    mt19937 mt(rd());
    uniform_int_distribution<int> randHeight(0, H-1);
    uniform_int_distribution<int> randWidth(0, W-1);
    uniform_int_distribution<int> randOrientation(0, 3);
    uniform_int_distribution<int> randWait(1, 500);
    uniform_int_distribution<int> randWaitLong(1, MXT);

    int x = randHeight(mt), y = randWidth(mt), o = randOrientation(mt);
    while(mp[x][y]){
        x = randHeight(mt);
        y = randWidth(mt);
    }
    int dx[4] = {0,-1,0,1}, dy[4]={1,0,-1,0};
    int t = 0;
    while(rsrv[x][y][t]||rsrv[x][y][t+1]){
        ++t;
        if(t==MXT-1){
            x = randHeight(mt);
            y = randWidth(mt);
            while(mp[x][y]){
                x = randHeight(mt);
                y = randWidth(mt);
            }
            t = 0;
        }
    }
    ++t;
    while(x>=0 && x<H && y>=0 && y<W){
        int xx = x+dx[o];
        int yy = y+dy[o];
        vector<int> v;
        if(xx<0 || xx>=H || yy<0 || yy>=W || mp[xx][yy]) {
            int wait = randWaitLong(mt);
            for(int i=t; i<MXT && v.size()<wait; ++i){
                if(rsrv[x][y][i])
                    break;
                v.push_back(i);
            }
            wait = v.back();
            for(int i=t-1; i<=wait; ++i){
                assert(rsrv[x][y][i]==0);
                rsrv[x][y][i]=1;
            }
            out<<"reserved_interval_at_cell: x= "<<x<<" y= "<<y<<" from_timestep: "<<t-1<<" to_timestep: "<<wait<<endl;
//            st[x][y].insert({t,wait});
            break;
        }

        int wait = randWait(mt);
        for(int i=t+1; i<MXT && v.size()<wait; ++i){
            if(rsrv[x][y][i])
                break;
            if(rsrv[xx][yy][i]==0 && rsrv[xx][yy][i-1]==0){
                v.push_back(i);
            }
        }
        if(v.size()==0) {
            wait = randWaitLong(mt);
            for(int i=t; i<MXT && v.size()<wait; ++i){
                if(rsrv[x][y][i])
                    break;
                v.push_back(i);
            }
            wait = v.back();
            for(int i=t-1; i<=wait; ++i){
                assert(rsrv[x][y][i]==0);
                rsrv[x][y][i]=1;
            }
            out<<"reserved_interval_at_cell: x= "<<x<<" y= "<<y<<" from_timestep: "<<t-1<<" to_timestep: "<<wait<<endl;
            break;
        }
        wait = v.back();
        out<<"reserved_interval_at_cell: x= "<<x<<" y= "<<y<<" from_timestep: "<<t-1<<" to_timestep: "<<wait<<endl;
//        out<<"e "<<x<<" "<<y<<" "<<wait<<" "<<o<<endl;
//        st[x][y].insert({t,wait});
//        st1[x][y].insert(t);
        for(int i=t-1; i<=wait; ++i){
            assert(rsrv[x][y][i]==0);
            rsrv[x][y][i]=true;
        }
        t = wait;
        x += dx[o];
        y += dy[o];
    }
}
int main(){
    ios::sync_with_stdio(0);
    string map = "32x32_20";
//    string map = "arena";
//    string map = "warehouse";
    for(int i=0; i<500; ++i) {

        in.open("../"+map+".txt");
        in>>H>>W;
        for(int i=0; i<H; ++i){
            for(int j=0; j<W; ++j){
                in>>mp[i][j];
            }
        }
        in.close();

        clr();
        out.open("../"+map+"/"+map+"-obs-"+to_string(i)+".txt");
        generate_start_end_points();
        int cnt = 500;
        while (cnt--) {
            generate_obstacle();
        }
        out.close();
    }
}



void clr() {
    for(int i=0; i<H; ++i){
        for(int j=0; j<W; ++j){
            for(int k=0; k<MXT; ++k){
                rsrv[i][j][k] = 0;
            }
        }
    }
}
