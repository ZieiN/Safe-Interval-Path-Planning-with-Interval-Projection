#include <iostream>
#include <set>
#include <vector>
#include <chrono>
#include <fstream>
#include <unordered_set>

using namespace std;
using namespace std::chrono;
#define INF 1000000
#define MXO 4 // number of different orientation. 0 is the East, then rotate with counterclockwise
#define MXV 3 // number of different velocities starting from 0 i.e. 0, 1, 2, ...
#define F 12 // number of timesteps in one second i.e. 1/T

string map = "room";
#define MXH 64 // maximum height of the environment
#define MXW 64 // maximum width of the environment

// string map = "arena";
// #define MXH 49 // maximum height of the environment
// #define MXW 49 // maximum width of the environment

// string map = "warehouse";
// #define MXH 84 // maximum height of the environment
// #define MXW 170 // maximum width of the environment


class Bot{
public:
    int x, y, o, v, t;
    Bot(int x, int y, int o, int v, int t):x(x),y(y),o(o),v(v),t(t){}
    Bot(){
        x = y = o = v = t = 0;
    }
    Bot(const Bot&);
    Bot &operator=(const Bot&);
    friend bool operator == (const Bot& a, const Bot& b);
    friend bool operator < (const Bot& a, const Bot& b);
};

int end_x, end_y, end_v = 0;
bool operator < (const Bot& a, const Bot& b){
    if(a.t+abs(a.x-end_x)+abs(a.y-end_y) != b.t+abs(b.x-end_x)+abs(b.y-end_y)){
        return a.t+abs(a.x-end_x)+abs(a.y-end_y) < b.t+abs(b.x-end_x)+abs(b.y-end_y);
    }
    if(abs(a.x-end_x)+abs(a.y-end_y) != abs(b.x-end_x)+abs(b.y-end_y)){
        return abs(a.x-end_x)+abs(a.y-end_y) < abs(b.x-end_x)+abs(b.y-end_y);
    }

    return ((a.x*MXW+a.y)*4+a.o)*MXV+a.v
    < ((b.x*MXW+b.y)*4+b.o)*MXV+b.v;
}
bool operator == (const Bot& a, const Bot& b){
    return a.x == b.x && a.y == b.y && a.o == b.o && a.v == b.v && a.t == b.t;
}
Bot::Bot(const Bot &other){
    x=other.x;y=other.y;o=other.o;v=other.v;t=other.t;
}
struct Bot_hash {
    long long operator()(Bot const &bt) const {
        return (long long)(((bt.x*MXW+bt.y)*4+bt.o)*MXV+bt.v)*INF+bt.t;
    }
};
Bot &Bot::operator=(const Bot &other) = default;

class Action{
public:
    int dx, dy, o, v, cost;
    Action(){
        dx = dy = o = v = cost = 0;
    }
    Action(int dx, int dy, int o, int v, int cost):dx(dx),dy(dy),o(o),v(v),cost(cost){}
};

int H, W;
set<pair<int, int>> rsrv_tbl[MXH][MXW];
vector<Action> actions[MXO][MXV];

set<Bot> OPEN;
unordered_set<Bot, Bot_hash> CLOSED;

bool outMap(int x, int y) {
    return x<0 || x>=H || y<0 || y>=W;
}

pair<int, bool> search(int st_x, int st_y, int st_o, int st_v, int st_t=0){
    auto it = --rsrv_tbl[st_x][st_y].lower_bound({st_t,INF});
    if(it->second > st_t){
        cerr<<"Error, the initial time falls in an obstacle!!"<<endl;
        exit(0);
    }
    CLOSED.clear();
    OPEN.clear();
    OPEN.insert(Bot(st_x, st_y, st_o, st_v, st_t));
    while(!OPEN.empty() && OPEN.size()+CLOSED.size()<1000000){
        auto tmp = *OPEN.begin();
        OPEN.erase(OPEN.begin());
        int x = tmp.x, y = tmp.y, o = tmp.o, v = tmp.v, t = tmp.t;
        CLOSED.insert(tmp);
        if(x == end_x && y == end_y && v == end_v){
            return {t, true};
        }
        int safe_interval_upper_bound = rsrv_tbl[x][y].lower_bound({t, INF})->first;
        if(v == 0 && t+1<safe_interval_upper_bound){
            OPEN.insert(Bot(x, y, o, v, t+1));
        }
        for(auto a:actions[o][v]){
            int new_x = x+a.dx, new_y = y+a.dy, new_o = a.o, new_v = a.v, new_t = t + a.cost;
            if(outMap(new_x, new_y)){
                continue;
            }
            if(new_t >= safe_interval_upper_bound)
                continue;
            it = rsrv_tbl[new_x][new_y].lower_bound({new_t, INF});
            auto it_prev = it;
            --it_prev;
            if((new_t-a.cost > (it_prev->second)) && (new_t<(it->first))){
                auto succ = Bot(new_x, new_y, new_o, new_v, new_t);
                if(CLOSED.find(succ)==CLOSED.end())
                    OPEN.insert(succ);
            }
        }
    }
    return {INF, false};
}

void pre(){
    for(int i=0; i<MXH; ++i){
        for(int j=0; j<MXW; ++j){
            rsrv_tbl[i][j].clear();
            rsrv_tbl[i][j].insert({-1, -1});
            rsrv_tbl[i][j].insert({INF, INF});
        }
    }
}

int testNum = 0;
void readInputs(int &stx, int &sty, int &sto) {
    ifstream in;
    in.open("../"+map+".txt");
    in>>H>>W;
    for(int i=0; i<H; ++i){
        for(int j=0; j<W; ++j){
            bool tmp;
            in>>tmp;
            if(tmp)
                rsrv_tbl[i][j].insert({0,INF});
        }
    }
    in.close();
    in.open("../"+map+"/"+map+"-obs-"+ to_string(testNum)+".txt");
    string ignore_string;
    int x, y, t1, t2;
    in>>ignore_string>>ignore_string>>stx>>ignore_string>>sty>>ignore_string>>sto;
    in>>ignore_string>>ignore_string>>end_x>>ignore_string>>end_y;
    while(in>>ignore_string){
        in>>ignore_string>>x>>ignore_string>>y>>ignore_string>>t1>>ignore_string>>t2;
        rsrv_tbl[x][y].insert({t1,t2});
    }
    in.close();
}

void fillActions() {
    // turn actions
    for(int o=0; o<MXO; ++o){
        actions[o][0].emplace_back(0, 0, (o+1)%4, 0, 1);
    }
    int dx[4] = {0,-1,0,1}, dy[4]={1,0,-1,0};
    for(int o=0; o<MXO; ++o){
        for(int v1=0; v1<MXV; ++v1){
            for(int v2=0; v2<MXV; ++v2){
                if(v1==0 && v2==0)
                    continue;
                int cost = 2*F/(v1+v2);
                actions[o][v1].emplace_back(dx[o], dy[o], o, v2, cost);
            }
        }
    }
}
int main() {
    ofstream out;
    fillActions();
    // for(int numOfObs = 100; numOfObs<=500; numOfObs+=100) {
        out.open("../res-" + map + "-Astar.txt");
        for (testNum = 0; testNum < 500; ++testNum) {
            pre();
            int stx, sty, sto;
            end_v = 0;
            readInputs(stx, sty, sto);
            auto start_time = high_resolution_clock::now();
            auto ans = search(stx, sty, sto, 0, 0);
            int diffTime = duration_cast<microseconds>(high_resolution_clock::now() - start_time).count();
            out << ans.first << " " << ans.second << " " << diffTime << endl;
        }
        out.close();
    // }
    return 0;
}

