#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#endif

// C++17: two-ant Langton's Ant highway/loop analyzer.
// Build: g++ -std=c++17 -O2 main.exe.cpp -o langtons_ant_analyzer
// Run:   ./langtons_ant_analyzer --visual --max-steps 300000
// Directions: up, right, down, left
namespace {
constexpr int DX[4]={0,1,0,-1}, DY[4]={-1,0,1,0};
constexpr const char* DIR_JA[4]={"上","右","下","左"};
constexpr uint64_t GOLD=0x9E3779B97F4A7C15ULL, SA=0x243F6A8885A308D3ULL, SB=0x13198A2E03707344ULL;
uint64_t mix(uint64_t z){z+=GOLD;z=(z^(z>>30))*0xBF58476D1CE4E5B9ULL;z=(z^(z>>27))*0x94D049BB133111EBULL;return z^(z>>31);}
uint64_t rotl(uint64_t x,int r){return (x<<r)|(x>>(64-r));}
struct Pos{int64_t x=0,y=0; bool operator==(const Pos&o)const{return x==o.x&&y==o.y;}};
struct PosHash{size_t operator()(const Pos&p)const{return static_cast<size_t>(mix(static_cast<uint64_t>(p.x)^rotl(mix(static_cast<uint64_t>(p.y)^SB),21)^SA));}};
std::pair<uint64_t,uint64_t> signature(int64_t x,int64_t y){uint64_t a=static_cast<uint64_t>(x),b=static_cast<uint64_t>(y);return {mix(mix(a^SA)^rotl(mix(b^SB),17)^SA), mix(mix(a^SB)^rotl(mix(b^SA),31)^SB)};}
struct Ant{Pos p;int dir=0;};
struct Key{int64_t x,y,rx,ry;int8_t d1,d2;uint64_t ha,hb;bool operator==(const Key&o)const{return x==o.x&&y==o.y&&rx==o.rx&&ry==o.ry&&d1==o.d1&&d2==o.d2&&ha==o.ha&&hb==o.hb;}};
struct KeyHash{size_t operator()(const Key&k)const{uint64_t h=mix(k.x)^rotl(mix(k.y),7)^rotl(mix(k.rx),13)^rotl(mix(k.ry),19)^rotl(mix(uint64_t(k.d1)|(uint64_t(k.d2)<<8)),29)^k.ha^rotl(k.hb,37);return static_cast<size_t>(h);}};
struct PairPos{Pos a,b;};
struct Segment{uint64_t start=0,confirmed=0;bool hasDelta=false;int64_t dx=0,dy=0;};
struct Opt{uint64_t maxSteps=200000,progress=10000,visualEvery=100;int64_t spacing=0;int d1=0,d2=0;int period=104,cycles=20,visualDelay=50;bool visual=false;std::string output="langtons_ant_report.json";};
bool dir(const std::string&s,int&d){if(s=="up")d=0;else if(s=="right")d=1;else if(s=="down")d=2;else if(s=="left")d=3;else return false;return true;}
template<class T> bool number(const std::string&s,T&v){try{size_t n=0;auto x=std::stoll(s,&n);if(n!=s.size())return false;v=static_cast<T>(x);return true;}catch(...){return false;}}
bool parse(int argc,char**argv,Opt&o){for(int i=1;i<argc;++i){std::string a=argv[i];auto val=[&](){if(i+1>=argc){std::cerr<<"値が必要です: "<<a<<"\n";std::exit(2);}return std::string(argv[++i]);};if(a=="--max-steps"){if(!number(val(),o.maxSteps)||!o.maxSteps)return false;}else if(a=="--spacing"){if(!number(val(),o.spacing)||o.spacing<0)return false;}else if(a=="--ant1-dir"){if(!dir(val(),o.d1))return false;}else if(a=="--ant2-dir"){if(!dir(val(),o.d2))return false;}else if(a=="--period"){if(!number(val(),o.period)||o.period<1)return false;}else if(a=="--cycles"){if(!number(val(),o.cycles)||o.cycles<2)return false;}else if(a=="--progress-every"){if(!number(val(),o.progress))return false;}else if(a=="--visual")o.visual=true;else if(a=="--visual-every"){if(!number(val(),o.visualEvery)||!o.visualEvery)return false;}else if(a=="--visual-delay"){if(!number(val(),o.visualDelay)||o.visualDelay<0)return false;}else if(a=="--output")o.output=val();else if(a=="--help"||a=="-h"){std::cout<<"--max-steps N --spacing N --ant1-dir up|right|down|left --ant2-dir up|right|down|left --period N --cycles N --progress-every N --output FILE --visual [--visual-every N] [--visual-delay MS]\n  --visual             アリの動きをアニメーション表示\n  --visual-every N     何歩ごとに画面を更新するか (既定: 100)\n  --visual-delay MS    画面更新ごとの待ち時間 (既定: 50)\n";std::exit(0);}else{std::cerr<<"不明な引数: "<<a<<"\n";return false;}}return true;}
std::string esc(const std::string&s){std::string o;for(char c:s){if(c=='"'||c=='\\')o+='\\';if(c=='\n'){o+="\\n";continue;}o+=c;}return o;}
class Analyzer{
public:
 explicit Analyzer(Opt o):opt(o){int64_t dist=opt.spacing+1,x1=-(dist/2);ants[0]={{x1,0},opt.d1};ants[1]={{x1+dist,0},opt.d2};visited.insert(ants[0].p);visited.insert(ants[1].p);windowMax=static_cast<size_t>(opt.period)*opt.cycles;history[0].reserve(std::min<uint64_t>(opt.maxSteps+1,5000000));history[1].reserve(std::min<uint64_t>(opt.maxSteps+1,5000000));seen.emplace(key(),0);}
 void run(){std::cerr<<"解析開始：最大歩数="<<opt.maxSteps<<"、間隔="<<opt.spacing<<"、周期="<<opt.period<<"、確認回数="<<opt.cycles<<"\n";if(opt.visual)render();while(steps<opt.maxSteps){step();if(loop){status="loop_detected";break;}if(active[0]&&active[1]){status="both_highways";break;}if(opt.visual&&steps%opt.visualEvery==0)render();if(!opt.visual&&opt.progress&&steps%opt.progress==0)std::cerr<<steps<<"歩 黒="<<black.size()<<" 通過="<<visited.size()<<"\n";}if(status.empty())status="step_limit";if(opt.visual){render();std::cout<<"\nアニメーション終了。Enter キーを押すと解析結果を表示します...";std::cin.get();}}
 void report(){auto hu=highway[0];hu.insert(highway[1].begin(),highway[1].end());size_t outside=0;for(auto&p:black)if(!hu.count(p))++outside;std::cout<<"\n=== 解析結果 ===\n判定: "<<status<<"\n歩数: "<<steps<<"\n";for(int i=0;i<2;++i){std::cout<<"\nアリ"<<i+1<<" 最終位置=("<<ants[i].p.x<<","<<ants[i].p.y<<") 向き="<<DIR_JA[ants[i].dir]<<"\n";if(segments[i].empty())std::cout<<"  ハイウェイ未確認\n";else{auto&s=segments[i][0];std::cout<<"  周期開始歩数="<<s.start<<" 確認完了="<<s.confirmed<<"\n";if(s.hasDelta)std::cout<<"  104歩変位=("<<s.dx<<","<<s.dy<<")\n";}}if(loop)std::cout<<"\nループ: 初回="<<loopAt<<" 再出現="<<steps<<" 周期="<<steps-loopAt<<"\n";else std::cout<<"\nループ未検出\n";std::cout<<"\n現在の黒マス数="<<black.size()<<"\nハイウェイ経路マス数="<<hu.size()<<"\nハイウェイ経路外の黒マス数="<<outside<<"\n一度でも通ったマス数="<<visited.size()<<"\n";}
 bool save(){auto hu=highway[0];hu.insert(highway[1].begin(),highway[1].end());size_t outside=0;for(auto&p:black)if(!hu.count(p))++outside;std::ofstream f(opt.output);if(!f)return false;f<<"{\n  \"status\": \""<<esc(status)<<"\",\n  \"steps_simulated\": "<<steps<<",\n  \"settings\": {\"spacing_empty_cells\": "<<opt.spacing<<", \"highway_period_steps\": "<<opt.period<<", \"required_consecutive_cycles\": "<<opt.cycles<<", \"maximum_steps\": "<<opt.maxSteps<<", \"simultaneous_processing\": true},\n  \"ants\": [\n";for(int i=0;i<2;++i){if(i)f<<",\n";f<<"    {\"ant\": "<<i+1<<", \"color\": \""<<(i?"blue":"red")<<"\", \"position_xy\": ["<<ants[i].p.x<<","<<ants[i].p.y<<"], \"direction_number\": "<<ants[i].dir<<", \"direction_japanese\": \""<<DIR_JA[ants[i].dir]<<"\", \"highway_active_at_end\": "<<(active[i]?"true":"false")<<", \"highway_cells_marked\": "<<highway[i].size()<<", \"highway_segments\": [";for(size_t j=0;j<segments[i].size();++j){auto&s=segments[i][j];if(j)f<<",";f<<"{\"start_step\":"<<s.start<<",\"confirmed_at_step\":"<<s.confirmed<<",\"period\":"<<opt.period<<",\"confirmed_cycles\":"<<opt.cycles<<",\"displacement_per_period_xy\":";if(s.hasDelta)f<<"["<<s.dx<<","<<s.dy<<"]";else f<<"null";f<<"}";}f<<"]}";}f<<"\n  ],\n  \"loop\": ";if(loop)f<<"{\"first_seen_step\":"<<loopAt<<",\"repeated_at_step\":"<<steps<<",\"loop_period_steps\":"<<steps-loopAt<<"}";else f<<"null";f<<",\n  \"statistics\": {\"black_cells_remaining\":"<<black.size()<<",\"highway_cells_marked_union\":"<<hu.size()<<",\"black_cells_outside_detected_highway\":"<<outside<<",\"ever_visited_cells\":"<<visited.size()<<"}\n}\n";return bool(f);}
private:
 Opt opt;std::array<Ant,2>ants;std::unordered_set<Pos,PosHash>black,visited;uint64_t ha=0,hb=0,steps=0;std::string status;std::array<std::vector<int>,2>history;std::array<int,2>matches{0,0};std::array<bool,2>active{false,false};std::array<std::vector<Segment>,2>segments;std::array<std::unordered_set<Pos,PosHash>,2>highway;std::deque<PairPos>window;size_t windowMax=0;std::unordered_map<Key,uint64_t,KeyHash>seen;bool loop=false;uint64_t loopAt=0;
 void render(){constexpr int width=41,height=21;const char*arrows[4]={"^",">","v","<"};
#ifdef _WIN32
  HANDLE console=GetStdHandle(STD_OUTPUT_HANDLE);DWORD mode=0;if(console!=INVALID_HANDLE_VALUE&&GetConsoleMode(console,&mode)&&!SetConsoleMode(console,mode|ENABLE_VIRTUAL_TERMINAL_PROCESSING))throw std::runtime_error("ターミナルの ANSI 表示を有効にできませんでした。");
#endif
  std::cout<<"\x1b[2J\x1b[H"<<"ラングトンのアリ（二匹）  歩数: "<<steps<<" / "<<opt.maxSteps<<"\n"
           <<"アリ1: "<<ants[0].p.x<<","<<ants[0].p.y<<" 向き "<<DIR_JA[ants[0].dir]<<"  "
           <<"アリ2: "<<ants[1].p.x<<","<<ants[1].p.y<<" 向き "<<DIR_JA[ants[1].dir]<<"\n"
           <<"黒マス: "<<black.size()<<"   通過マス: "<<visited.size()<<"   終了: "<<(status.empty()?"実行中":status)<<"\n"
           <<"アリ1: 赤  アリ2: 青  黒マス: ##\n";
  int64_t centerX=(ants[0].p.x+ants[1].p.x)/2,centerY=(ants[0].p.y+ants[1].p.y)/2;
  for(int y=0;y<height;++y){for(int x=0;x<width;++x){Pos p{centerX+x-width/2,centerY+y-height/2};int ant=-1;for(int i=0;i<2;++i)if(ants[i].p==p){ant=i;break;}if(ant>=0)std::cout<<(ant==0?"\x1b[31m":"\x1b[34m")<<arrows[ants[ant].dir]<<"\x1b[0m ";else std::cout<<(isBlack(p)?"##":"  ");}std::cout<<"\n";}
  std::cout<<std::flush;if(opt.visualDelay)std::this_thread::sleep_for(std::chrono::milliseconds(opt.visualDelay));}
 Key key()const{return {ants[0].p.x,ants[0].p.y,ants[1].p.x-ants[0].p.x,ants[1].p.y-ants[0].p.y,static_cast<int8_t>(ants[0].dir),static_cast<int8_t>(ants[1].dir),ha,hb};}
 bool isBlack(Pos p)const{return black.count(p)!=0;}
 void setBlack(Pos p,bool b){bool old=isBlack(p);if(old==b)return;auto s=signature(p.x,p.y);ha^=s.first;hb^=s.second;if(b)black.insert(p);else black.erase(p);}
 void markSegment(int i,uint64_t start){Segment s;s.start=start;s.confirmed=steps;uint64_t base=steps-window.size();size_t idx=static_cast<size_t>(start>base?start-base:0);if(idx>=window.size())idx=window.empty()?0:window.size()-1;if(!window.empty()){for(size_t j=idx;j<window.size();++j){Pos p=i==0?window[j].a:window[j].b;highway[i].insert(p);}size_t end=idx+static_cast<size_t>(opt.period);if(end<window.size()){Pos p0=i==0?window[idx].a:window[idx].b;Pos p1=i==0?window[end].a:window[end].b;s.dx=p1.x-p0.x;s.dy=p1.y-p0.y;s.hasDelta=true;}}segments[i].push_back(s);}
 void step(){std::array<Pos,2>src{ants[0].p,ants[1].p};window.push_back({src[0],src[1]});while(window.size()>windowMax)window.pop_front();for(auto p:src)visited.insert(p);for(int i=0;i<2;++i)if(active[i])highway[i].insert(src[i]);struct Op{Pos p;int d;bool black;};std::array<Op,2>ops;for(int i=0;i<2;++i){bool b=isBlack(src[i]);ops[i]={src[i],b?(ants[i].dir+3)%4:(ants[i].dir+1)%4,!b};}for(int i=0;i<2;++i){setBlack(ops[i].p,ops[i].black);ants[i].dir=ops[i].d;}for(int i=0;i<2;++i){ants[i].p.x+=DX[ants[i].dir];ants[i].p.y+=DY[ants[i].dir];visited.insert(ants[i].p);}++steps;std::array<bool,2>now{false,false};for(int i=0;i<2;++i){history[i].push_back(ants[i].dir);size_t t=history[i].size()-1;if(t>=static_cast<size_t>(opt.period)&&history[i][t]==history[i][t-opt.period]){++matches[i];now[i]=true;}else matches[i]=0;if(active[i]&&!now[i])active[i]=false;}Key k=key();auto it=seen.find(k);if(it!=seen.end()){loop=true;loopAt=it->second;return;}seen.emplace(k,steps);int needed=opt.period*(opt.cycles-1);for(int i=0;i<2;++i)if(!active[i]&&matches[i]>=needed){int t=static_cast<int>(history[i].size())-1;int start=t-matches[i]+1-opt.period;markSegment(i,static_cast<uint64_t>(std::max(0,start)));active[i]=true;}}
};
}
int main(int argc,char**argv){Opt o;if(!parse(argc,argv,o)){std::cerr<<"引数が不正です。--help を確認してください。\n";return 2;}try{Analyzer a(o);a.run();a.report();if(a.save())std::cout<<"\nJSONレポート: "<<o.output<<"\n";else std::cerr<<"JSON保存失敗: "<<o.output<<"\n";}catch(const std::exception&e){std::cerr<<"エラー: "<<e.what()<<"\n";return 1;}return 0;}
