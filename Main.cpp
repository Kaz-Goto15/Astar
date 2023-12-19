#include "Astar.h"

int main() {
    vector<vector<int>> map = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 1, 1, 0} };
    Astar::POINT start = { 0, 2 };
    Astar::POINT end = { 4,4 };
//    vector<vector<int>> map = {
//{FLOOR,FLOOR,FLOOR,FLOOR, WALL,FLOOR,FLOOR,FLOOR,FLOOR,FLOOR, WALL,FLOOR,FLOOR},
//{FLOOR,FLOOR,FLOOR,FLOOR,FLOOR,FLOOR, WALL, WALL, WALL,FLOOR,FLOOR,FLOOR, WALL},
//{FLOOR, WALL,FLOOR,FLOOR, WALL,FLOOR,FLOOR,FLOOR, WALL,FLOOR, WALL,FLOOR, WALL},
//{FLOOR,FLOOR,FLOOR, WALL, WALL, WALL, WALL, WALL, WALL,FLOOR, WALL,FLOOR, WALL},
//{FLOOR, WALL,FLOOR,FLOOR, WALL,FLOOR,FLOOR,FLOOR,FLOOR,FLOOR, WALL,FLOOR, WALL},
//{FLOOR,FLOOR, WALL,FLOOR, WALL, WALL, WALL, WALL, WALL, WALL, WALL,FLOOR, WALL},
//{FLOOR,FLOOR, WALL,FLOOR, WALL,FLOOR,FLOOR, WALL,FLOOR,FLOOR, WALL,FLOOR, WALL},
//{FLOOR, WALL,FLOOR,FLOOR, WALL,FLOOR,FLOOR,FLOOR,FLOOR,FLOOR,FLOOR,FLOOR, WALL} };
//    Astar::POINT start = { 0, 7 };
//    Astar::POINT end = { 5,6 };
//    vector<vector<int>> map = {
//    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//    { 1,0,0,0,0,0,0,0,1,0,0,0,1 },
//    { 1,1,1,1,1,0,1,0,1,1,1,0,1 },
//    { 1,0,0,0,0,0,1,0,0,0,1,0,1 },
//    { 1,0,1,1,1,1,1,1,1,0,1,0,1 },
//    { 1,0,0,0,1,0,0,0,1,0,0,0,1 },
//    { 1,1,1,0,1,0,1,0,1,1,1,1,1 },
//    { 1,0,0,0,1,0,1,0,1,0,0,0,1 },
//    { 1,0,1,1,1,0,1,0,1,1,1,0,1 },
//    { 1,0,1,0,0,0,1,0,0,0,1,0,1 },
//    { 1,0,1,0,1,1,1,1,1,0,1,0,1 },
//    { 1,0,1,0,0,0,1,0,0,0,1,0,1 },
//    { 1,0,1,1,1,0,1,1,1,1,1,0,1 },
//    { 1,0,0,0,0,0,1,0,0,0,1,0,1 },
//    { 1,0,1,1,1,1,1,1,1,0,1,0,1 },
//    { 1,0,0,0,0,0,0,0,1,0,0,0,1 },
//    { 1,0,1,1,1,1,1,0,1,1,1,0,1 },
//    { 1,0,1,0,0,0,1,0,0,0,1,0,1 },
//    { 1,0,1,0,1,1,1,0,1,1,1,0,1 },
//    { 1,0,0,0,1,0,0,0,1,0,0,0,1 },
//    { 1,1,1,1,1,0,1,1,1,0,1,1,1 },
//    { 1,0,1,0,0,0,1,0,1,0,0,0,1 },
//    { 1,0,1,0,1,0,1,0,1,1,1,0,1 },
//    { 1,0,0,0,1,0,0,0,0,0,0,0,1 },
//    { 1,1,1,1,1,1,1,1,1,1,1,1,1 }
//};
//    Astar::POINT start = { 1, 1 };
//    Astar::POINT end = { 11,23 };
    Astar astar;
    astar.Init(map, start, end, true);
    astar.IsDebugMsg(false);
    astar.Run();

    astar.DrawInfoTable();
    astar.DrawMap();
    astar.DrawPath();
	return 0;
}

//////////////////////////////////////////////////////
// https://qiita.com/2dgames_jp/items/f29e915357c1decbc4b7
// https://stone-program.com/python/algorithm/a-star-introduction/#
// https://ja.wikipedia.org/wiki/A*#é¿ç€Ç…égÇÌÇÍÇƒÇ¢ÇÈOPEN/CLOSEÉäÉXÉgÇÃé¿ëï
//////////////////////////////////////////////////////