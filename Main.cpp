#include <iostream>
#include "Astar.h"
#include <Windows.h>
#include <vector>
using std::vector;
using std::cout;
using std::endl;

int main() {
    vector<vector<int>> map = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 1, 1, 0}};
    POINT start = { 2, 2 };
    POINT end = { 4,4 };
    Astar astar;
    astar.Init(map, start, end);
    astar.Run();
    cout << astar.GetPathStr() << endl;
	return 0;
}

//////////////////////////////////////////////////////
// https://qiita.com/2dgames_jp/items/f29e915357c1decbc4b7
// https://stone-program.com/python/algorithm/a-star-introduction/#
// https://ja.wikipedia.org/wiki/A*#ŽÀÛ‚ÉŽg‚í‚ê‚Ä‚¢‚éOPEN/CLOSEƒŠƒXƒg‚ÌŽÀ‘•
//////////////////////////////////////////////////////