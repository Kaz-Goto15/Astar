#pragma once
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <Windows.h>
class Astar
{
private:
	struct NODE {
		NODE* parent = nullptr;
		POINT position = { -1, -1 };
		int score = 0;		//合計コスト(移動距離)
		int cost = 0;		//スタートから現時点までの距離
		int heuristic = 0;	//ゴールまでの推定値
		int& f = score;		//合計コスト(移動距離)
		int& g = cost;		//スタートから現時点までの距離
		int& h = heuristic;	//ゴールまでの推定値

		NODE operator = (const NODE& node) {
			this->parent = node.parent;
			this->position = node.position;
			this->f = node.f;
			this->g = node.g;
			this->h = node.h;
		}
		bool operator == (const NODE& node) {
			return (this->position.x == node.position.x && 
				this->position.y == node.position.y);
		}
	};
	void Result();
	string pathStr;	//パスをstringで記録
	vector<vector<int>> map;
	POINT startPt, endPt;
	bool isGoal_;
	bool enDiagonal = false;
public:
	Astar();
	~Astar();
	void Init(vector<vector<int>> map, POINT s, POINT e);
	void Run();
	string GetPathStr() { return pathStr; }
};

