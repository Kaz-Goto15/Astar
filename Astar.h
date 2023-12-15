#pragma once
#include <string>
using std::string;
#include <vector>
using std::vector;
//#include <Windows.h>
class Astar
{
public:
	typedef struct POINT {
		int x = 0;
		int y = 0;

		void Set(int x, int y) {
			this->x = x;
			this->y = y;
		}
		void Set(POINT pts) {
			Set(pts.x, pts.y);
		}
		POINT GetPoint() { return *this; }
		bool operator == (const POINT pts) const {
			return (x == pts.x && y == pts.y);
		}
		void operator = (const POINT pts) {
			this->x = pts.x;
			this->y = pts.y;
		}
		POINT operator + (const POINT& pts) {
			POINT ret;
			ret.x = x + pts.x;
			ret.y = y + pts.y;
			return ret;
		}
		POINT operator - (const POINT& pts) {
			POINT ret;
			ret.x = x - pts.x;
			ret.y = y - pts.y;
			return ret;
		}
		POINT operator += (const POINT& pts) {
			return (*this + pts);
		}
		POINT operator -= (const POINT& pts) {
			return (*this - pts);
		}

	};

private:


	enum DIRECTION {
		DIR_N,
		DIR_W,
		DIR_S,
		DIR_E,
		DIR_NW,
		DIR_SW,
		DIR_NE,
		DIR_SE,
		DIR_MAX,
	};

	/// <summary>
	/// ノード情報 equalsはノード座標が等しいかを見る
	/// </summary>
	typedef struct NODE {
		NODE* parent = nullptr;
		POINT position = { -1, -1 };
		int score = 0;		//合計コスト(移動距離)
		int cost = 0;		//スタートから現時点までの距離
		int heuristic = 0;	//ゴールまでの推定値
		int& f = score;		//合計コスト(移動距離)
		int& g = cost;		//スタートから現時点までの距離
		int& h = heuristic;	//ゴールまでの推定値

		NODE operator = (const NODE& node) {
			this->parent(node.parent);
			this->position = node.position;
			this->f = node.f;
			this->g = node.g;
			this->h = node.h;
			return node;
		}

		//void operator = (const NODE& node) {
		//	this->parent = node.parent;
		//	this->position = node.position;
		//	this->f = node.f;
		//	this->g = node.g;
		//	this->h = node.h;
		//}

		//NODE operator = (const NODE& node) {
		//	this->parent = node.parent;
		//	this->position = node.position;
		//	this->f = node.f;
		//	this->g = node.g;
		//	this->h = node.h;
		//	return node;
		//}

		bool operator == (const NODE& node) {
			return (this->position.x == node.position.x && 
				this->position.y == node.position.y);
		}
		bool operator == (const POINT& pts) {
			return (this->position.x == pts.x &&
				this->position.y == pts.y);
		}
	};

	string pathStr;				//パスをstringで記録 いらないかも
	vector<vector<int>> map;	//マップが通れるか通れないかを01指定
	POINT mapRange;				//マップ縦横幅
	POINT startPt, endPt;		//開始地点、終了地点
	bool isGoal_;				//ゴールしたか
	bool enDiagonal;			//8方向(斜め)見るか
	vector<NODE> closeList;
	void Result();
	/// <summary>
	/// 方向に応じて-1~1のxyを返す
	/// </summary>
	/// <param name="dir">方向</param>
	/// <returns>方向に応じた-1~1xy座標</returns>
	POINT Dir2Value(DIRECTION dir);

	/// <summary>
	/// 最小最大の間に収まっているか or２回書くのが面倒だったから
	/// </summary>
	/// <param name="val">値</param>
	/// <param name="min">最小値</param>
	/// <param name="max">最大値</param>
	/// <returns>値が範囲内かの真偽値</returns>
	bool between(int val, int min, int max);

	/// <summary>
	/// マップ範囲内かを返す
	/// </summary>
	/// <param name="tgt">座標</param>
	/// <returns>範囲内=true</returns>
	bool IsValidPoint(POINT tgt);

	int CalcDistance(POINT p1, POINT p2);

	string GetRoute(NODE& node);
	void ShowAllNode();
	void OutCloseList();
	void OutList(vector<NODE> nodList, string nodListName);
	void GetInfo(NODE& node, string nodeName);
public:
	Astar();
	~Astar();
	void Init(vector<vector<int>> map, POINT s, POINT e, bool diagonal = false);
	void Run();
	string GetPathStr() { return pathStr; }
};

