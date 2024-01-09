#pragma once
#include <string>
#include <vector>

using std::string;
using std::vector;

enum NODE_ATTRIBUTE {
	FLOOR,
	WALL,
	PATH,
	START,
	END,
	MAX
};

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

	Astar();
	~Astar();
	void Init(vector<vector<int>> map, POINT s, POINT e, bool diagonal = false);
	void Run();
	void DrawInfoTable();
	void DrawPath();
	void DrawMap();

	void IsDebugMsg(bool b) { debug = b; }
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
		DIR_MAX_8,
		DIR_MAX_4 = DIR_NW,
	};

	// 各ノードの情報	Equalsは座標が等しいかを見る
	typedef struct NODE {
		int ID = -1;				//nullを-1とするためここも-1にする 親があるとき初めて0以上になる
		int parentID = -1;			//親をID管理
		POINT position = { -1,-1 };	//座標
		int f = 0;					//合計コスト(移動距離) score
		int g = 0;					//スタートから現時点までの距離 cost
		int h = 0;					//ゴールまでの推定値 heuristic

		void operator = (const NODE& node) {
			this->parentID = node.parentID;
			this->position = node.position;
			this->f = node.f;
			this->g = node.g;
			this->h = node.h;
		}

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
	vector<int> pathList;
	bool debug;


	//ゴール判定+事後処理まとめ
	void Result();

	//ノード属性表示
	string Attribute2Str(NODE_ATTRIBUTE id);

	/// 方向に応じて-1~1のxyを返す
	POINT Dir2Value(DIRECTION dir);

	/// 最小最大の間に収まっているか 最小値最大値は含む
	bool between(int val, int min, int max);

	/// マップ範囲内か
	bool IsValidPoint(POINT tgt);

	/// <summary>
	/// パスを記録
	/// </summary>
	/// <param name="nodeID">ノードID 起動時は最後のノードを指定</param>
	/// <param name="pathArr">記録するintvector</param>
	void CalcPath(int nodeID, vector<int>& pathArr);

	//以下テスト出力系
	void OutList(vector<NODE> nodList, string nodListName);
	void GetInfo(NODE& node, string nodeName);

	//色
	enum COLOR_SEQ {
		RED,
		LIME,
		BLUE,
		YELLOW,
		BEIGE,
		CYAN,
		DEFAULT,
	};
	//コンソール色変え関数
	string OutStrColor(string str = "", COLOR_SEQ color = DEFAULT);
};