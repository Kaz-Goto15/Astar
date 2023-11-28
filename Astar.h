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

		void Set(int x, int z) {
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
	struct NODE {
		NODE* parent = nullptr;
		POINT position = { -1, -1 };
		int score = 0;		//���v�R�X�g(�ړ�����)
		int cost = 0;		//�X�^�[�g���猻���_�܂ł̋���
		int heuristic = 0;	//�S�[���܂ł̐���l
		int& f = score;		//���v�R�X�g(�ړ�����)
		int& g = cost;		//�X�^�[�g���猻���_�܂ł̋���
		int& h = heuristic;	//�S�[���܂ł̐���l

		NODE operator = (const NODE& node) {
			this->parent = node.parent;
			this->position = node.position;
			this->f = node.f;
			this->g = node.g;
			this->h = node.h;
			return node;
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
	void Result();
	string pathStr;	//�p�X��string�ŋL�^
	vector<vector<int>> map;
	POINT mapRange;
	POINT startPt, endPt;
	bool isGoal_;
	bool enDiagonal = false;

	POINT Dir2Value(DIRECTION dir);
	bool between(int val, int min, int max);
	bool IsValidPoint(POINT tgt);
public:
	Astar();
	~Astar();
	void Init(vector<vector<int>> map, POINT s, POINT e);
	void Run();
	string GetPathStr() { return pathStr; }
};

