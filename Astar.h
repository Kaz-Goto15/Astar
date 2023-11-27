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
		}
		bool operator == (const NODE& node) {
			return (this->position.x == node.position.x && 
				this->position.y == node.position.y);
		}
	};
	void Result();
	string pathStr;	//�p�X��string�ŋL�^
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

