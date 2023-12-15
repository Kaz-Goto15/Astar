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
	/// �m�[�h��� equals�̓m�[�h���W����������������
	/// </summary>
	typedef struct NODE {
		NODE* parent = nullptr;
		POINT position = { -1, -1 };
		int score = 0;		//���v�R�X�g(�ړ�����)
		int cost = 0;		//�X�^�[�g���猻���_�܂ł̋���
		int heuristic = 0;	//�S�[���܂ł̐���l
		int& f = score;		//���v�R�X�g(�ړ�����)
		int& g = cost;		//�X�^�[�g���猻���_�܂ł̋���
		int& h = heuristic;	//�S�[���܂ł̐���l

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

	string pathStr;				//�p�X��string�ŋL�^ ����Ȃ�����
	vector<vector<int>> map;	//�}�b�v���ʂ�邩�ʂ�Ȃ�����01�w��
	POINT mapRange;				//�}�b�v�c����
	POINT startPt, endPt;		//�J�n�n�_�A�I���n�_
	bool isGoal_;				//�S�[��������
	bool enDiagonal;			//8����(�΂�)���邩
	vector<NODE> closeList;
	void Result();
	/// <summary>
	/// �����ɉ�����-1~1��xy��Ԃ�
	/// </summary>
	/// <param name="dir">����</param>
	/// <returns>�����ɉ�����-1~1xy���W</returns>
	POINT Dir2Value(DIRECTION dir);

	/// <summary>
	/// �ŏ��ő�̊ԂɎ��܂��Ă��邩 or�Q�񏑂��̂��ʓ|����������
	/// </summary>
	/// <param name="val">�l</param>
	/// <param name="min">�ŏ��l</param>
	/// <param name="max">�ő�l</param>
	/// <returns>�l���͈͓����̐^�U�l</returns>
	bool between(int val, int min, int max);

	/// <summary>
	/// �}�b�v�͈͓�����Ԃ�
	/// </summary>
	/// <param name="tgt">���W</param>
	/// <returns>�͈͓�=true</returns>
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

