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

	// �e�m�[�h�̏��	Equals�͍��W����������������
	typedef struct NODE {
		int ID = -1;				//null��-1�Ƃ��邽�߂�����-1�ɂ��� �e������Ƃ����߂�0�ȏ�ɂȂ�
		int parentID = -1;			//�e��ID�Ǘ�
		POINT position = { -1,-1 };	//���W
		int f = 0;					//���v�R�X�g(�ړ�����) score
		int g = 0;					//�X�^�[�g���猻���_�܂ł̋��� cost
		int h = 0;					//�S�[���܂ł̐���l heuristic

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

	string pathStr;				//�p�X��string�ŋL�^ ����Ȃ�����
	vector<vector<int>> map;	//�}�b�v���ʂ�邩�ʂ�Ȃ�����01�w��
	POINT mapRange;				//�}�b�v�c����
	POINT startPt, endPt;		//�J�n�n�_�A�I���n�_
	bool isGoal_;				//�S�[��������
	bool enDiagonal;			//8����(�΂�)���邩
	vector<NODE> closeList;
	vector<int> pathList;
	bool debug;


	//�S�[������+���㏈���܂Ƃ�
	void Result();

	//�m�[�h�����\��
	string Attribute2Str(NODE_ATTRIBUTE id);

	/// �����ɉ�����-1~1��xy��Ԃ�
	POINT Dir2Value(DIRECTION dir);

	/// �ŏ��ő�̊ԂɎ��܂��Ă��邩 �ŏ��l�ő�l�͊܂�
	bool between(int val, int min, int max);

	/// �}�b�v�͈͓���
	bool IsValidPoint(POINT tgt);

	/// <summary>
	/// �p�X���L�^
	/// </summary>
	/// <param name="nodeID">�m�[�hID �N�����͍Ō�̃m�[�h���w��</param>
	/// <param name="pathArr">�L�^����intvector</param>
	void CalcPath(int nodeID, vector<int>& pathArr);

	//�ȉ��e�X�g�o�͌n
	void OutList(vector<NODE> nodList, string nodListName);
	void GetInfo(NODE& node, string nodeName);

	//�F
	enum COLOR_SEQ {
		RED,
		LIME,
		BLUE,
		YELLOW,
		BEIGE,
		CYAN,
		DEFAULT,
	};
	//�R���\�[���F�ς��֐�
	string OutStrColor(string str = "", COLOR_SEQ color = DEFAULT);
};