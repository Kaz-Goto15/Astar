#include "Astar.h"
#include <iostream>
#include <sstream>
//#include <Windows.h>
#include <cmath>
using std::cout;
using std::endl;

Astar::POINT Astar::Dir2Value(DIRECTION dir)
{
	switch (dir) {
	case Astar::DIR_N:	return { 0,-1 };
	case Astar::DIR_W:	return { -1, 0 };
	case Astar::DIR_S:	return { 0, 1 };
	case Astar::DIR_E:	return { 1, 0 };
	case Astar::DIR_NW:	return { -1,-1 };
	case Astar::DIR_SW:	return { -1, 1 };
	case Astar::DIR_NE:	return { 1,-1 };
	case Astar::DIR_SE:	return { 1, 1 };
	default:			exit(3);
	}
}

bool Astar::between(int val, int min, int max)
{
	if (val >= min && val <= max)return true;
	return false;
}

bool Astar::IsValidPoint(POINT tgt)
{
	if (between(tgt.x, 0, mapRange.x - 1) &&
		between(tgt.y, 0, mapRange.y - 1))return true;
	return false;
}


string Astar::OutStrColor(string str, COLOR_SEQ color)
{
	string ret = "";
	switch (color) {
	case RED:	ret += "\033[38;2;255;0;0m"; break;
	case LIME:	ret += "\033[38;2;0;255;0m"; break;
	case BLUE:	ret += "\033[38;2;99;99;255m"; break;
	case YELLOW:ret += "\033[38;2;255;255;0m"; break;
	case BEIGE:	ret += "\033[38;2;255;255;128m"; break;
	case CYAN:	ret += "\033[38;2;128;128;255m"; break;
	case DEFAULT:ret += "\033[0m"; break;
	}
	ret += str + "\033[0m";
	return ret;
}

void Astar::ShowMap()
{
	enum ENUM_STRID {
		START,
		END,
		WALL,
		FLOOR,
		PATH,
		MAX
	};
	string s[ENUM_STRID::MAX] = {
		"�r","�d","��","�@", "��"
	};
	for (int h = 0; h < mapRange.y; h++) {
		for (int w = 0; w < mapRange.x; w++) {
			if (map[h][w] == 1) {
				cout << s[WALL];
				continue;
			}
			else {
				//if()
			}
		}
		cout << endl;
	}


}

Astar::Astar() :
	pathStr(""),
	map(0),
	mapRange({ 0,0 }),
	startPt({ -1,-1 }),
	endPt({ -1,-1 }),
	isGoal_(false),
	enDiagonal(false)
{
}

Astar::~Astar()
{
}

void Astar::Init(vector<vector<int>> m, POINT s, POINT e, bool diagonal)
{
	pathStr = "";
	map = m;
	mapRange.x = m[0].size();
	mapRange.y = m.size();
	startPt = s;
	endPt = e;
	isGoal_ = false;
	enDiagonal = diagonal;

}

void Astar::Run()
{
	//�X�^�[�g�S�[���m�[�h������
	NODE startNode, endNode;
	startNode.position = startPt;
	endNode.position = endPt;

	//OPEN���X�g�쐬�E�X�^�[�g�m�[�h��ǉ�
	vector<NODE> openList;
	GetInfo(startNode, "startNode");
	openList.push_back(startNode);

	ShowAllNode();

	int count = 0;
	//OPEN���X�g����ɂȂ�܂�
	while (openList.size() > 0) {
		//ShowAllNode();
		OutList(openList, "openList");

		GetInfo(openList[0], "oL0");
		//OPEN���X�g����F�l����ԏ������m�[�h��I��
		NODE currentNode;
		currentNode = openList[0];	//�Ƃ肠����0�Ԗ�

		GetInfo(openList[0], "oL0");
		int currentIndex = 0;
		for (int i = 0; i < openList.size(); i++) {
			if (openList[i].f < currentNode.f) {
				currentNode = openList[i];
				currentIndex = i;
			}
		}

		GetInfo(currentNode, "currentNode");
		GetInfo(openList[0], "oL0");

		//count++;
		//if (count >= 2)currentNode.parentID = startNode.ID;
		//if (count >= 3)break;

		//�I�������ŏ�F�l�m�[�h��CLOSE���X�g�ɒǉ��AOPEN���X�g����폜
		//closeList.push_back(openList[currentIndex]);
		//openList[0].parentID = &startNode;
		currentNode.ID = closeList.size();
		closeList.push_back(currentNode);
		//closeList.back().parentID = cNode.parentID;


		cout << closeList.back().parentID << endl;
		GetInfo(openList[0], "oL0");

		openList.erase(openList.begin() + currentIndex);

		//�I���m�[�h���S�[���ł���ΏI��
		if (currentNode == endNode) {
			cout << "**********************END**********************" << endl;
			isGoal_ = true;
			break;
		}
		else {
			//�ȍ~�̓S�[���łȂ��Ƃ��̏���

			//���݃m�[�h�ɑ΂��Ĉړ��\��4�����m�[�h�i�㉺���E�j�A�΂߈ړ����n�j�Ƃ���Ȃ�8�����m�[�h�i�㉺���E�{�΂�4�����j�̎q�m�[�h�ɑ΂��A
			//�ړ��s�\�ʒu or �N���[�Y���X�g�ɂ���Ȃ疳���A����ȊO�Ȃ玟�̎菇�����s
			//�I�[�v�����X�g�ɂȂ���Βǉ�����B���̍ی��݃m�[�h���q�m�[�h�̐e�ɐݒ肷��B
			//�����Ďq�m�[�h��F, G, H�l���v�Z����B
			//�I�[�v�����X�g�ɓ����q�m�[�h�i�����ʒu�j�����ɂ���΁A
			//G�l���r���Ă��ǂ��o�H���ǂ����iG�l�����������ǂ����j�m�F����B
			//������G�l�͂��ǂ��o�H���Ӗ����܂��B
			//�����A�����q�m�[�h�ł�菬����G�l�ł���΁A�e�m�[�h�����݃m�[�h�ɐݒ肷��B

			//�΂߃A����
			if (enDiagonal) {
				cout << "TRIGGER : DIAGONAL" << endl;
				//�΂߂���
				for (DIRECTION d = static_cast<DIRECTION>(0); d < DIR_MAX; d = static_cast<DIRECTION>(d + 1)) {
					cout << "DIR : " << d << endl;
					OutCloseList();
					//�Ώۍ��W
					POINT targetPoint = currentNode.position + Dir2Value(d);
					cout << " TGT : " << targetPoint.x << "," << targetPoint.y;
					//MAP�͈͓��A�ړ��\�A�N���[�Y���X�g�ɂȂ��Ȃ��
					if (IsValidPoint(targetPoint)) {
						cout << OutStrColor(" RANGE ", LIME);
						if (map[targetPoint.y][targetPoint.x] == 0) {
							cout << OutStrColor(" FLOOR ", LIME);
							if (std::find(closeList.begin(), closeList.end(), targetPoint) == closeList.end()) {
								cout << OutStrColor(" OPEN ", LIME) << OutStrColor(" EXISTS ", YELLOW) << endl;
								NODE targetNode;
								targetNode.position = targetPoint;


								auto result = std::find(openList.begin(), openList.end(), targetPoint);
								//�����W���I�[�v�����X�g�ɂȂ����
								if (result == openList.end()) {
									cout << OutStrColor(" NOT EXISTS IN OPENLIST ", CYAN);
									//���݃m�[�h���q�m�[�h�̐e�ɐݒ�
									targetNode.parentID = currentNode.ID;

									//�q�m�[�h��F, G, H�l���v�Z
									targetNode.g = currentNode.g + 1;
									targetNode.h = std::pow(std::max(
										std::abs(targetNode.position.x - startNode.position.x),
										std::abs(targetNode.position.y - startNode.position.y)
									), 2);
									targetNode.f = targetNode.g + targetNode.h;

									std::stringstream ss;
									ss << "ADD OPENLIST : {" << targetNode.position.x << ", " << targetNode.position.y << "} parentID: {" << closeList[targetNode.parentID].position.x << ", " << closeList[targetNode.parentID].position.y << "} fgh: " << targetNode.f << ", " << targetNode.g << ", " << targetNode.h << " \033[0m" << endl;
									cout << OutStrColor(ss.str(), BLUE);
									//�q�m�[�h���I�[�v�����X�g�ɒǉ�
									openList.push_back(targetNode);

								}
								//�����W���I�[�v�����X�g�ɂ����
								else {
									cout << OutStrColor("EXISTS IN OPENLIST", BEIGE);
									//G�l���r���Ă��ǂ��o�H���ǂ����iG�l�����������ǂ����j�m�F(������G�l�͂��ǂ��o�H)
									NODE& existNode = *result;
									targetNode.g = currentNode.g + 1;
									cout << "tgt.g: " << targetNode.g;
									cout << " opl.g: " << existNode.g;
									if (existNode.g > targetNode.g) {
										//�����A�����q�m�[�h�ł�菬����G�l�ł���΁A�e�m�[�h�����݃m�[�h�ɐݒ肷��B
										//existNode.parentID = &currentNode;
										existNode.parentID = currentNode.ID;
										cout << "CHANGE parentID-> " << closeList[existNode.parentID].position.x << "," << closeList[existNode.parentID].position.y;
									}
									cout << endl;
								}
							}
							else { cout << OutStrColor(" CLOSED ", RED) << endl; continue; }
						}
						else { cout << OutStrColor(" WALL ", RED) << endl; continue; }
					}
					else { cout << OutStrColor(" RANGE ", RED) << endl; continue; }
				}
			}
			else {
				cout << "TRIGGER : NON DIAGONAL" << endl;
				//�΂߂Ȃ�
				for (DIRECTION d = static_cast<DIRECTION>(0); d <= DIR_E; d = static_cast<DIRECTION>(d + 1)) {

				}
			}
		}
	}

	Result();
}

void Astar::ShowAllNode()
{
	cout << "=========================================" << endl;

	NODE nodes[5][5];
	cout << "closeList:"; for (auto& a : closeList) { cout << "{" << a.position.x << "," << a.position.y << "}"; } cout << endl;
	for (auto& cL : closeList) {
		nodes[cL.position.y][cL.position.x] = cL;
	}
	for (int i = 0; i < mapRange.x; i++)	cout << "\033[4m       \033[0m";
	cout << endl;
	for (int j = 0; j < mapRange.y; j++) {
		for (int i = 0; i < mapRange.x; i++)	cout << "| f g h";
		cout << "|" << endl;
		for (int i = 0; i < mapRange.x; i++) {
			cout << "|";
			if (nodes[j][i].f < 10) cout << " ";
			cout << nodes[j][i].f;
			if (nodes[j][i].g < 10) cout << " ";
			cout << nodes[j][i].g;
			if (nodes[j][i].h < 10) cout << " ";
			cout << nodes[j][i].h;
		}
		cout << "|" << endl;
		for (int i = 0; i < mapRange.x; i++) {
			cout << "\033[4m";
			if (nodes[j][i].parentID == -1)	cout << "|(?, ?)";
			else								cout << "|(" << closeList[nodes[j][i].parentID].position.x << ", " << closeList[nodes[j][i].parentID].position.y << ")";
			cout << "\033[0m";
		}
		cout << "|" << endl;
	}
	cout << endl << endl;
	cout << "=========================================" << endl;
}

void Astar::OutCloseList()
{
	cout << "closeList:";
	for (auto& a : closeList) {
		cout << "{" << a.position.x << "," << a.position.y << "}"
			<< "fgh:" << a.f << "," << a.g << "," << a.h
			<< " parentID:(";
		if (a.parentID == -1) {
			cout << "nothing)";
		}
		else {
			cout << closeList[a.parentID].position.x << "," << closeList[a.parentID].position.y << ")";
		}
		cout << endl << "           ";
	}
	cout << endl;
}

void Astar::OutList(vector<NODE> nodList, string nodListName)
{
	cout << nodListName << ":";
	for (auto& a : nodList) {
		cout << "{" << a.position.x << "," << a.position.y << "}"
			<< "fgh:" << a.f << "," << a.g << "," << a.h
			<< " parentID:(";
		if (a.parentID == -1) {
			cout << "nothing)";
		}
		else {
			cout << closeList[a.parentID].position.x << "," << closeList[a.parentID].position.y << ")";
		}
		cout << endl << "           ";
	}
	cout << endl;
}

void Astar::GetInfo(NODE& node, string nodeName)
{
	cout << nodeName << ":{" << node.position.x << "," << node.position.y << "}"
		<< "fgh:" << node.f << "," << node.g << "," << node.h
		<< " parentID:(";
	if (node.parentID == -1) {
		cout << "nothing)";
	}
	else {
		cout << closeList[node.parentID].position.x << "," << closeList[node.parentID].position.y << ")";
	}
	cout << endl;
}

string Astar::GetRoute(int nodeID)
{
	string ret = "";
	NODE node = closeList[nodeID];
	cout << node.position.x << "," << node.position.y << endl;
	if (node.parentID != -1)ret += GetRoute(node.parentID);
	return ret += "-> {" + std::to_string(node.position.x) + "," + std::to_string(node.position.y) + "}";
}

void Astar::Result() {
	if (isGoal_) {
		pathStr = GetRoute(closeList.back().ID);
		for (int i = closeList.size() - 1; i >= 0; --i) {
			cout << "{" << closeList[i].position.x << "," << closeList[i].position.y << "} -> {";
			if (closeList[i].parentID == -1) {
				cout << "?,?}\n";
			}
			else {
				cout << closeList[closeList[i].parentID].position.x << "," << closeList[closeList[i].parentID].position.y << "}" << endl;
			}
		}
		ShowAllNode();
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// �S�[���m�[�h�̐e�m�[�h��H���Ă����A�X�^�[�g�m�[�h�ɖ߂�܂Őe��H���Ă����B�e�m�[�h�ʒu���t��(reverse)�ɂ���Ɨ~�����o�H���o�� //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	}
	else { cout << "�S�[���s�\" << endl; }
}

/*
�X�^�[�g�m�[�h���I�[�v�����X�g�ɒǉ�����B
���L�菇���J��Ԃ��B
�I�[�v�����X�g�̒��ōł��ႢF�l��T���Ă��������B��������݂̃m�[�h�Ƃ��܂��B
������I�[�v�����X�g����폜���A�N���[�Y���X�g�ɒǉ����Ă��������B

���݃m�[�h�ɑ΂��Ĉړ��\��4�����m�[�h�i�㉺���E�j�A�΂߈ړ����n�j�Ƃ���Ȃ�8�����m�[�h�i�㉺���E�{�΂�4�����j�̎q�m�[�h�ɑ΂��A
�ړ��s�\�ʒu or �N���[�Y���X�g�ɂ���Ȃ疳���A����ȊO�Ȃ玟�̎菇�����s
�I�[�v�����X�g�ɂȂ���Βǉ�����B���̍ی��݃m�[�h���q�m�[�h�̐e�ɐݒ肷��B�����Ďq�m�[�h��F, G, H�l���v�Z����B
�I�[�v�����X�g�ɓ����q�m�[�h�i�����ʒu�j�����ɂ���΁AG�l���r���Ă��ǂ��o�H���ǂ����iG�l�����������ǂ����j�m�F����B������G�l�͂��ǂ��o�H���Ӗ����܂��B�����A�����q�m�[�h�ł�菬����G�l�ł���΁A�e�m�[�h�����݃m�[�h�ɐݒ肷��B

�������𖞂����Ȃ�v���O�����I������B
���݃m�[�h���S�[���m�[�h��������i�S�[������������j�A
���̓S�[���m�[�h�������炸�A�I�[�v�����X�g������ۂɂȂ�����B�i�S�[���ւ̌o�H�����݂��Ȃ��P�[�X�B�j
�������o�H��ۑ�����B�S�[���m�[�h�̐e�m�[�h��H���Ă����A�X�^�[�g�m�[�h�ɖ߂�܂Őe��H���Ă����B�e�m�[�h�ʒu���t��(reverse)�ɂ���Ɨ~�����o�H���o���B

�X�R�A�������ꍇ�A���R�X�g��D�悵�Ċ�m�[�h�Ƃ��Ȃ��ƍŒZ���[�g�ƂȂ�Ȃ� �P�[�X������B
*/

//NODE��ID��ǉ�����������Ƃ���Ȃ��񂶂�Ȃ���