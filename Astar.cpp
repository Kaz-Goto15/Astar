#include "Astar.h"
#include <iostream>
#include <sstream>
#include <cmath>

using std::cout;
using std::endl;

Astar::Astar() :
	pathStr(""),
	map(0),
	mapRange({ 0,0 }),
	startPt({ -1,-1 }),
	endPt({ -1,-1 }),
	isGoal_(false),
	enDiagonal(false),
	closeList(0),
	pathList(0),
	debug(false)
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
	openList.push_back(startNode);

	//OPEN���X�g����ɂȂ�܂�
	while (openList.size() > 0) {
		//OPEN���X�g����F�l����ԏ������m�[�h��I��
		NODE currentNode;
		currentNode = openList[0];	//�Ƃ肠����0�Ԗ�

		int currentIndex = 0;
		for (int i = 0; i < openList.size(); i++) {
			if (openList[i].f < currentNode.f) {
				currentNode = openList[i];
				currentIndex = i;
			}
		}

		GetInfo(currentNode, "currentNode");

		//�I�������ŏ�F�l�m�[�h��CLOSE���X�g�ɒǉ��AOPEN���X�g����폜
		currentNode.ID = closeList.size();
		closeList.push_back(currentNode);
		openList.erase(openList.begin() + currentIndex);

		//�I���m�[�h���S�[���ł���ΏI��
		if (currentNode == endNode) {
			if(debug)cout << "**********************END**********************" << endl;
			isGoal_ = true;
			break;
		}
		else {
			//�ȍ~�̓S�[���łȂ��Ƃ��̏���
			//�΂߃A����
			if (enDiagonal) {
				//�΂߂���
				for (DIRECTION d = static_cast<DIRECTION>(0); d < DIR_MAX; d = static_cast<DIRECTION>(d + 1)) {
					OutList(closeList, "closeList");
					//�Ώۍ��W
					POINT targetPoint = currentNode.position + Dir2Value(d);
					if(debug)cout << " TGT : " << targetPoint.x << "," << targetPoint.y;
					//MAP�͈͓��A�ړ��\�A�N���[�Y���X�g�ɂȂ��Ȃ��
					if (IsValidPoint(targetPoint)) {
						cout << OutStrColor(" RANGE ", LIME);
						if (map[targetPoint.y][targetPoint.x] == 0) {
							cout << OutStrColor(" FLOOR ", LIME);
							if (std::find(closeList.begin(), closeList.end(), targetPoint) == closeList.end()) {
								cout << OutStrColor(" OPEN ", LIME) << OutStrColor(" EXISTS \n", YELLOW);
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
									if (debug) {
										cout << "tgt.g: " << targetNode.g;
										cout << " opl.g: " << existNode.g;
									}
									if (existNode.g > targetNode.g) {
										//�����A�����q�m�[�h�ł�菬����G�l�ł���΁A�e�m�[�h�����݃m�[�h�ɐݒ肷��B
										existNode.parentID = currentNode.ID;
										if(debug)cout << "CHANGE parentID-> " << closeList[existNode.parentID].position.x << "," << closeList[existNode.parentID].position.y;
									}
									if(debug)cout << endl;
								}
							}
							else { cout << OutStrColor(" CLOSED \n", RED); continue; }
						}
						else { cout << OutStrColor(" WALL \n", RED); continue; }
					}
					else { cout << OutStrColor(" RANGE \n", RED); continue; }
				}
			}
			else {
				//�΂߂Ȃ�
				for (DIRECTION d = static_cast<DIRECTION>(0); d <= DIR_E; d = static_cast<DIRECTION>(d + 1)) {

				}
			}
		}
	}

	Result();
}

void Astar::DrawInfoTable()
{
	vector<vector<NODE>> nodes(mapRange.y, vector<NODE>(mapRange.x));
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
			if (nodes[j][i].parentID == -1)	cout << "|(-, -)";
			else								cout << "|(" << closeList[nodes[j][i].parentID].position.x << ", " << closeList[nodes[j][i].parentID].position.y << ")";
			cout << "\033[0m";
		}
		cout << "|" << endl;
	}
	cout << endl << endl;
}

void Astar::DrawPath()
{
	for (int i = pathList.size() - 1; i >= 0; i--){
		cout << "{" << closeList[pathList[i]].position.x << "," << closeList[pathList[i]].position.y << "}->";
	}
}

void Astar::DrawMap()
{
	//�������̂ق�����r�I����
	vector<vector<NODE_ATTRIBUTE>> mapData(mapRange.y, vector<NODE_ATTRIBUTE>(mapRange.x));
	for (int h = 0; h < mapRange.y; h++) {
		for (int w = 0; w < mapRange.x; w++) {
			mapData[h][w] = static_cast<NODE_ATTRIBUTE>(map[h][w]);
		}
	}
	for (int& id : pathList) {
		mapData[closeList[id].position.y][closeList[id].position.x] = PATH;
	}
	mapData[startPt.y][startPt.x] = START;
	mapData[endPt.y][endPt.x] = END;

	//�o��
	for (int h = 0; h < mapRange.y; h++) {
		for (int w = 0; w < mapRange.x; w++) {
			cout << Attribute2Str(mapData[h][w]);
		}
		cout << endl;
	}
}


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
	}
	return { 0,0 };
}

bool Astar::between(int val, int min, int max)
{
	return  (val >= min && val <= max);
}

bool Astar::IsValidPoint(POINT tgt)
{
	if (between(tgt.x, 0, mapRange.x - 1) &&
		between(tgt.y, 0, mapRange.y - 1))return true;
	return false;
}


string Astar::OutStrColor(string str, COLOR_SEQ color)
{
	if (debug) {
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
	return "";
}

void Astar::OutList(vector<NODE> nodList, string nodListName)
{
	if (debug) {
		cout << nodListName << ":\n";
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
			cout << endl;
		}
		cout << endl;
	}
}

void Astar::CalcPath(int nodeID, vector<int>& pathArr)
{
	pathArr.push_back(nodeID);
	NODE node = closeList[nodeID];
	if (node.parentID != -1) CalcPath(node.parentID, pathArr);
}

void Astar::GetInfo(NODE& node, string nodeName)
{
	if (debug) {
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
}

string Astar::Attribute2Str(NODE_ATTRIBUTE id)
{
	switch (id)
	{
	case NODE_ATTRIBUTE::FLOOR:	return "�@";
	case NODE_ATTRIBUTE::WALL:	return "��";
	case NODE_ATTRIBUTE::PATH:	return "��";
	case NODE_ATTRIBUTE::START:	return "�r";
	case NODE_ATTRIBUTE::END:	return "�d";
	}
	return "�}";
}

void Astar::Result() {
	if (isGoal_) {
		CalcPath(closeList.size() - 1, pathList);
	}
	else { cout << "�S�[���s�\" << endl; }
}