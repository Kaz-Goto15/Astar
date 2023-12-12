#include "Astar.h"
#include <iostream>
//#include <Windows.h>
#include <cmath>
using std::cout;
using std::endl;

Astar::POINT Astar::Dir2Value(DIRECTION dir)
{
	switch (dir){
	case Astar::DIR_N:	return {  0,-1 };
	case Astar::DIR_W:	return { -1, 0 };
	case Astar::DIR_S:	return {  0, 1 };
	case Astar::DIR_E:	return {  1, 0 };
	case Astar::DIR_NW:	return { -1,-1 };
	case Astar::DIR_SW:	return { -1, 1 };
	case Astar::DIR_NE:	return {  1,-1 };
	case Astar::DIR_SE:	return {  1, 1 };
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
	if (between(tgt.x, 0, mapRange.x) &&
		between(tgt.y, 0, mapRange.y))return true;
	return false;
}



Astar::Astar():
	pathStr(""),
	map(0),
	mapRange({ 0,0 }),
	startPt({-1,-1}),
	endPt({-1,-1}),
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

	//cout << "TEST: mapRange:" << mapRange.x << "," << mapRange.y << endl;

}

void Astar::Run()
{
	//�X�^�[�g�S�[���m�[�h������
	NODE startNode, endNode;
	startNode.position = startPt;
	endNode.position = endPt;

	//cout << "startNode:" << startNode.position.x << "," << startNode.position.y << endl;
	//cout << "endNode:" << endNode.position.x << "," << endNode.position.y << endl;

	//OPEN���X�g�ACLOSE���X�g�쐬�EOPEN���X�g�ɃX�^�[�g�m�[�h��ǉ�
	vector<NODE> openList;
	vector<NODE> closeList;
	openList.push_back(startNode);

	//cout << "openList:" << openList.size() << endl;

	//OPEN���X�g����ɂȂ�܂�
	while (openList.size() > 0) {
		cout << "openList size:" << openList.size() << endl;
		//OPEN���X�g����F�l����ԏ������m�[�h��I��
		NODE currentNode = openList[0];	//�Ƃ肠����0�Ԗ�
		int currentIndex = 0;
		for (int i = 0; i < openList.size(); i++) {
			if (openList[i].f < currentNode.f) {
				currentNode = openList[i];
				currentIndex = i;
			}
		}
		cout << "currentNode:" << currentNode.position.x << "," << currentNode.position.y << endl;

		//�I�������ŏ�F�l�m�[�h��CLOSE���X�g�ɒǉ��AOPEN���X�g����폜
		closeList.push_back(openList[currentIndex]);
		openList.erase(openList.begin() + currentIndex);

		//�I���m�[�h���S�[���ł���ΏI��
		if (currentNode == endNode) {
			cout << "TRIGGER : GOAL" << endl;
			isGoal_ = true;
			break;
		}
		else {
			cout << "TRIGGER : NOT GOAL" << endl;
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
					cout << "DIRECTION : " << d << endl;
					//�Ώۍ��W
					POINT targetPoint = currentNode.position + Dir2Value(d);
					cout << "TARGET : " << targetPoint.x << "," << targetPoint.y << endl;
					//MAP�͈͓��A�ړ��\�A�N���[�Y���X�g�ɂȂ��Ȃ��
					if (IsValidPoint(targetPoint) {
						cout << "TRIGGER : SAFE VALID" << endl;
					if( map[targetPoint.y][targetPoint.x] == 1){
						cout << "TRIGGER : SAFE RANGE" << endl;
					if( std::find(closeList.begin(), closeList.end(), targetPoint) != closeList.end()) {
						cout << "TRIGGER : NO RESULTS CLOSE LIST" << endl;
						cout << "TRIGGER : EXISTS" << endl;
						NODE targetNode;
						targetNode.position = targetPoint;


						auto result = std::find(openList.begin(), openList.end(), targetPoint);
						//�����W���I�[�v�����X�g�ɂȂ����
						if (result == openList.end()) {
							//���݃m�[�h���q�m�[�h�̐e�ɐݒ�
							targetNode.parent = &currentNode;

							//�q�m�[�h��F, G, H�l���v�Z
							targetNode.g = currentNode.g + 1;
							targetNode.h = std::pow(std::max(
								std::abs(targetNode.position.x - startNode.position.x),
								std::abs(targetNode.position.y - startNode.position.y)
							),2);
							targetNode.f = targetNode.g + targetNode.h;

							//�q�m�[�h���I�[�v�����X�g�ɒǉ�
							openList.push_back(targetNode);

						}
						//�����W���I�[�v�����X�g�ɂ����
						else {
							//G�l���r���Ă��ǂ��o�H���ǂ����iG�l�����������ǂ����j�m�F(������G�l�͂��ǂ��o�H)
							NODE& existNode = *result;
							if (existNode.g > targetNode.g) {
								//�����A�����q�m�[�h�ł�菬����G�l�ł���΁A�e�m�[�h�����݃m�[�h�ɐݒ肷��B
								existNode.parent = &currentNode;
							}
						}
					}
					else { cout << "TRIGGER : HAS CLOSE LIST" << endl; continue; }

					else {
						cout << "TRIGGER : NOT EXISTS" << endl;
						continue;
					}
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

void Astar::Result() {
	if (isGoal_) {
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// �S�[���m�[�h�̐e�m�[�h��H���Ă����A�X�^�[�g�m�[�h�ɖ߂�܂Őe��H���Ă����B�e�m�[�h�ʒu���t��(reverse)�ɂ���Ɨ~�����o�H���o�� //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	}
	else {cout << "�S�[���s�\" << endl;}
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