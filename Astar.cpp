#include "Astar.h"

Astar::Astar():
	pathStr(""),
	map(0),
	startPt({-1,-1}),
	endPt({-1,-1})
{
}

Astar::~Astar()
{
}

void Astar::Init(vector<vector<int>> m, POINT s, POINT e)
{
	map = m;
	startPt = s;
	endPt = e;
}

void Astar::Run()
{
	//�X�^�[�g�S�[���m�[�h������
	NODE startNode;
	startNode.position = startPt;
	NODE endNode;
	endNode.position = endPt;

	//OPEN���X�g�ACLOSE���X�g�@OPEN���X�g�ɃX�^�[�g�m�[�h��ǉ�
	vector<NODE> openList;
	vector<NODE> closeList;
	openList.push_back(startNode);

	//OPEN���X�g����ɂȂ�܂�
	while (openList.size() > 0){
		//���ݒ��ׂĂ���m�[�h���擾 OPEN���X�g��F�l����ԏ������m�[�h��I��
		NODE currentNode = openList[0];
		int currentIndex = 0;

		//for (NODE& n : openList) {
		//	if (n.f < currentNode.f) {
		//		currentNode = n;
		//		
		//	}
		//}
		for (int i = 0; i < openList.size(); i++) {
			if (openList[i].f < currentNode.f) {
				currentNode = openList[i];
				currentIndex = i;
			}
		}
		//�I�������ŏ�F�l�m�[�h��OPEN���X�g����폜�ACLOSE���X�g�ɒǉ�
		closeList.push_back(openList[currentIndex]);
		openList.erase(openList.begin() + currentIndex);

		//�I���m�[�h���S�[���ł���ΏI��
		if (currentNode == endNode) {
			//////////////////////////////////
			//
			//  �p�X�\���i�e�m�[�h�����ǂ�j�j
			//
			//////////////////////////////////
			break;
		}

		//�ȍ~�̓S�[���łȂ��Ƃ��̏���
	}

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