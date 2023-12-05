#include "Astar.h"
#include <iostream>
#include <Windows.h>

using std::cout;
using std::endl;

Astar::POINT Astar::Dir2Value(DIRECTION dir)
{
	switch (dir){
	case Astar::DIR_N:	return {  0,-1 };
	case Astar::DIR_W:	return { -1, 0 };
	case Astar::DIR_S:	return {  0, 1 };
	case Astar::DIR_E:	return {  0, 1 };
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

void Astar::Init(vector<vector<int>> m, POINT s, POINT e, bool diagonal = false)
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
	//スタートゴールノード初期化
	NODE startNode, endNode;
	startNode.position = startPt;
	endNode.position = endPt;

	//OPENリスト、CLOSEリスト作成・OPENリストにスタートノードを追加
	vector<NODE> openList;
	vector<NODE> closeList;
	openList.push_back(startNode);

	//OPENリストが空になるまで
	while (openList.size() > 0) {
		//OPENリスト内でF値が一番小さいノードを選ぶ
		NODE currentNode = openList[0];	//とりあえず0番目
		int currentIndex = 0;
		for (int i = 0; i < openList.size(); i++) {
			if (openList[i].f < currentNode.f) {
				currentNode = openList[i];
				currentIndex = i;
			}
		}

		//選択した最小F値ノードをCLOSEリストに追加、OPENリストから削除
		closeList.push_back(openList[currentIndex]);
		openList.erase(openList.begin() + currentIndex);

		//選択ノードがゴールであれば終了
		if (currentNode == endNode) {
			isGoal_ = true;
			break;
		}
		else {
			//以降はゴールでないときの処理

			//現在ノードに対して移動可能な4方向ノード（上下左右）、斜め移動をＯＫとするなら8方向ノード（上下左右＋斜め4方向）の子ノードに対し、
			//移動不可能位置 or クローズリストにあるなら無視、それ以外なら次の手順を実行
			//オープンリストになければ追加する。その際現在ノードを子ノードの親に設定する。
			//そして子ノードのF, G, H値を計算する。
			//オープンリストに同じ子ノード（同じ位置）が既にあれば、
			//G値を比較してより良い経路かどうか（G値が小さいかどうか）確認する。
			//小さいG値はより良い経路を意味します。
			//もし、同じ子ノードでより小さいG値であれば、親ノードを現在ノードに設定する。


			//斜めアリか
			if (enDiagonal) {
				//斜めあり
				for (DIRECTION d = static_cast<DIRECTION>(0); d < DIR_MAX; d = static_cast<DIRECTION>(d + 1)) {
					//対象座標
					POINT targetPoint = currentNode.position + Dir2Value(d);
					//MAP範囲内、移動可能、クローズリストにないならば
					if (IsValidPoint(targetPoint) &&
						map[targetPoint.y][targetPoint.x] == 1 &&
						std::find(closeList.begin(), closeList.end(), targetPoint) != closeList.end()
						) {
						//オープンリストになければ
						if (std::find(openList.begin(), openList.end(), targetPoint) != openList.end()) {
							//現在ノードを子ノードの親に設定
							NODE targetNode;
							targetNode.position = targetPoint;
							targetNode.parent = &currentNode;

							//子ノードのF, G, H値を計算
							targetNode.f = 0;
							targetNode.g = currentNode.g + 1;
							targetNode.h = 0;

							//オープンリストに同じ子ノード（同じ位置）が既にあれば、
							//G値を比較してより良い経路かどうか（G値が小さいかどうか）確認
							//小さいG値はより良い経路を意味します。
							//もし、同じ子ノードでより小さいG値であれば、親ノードを現在ノードに設定する。

							//子ノードをオープンリストに追加
							openList.push_back(targetNode);

						}
						else {
							//NODE& subNode;
							//オープンリストに同座標の子ノードがある場合
							
							//G値を比較してより良い経路かどうか（G値が小さいかどうか）確認
							//小さいG値はより良い経路を意味します。
							//もし、同じ子ノードでより小さいG値であれば、親ノードを現在ノードに設定する。
						}
					} else continue;
				}
			}
			else {
				//斜めなし
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
		// ゴールノードの親ノードを辿っていき、スタートノードに戻るまで親を辿っていく。各ノード位置を逆順(reverse)にすると欲しい経路が出る //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	}
	else {cout << "ゴール不可能" << endl;}
}

/*
スタートノードをオープンリストに追加する。
下記手順を繰り返す。
オープンリストの中で最も低いF値を探してください。これを現在のノードとします。
これをオープンリストから削除し、クローズリストに追加してください。

現在ノードに対して移動可能な4方向ノード（上下左右）、斜め移動をＯＫとするなら8方向ノード（上下左右＋斜め4方向）の子ノードに対し、
移動不可能位置 or クローズリストにあるなら無視、それ以外なら次の手順を実行
オープンリストになければ追加する。その際現在ノードを子ノードの親に設定する。そして子ノードのF, G, H値を計算する。
オープンリストに同じ子ノード（同じ位置）が既にあれば、G値を比較してより良い経路かどうか（G値が小さいかどうか）確認する。小さいG値はより良い経路を意味します。もし、同じ子ノードでより小さいG値であれば、親ノードを現在ノードに設定する。

もし次を満たすならプログラム終了する。
現在ノードがゴールノードだったら（ゴールを見つけたら）、
又はゴールノードが見つからず、オープンリストが空っぽになったら。（ゴールへの経路が存在しないケース。）
見つけた経路を保存する。ゴールノードの親ノードを辿っていき、スタートノードに戻るまで親を辿っていく。各ノード位置を逆順(reverse)にすると欲しい経路が出るよ。

スコアが同じ場合、実コストを優先して基準ノードとしないと最短ルートとならない ケースがある。
*/

//NODEにIDを追加する方式だといらないんじゃないか