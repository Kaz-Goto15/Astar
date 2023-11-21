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
	//スタートゴールノード初期化
	NODE startNode;
	startNode.position = startPt;
	NODE endNode;
	endNode.position = endPt;

	//OPENリスト、CLOSEリスト　OPENリストにスタートノードを追加
	vector<NODE> openList;
	vector<NODE> closeList;
	openList.push_back(startNode);

	//OPENリストが空になるまで
	while (openList.size() > 0){
		//現在調べているノードを取得 OPENリストでF値が一番小さいノードを選ぶ
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
		//選択した最小F値ノードをOPENリストから削除、CLOSEリストに追加
		closeList.push_back(openList[currentIndex]);
		openList.erase(openList.begin() + currentIndex);

		//選択ノードがゴールであれば終了
		if (currentNode == endNode) {
			//////////////////////////////////
			//
			//  パス表示（親ノードをたどる））
			//
			//////////////////////////////////
			break;
		}

		//以降はゴールでないときの処理
	}

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