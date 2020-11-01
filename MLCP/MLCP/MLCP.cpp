// MLCP.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <ctime>
#include <string>
#include <string.h>
#include <algorithm>

//该版本加入了swap操作，tt固定15，alpha0.01，omega1000，与exp相比omega变了，tt变了
using namespace std;
#define MAX_VAL 999999

clock_t startTime, endTime;//记录运行时间
double bestTime;//最佳时间
double cutting_time = 180;//终止条件
string inFile = "C:/Users/18367/Documents/Visual Studio 2015/Projects/MLCP/Debug/data/anna.col";
string outFile = "C:/Users/18367/Documents/Visual Studio 2015/Projects/MLCP/Debug/data/experiment_tt20_swap_exp4.txt";


//主要调节的参数
int tt = 2;//禁忌长度
int omega = 500;//扰动的阈值
float alpha = 0.01;//扰动强度
int noimprove = 0;//连续未更新的次数

int Sc;
int Sb;

int edgeNum, verNum;
int** edge;//两点间是否有边
int** adj;//记录跟每架飞机有冲突的飞机
int* adjlen;//记录跟每架飞机冲突飞机的数量

//主要数据结构
int** color;
int* colorlen;
int* v2color;//点的着色
int* v2idx;
int* conect;//与所在颜色的边
int* bian;//两个颜色中边的数量

//局部搜索
int* tabu;//禁忌表
int iter = 0;//迭代次数

//函数定义
int random_int(int);
void read_initial(string);
void random_initial();//随机构建初始解
void compute_obj();//计算目标值
void FLIP(int n);//将飞机flt从当前gate移动到gt
void show_result();
void local_search();
void judge_best();
void pertubation();//扰动


//随机生成一个 0-(n-1) 的整数
int random_int(int n)
{
	return rand() % n;
}

//读取数据
void read_initial(string inFile)
{
	int i, j, k;
	ifstream FIC;
	FIC.open(inFile);//读取数据
	if (FIC.fail())
	{
		cout << "### Erreur open, File_Name " << inFile << endl;
		getchar();
		exit(0);
	}
	if (FIC.eof())
	{
		cout << "### Error open, File_Name " << inFile << endl;
		getchar();
		exit(0);
	}
	FIC >> verNum >> edgeNum;
	edge = new int*[verNum];//记录哪两个点相连
	adj = new int*[verNum];//记录点与哪些点相连
	color = new int*[2];//记录每种颜色都有哪些点
	adjlen = new int[verNum];//点与哪些点相连的长度
	tabu = new int[verNum];
	v2color = new int[verNum];
	v2idx = new int[verNum];
	conect = new int[verNum];
	colorlen = new int[2];//两种颜色着色的点数计数
	bian = new int[2];
	color[0] = new int[verNum];
	color[1] = new int[verNum];

	for (i = 0; i < verNum; i++)
	{
		edge[i] = new int[verNum];
		adj[i] = new int[verNum];
		adjlen[i] = 0;
	}
	for (i = 0; i < verNum; i++)
		for (j = 0; j < verNum; j++)
		{
			edge[i][j] = 0;
			adj[i][j] = 0;
		}

	int a, b;
	for (i = 0; i < edgeNum; i++)
	{
		FIC >> a >> b;
		a--;
		b--;
		edge[a][b] = edge[b][a] = 1;//如果二者有边相连

	}

	edgeNum = 0;
	for (i = 0; i < verNum - 1; i++)
		for (j = i + 1; j < verNum; j++)
			if (edge[i][j])
			{
				edgeNum++;
				adj[i][adjlen[i]] = j;
				adj[j][adjlen[j]] = i;
				adjlen[i]++;
				adjlen[j]++;
			}

	FIC.close();//关闭数据
}

//随机构建初始解
void random_initial()
{
	int i, j, k, m;
	colorlen[0] = 0;
	colorlen[1] = 0;
	bian[0] = 0;
	bian[1] = 0;
	for (i = 0; i < verNum; i++)
	{
		tabu[i] = 0;
		conect[i] = 0;
	}
	for (i = 0; i < verNum; i++)
	{
		k = random_int(2);
		color[k][colorlen[k]] = i;
		v2color[i] = k;//点的着色
		v2idx[i] = colorlen[k];//是第几个着这个颜色的
		colorlen[k]++;
	}
	for (i = 0; i < verNum; i++)
		for (j = 0; j < verNum; j++)
			if (edge[i][j])
				if (v2color[i] == v2color[j])//遇到与自己着色相同且有边相连的，就计数1
					conect[i]++;
	for (i = 0; i < verNum; i++)
	{
		k = v2color[i];
		bian[k] += conect[i];//累加的话会计算两次，需要除以2
	}
	bian[0] /= 2;
	bian[1] /= 2;
	Sc = min(bian[0], bian[1]);
	Sb = Sc;
}

//局部搜索
void local_search()
{
	int delta;
	int temp_delta = -MAX_VAL;
	int swap_delta = -MAX_VAL;
	int i, j, k, m;
	int mark = -1;
	int swap_0 = -1;
	int swap_1 = -1;
	int org_0, org_1;
	//FLIP
	for (i = 0; i < verNum; i++)
	{
		if (tabu[i] > iter)//如果禁忌了，则直接跳过
			continue;

		k = bian[1 - v2color[i]] + adjlen[i] - conect[i];//移入的那一方，快速计算方法，加上与之相连的所有的点，减去与自己颜色原本相同的点，剩下的就是与自己颜色原本不同的点
		m = bian[v2color[i]] - conect[i];//移出去的那一方，减去与自己颜色原本相同的点
		delta = min(k, m) - Sc;//计算delta
		if (delta > temp_delta)//保留最佳flip操作
		{
			temp_delta = delta;
			mark = i;
		}
	}
	//cout << "mark: " << mark+1 << " temp_delta: " << temp_delta << endl;
	if (mark == -1)
	{
		cout << "mark==-1" << endl;
		getchar();
	}
	//SWAPmove
	for (i = 0; i < colorlen[0]; i++)
	{
		org_0 = color[0][i];//0色的点
		if (tabu[org_0] > iter)
			continue;
		for (j = 0; j < colorlen[1]; j++)
		{
			org_1 = color[1][j];//1色的点
			if (tabu[org_1] > iter)//不违反禁忌搜索
				continue;
			k = bian[1] + adjlen[org_0] - conect[org_0] - conect[org_1] - edge[org_0][org_1];//交换之后，需要关注两个点之间是否有边相连，如果有边相连，需要把自身这条边减掉，相当于是多了
			m = bian[0] + adjlen[org_1] - conect[org_1] - conect[org_0] - edge[org_1][org_0];
			delta = min(k, m) - Sc;
			if (delta > swap_delta)//保留最佳swap操作
			{
				swap_delta = delta;
				swap_0 = org_0;
				swap_1 = org_1;
			}
		}

	}
	if (swap_delta > temp_delta)//判断swap好还是flip好
	{
		//cout << "swap" << endl;
		tabu[swap_0] = iter + tt;
		tabu[swap_1] = iter + tt;
		FLIP(swap_0);//对数据结构进行更新，相当于先移过去了，再移过来
		FLIP(swap_1);
		judge_best();//保留最优解

	}
	else
	{
		tabu[mark] = iter + tt;
		FLIP(mark);//对数据结构进行更新
		judge_best();
	}

}


//翻转操作
void FLIP(int n)
{
	int org_color;
	int org_idx;
	int i, j, k, m;
	org_color = v2color[n];//着的颜色
	org_idx = v2idx[n];//第几个

	color[1 - org_color][colorlen[1 - org_color]] = n;//着色
	v2color[n] = 1 - org_color;
	v2idx[n] = colorlen[1 - org_color];
	colorlen[1 - org_color]++;//颜色数量加1
	bian[org_color] -= conect[n];
	conect[n] = adjlen[n] - conect[n];//补集
	bian[1 - org_color] += conect[n];//改变两个bian

	colorlen[org_color]--;//颜色数量减1
	m = color[org_color][colorlen[org_color]];//最后一个补上去
	color[org_color][org_idx] = m;
	v2idx[m] = org_idx;

	//更新conect
	for (i = 0; i < adjlen[n]; i++)//和点有连接的点有变化
	{
		k = adj[n][i];
		m = v2color[k];
		if (m == org_color)//和老的颜色一样
			conect[k]--;
		else//和老的颜色不一样
			conect[k]++;
	}
	Sc = min(bian[0], bian[1]);//更新Sc
}

void judge_best()
{
	if (Sc > Sb)
	{
		Sb = Sc;
		noimprove = 0;
		endTime = clock();//计时
		bestTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
		//compute_obj();
		cout << "iter: " << iter << " Sc: " << Sc << " time: " << bestTime << "s" << endl;
	}
	else
	{
		noimprove++;
		if (noimprove > omega)
		{
			noimprove = 0;
			pertubation();//扰动
		}
	}
}


void show_result()
{
	int i, j, k, m;

	cout << "颜色0[" << colorlen[0] << "]bian0[" << bian[0] << "]: ";
	for (i = 0; i < colorlen[0]; i++)
	{
		cout << color[0][i] + 1 << "[" << conect[color[0][i]] << "] ";
	}
	cout << endl << "颜色1[" << colorlen[1] << "]bian1[" << bian[1] << "]: ";
	for (i = 0; i < colorlen[1]; i++)
	{
		cout << color[1][i] + 1 << "[" << conect[color[1][i]] << "] ";
	}
	cout << endl;

}


void pertubation()//扰动
{
	int i, j, k;

	k = max(int(alpha * verNum), 1);
	//cout <<"iter: "<<iter<< " 扰动前，sc: " << Sc << endl;
	for (i = 0; i < alpha * verNum; i++)
	{
		j = random_int(verNum);//随机进行移动
		FLIP(j);
		//tabu[j] = iter + 0.2 * tt;
		tabu[j] = 0;
	}
	//cout << "iter: " << iter<< "扰动后，sc: " << Sc << endl;
}

//主函数
int main(int argc, char** argv)
{
	int i;
	/*if (argc == 3)
	{
	inFile = argv[1];
	cutting_time = atoi(argv[2]);
	}
	else
	{
	cout << "error: MLCPexp inFile time" << endl;
	getchar();
	exit(0);
	}*/
	srand((unsigned)time(NULL));
	read_initial(inFile);//读取数据
	ofstream outFile(outFile, ios::app);
	cout << inFile << endl << endl;
	//tt = min((int)(0.2 * verNum), 20);
	//cout << "tt: " << tt << endl;

	for (i = 0; i < 10; i++) {
		startTime = clock();//计时开始	
		endTime = startTime;
		random_initial();//随机初始化，纯随机的方式
		//cout << "Sc: " << Sc << endl;
		iter = 0;//迭代次数
		while ((endTime - startTime) / CLOCKS_PER_SEC < cutting_time)
		{
			//show_result();
			iter++;
			local_search();
			endTime = clock();//计时
		}
		outFile << inFile << " " << verNum << " " << edgeNum << " " << Sb << " " << bestTime << endl;
	}
}




