// MLCP.cpp : �������̨Ӧ�ó������ڵ㡣
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

//�ð汾������swap������tt�̶�15��alpha0.01��omega1000����exp���omega���ˣ�tt����
using namespace std;
#define MAX_VAL 999999

clock_t startTime, endTime;//��¼����ʱ��
double bestTime;//���ʱ��
double cutting_time = 180;//��ֹ����
string inFile = "C:/Users/18367/Documents/Visual Studio 2015/Projects/MLCP/Debug/data/anna.col";
string outFile = "C:/Users/18367/Documents/Visual Studio 2015/Projects/MLCP/Debug/data/experiment_tt20_swap_exp4.txt";


//��Ҫ���ڵĲ���
int tt = 2;//���ɳ���
int omega = 500;//�Ŷ�����ֵ
float alpha = 0.01;//�Ŷ�ǿ��
int noimprove = 0;//����δ���µĴ���

int Sc;
int Sb;

int edgeNum, verNum;
int** edge;//������Ƿ��б�
int** adj;//��¼��ÿ�ܷɻ��г�ͻ�ķɻ�
int* adjlen;//��¼��ÿ�ܷɻ���ͻ�ɻ�������

//��Ҫ���ݽṹ
int** color;
int* colorlen;
int* v2color;//�����ɫ
int* v2idx;
int* conect;//��������ɫ�ı�
int* bian;//������ɫ�бߵ�����

//�ֲ�����
int* tabu;//���ɱ�
int iter = 0;//��������

//��������
int random_int(int);
void read_initial(string);
void random_initial();//���������ʼ��
void compute_obj();//����Ŀ��ֵ
void FLIP(int n);//���ɻ�flt�ӵ�ǰgate�ƶ���gt
void show_result();
void local_search();
void judge_best();
void pertubation();//�Ŷ�


//�������һ�� 0-(n-1) ������
int random_int(int n)
{
	return rand() % n;
}

//��ȡ����
void read_initial(string inFile)
{
	int i, j, k;
	ifstream FIC;
	FIC.open(inFile);//��ȡ����
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
	edge = new int*[verNum];//��¼������������
	adj = new int*[verNum];//��¼������Щ������
	color = new int*[2];//��¼ÿ����ɫ������Щ��
	adjlen = new int[verNum];//������Щ�������ĳ���
	tabu = new int[verNum];
	v2color = new int[verNum];
	v2idx = new int[verNum];
	conect = new int[verNum];
	colorlen = new int[2];//������ɫ��ɫ�ĵ�������
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
		edge[a][b] = edge[b][a] = 1;//��������б�����

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

	FIC.close();//�ر�����
}

//���������ʼ��
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
		v2color[i] = k;//�����ɫ
		v2idx[i] = colorlen[k];//�ǵڼ����������ɫ��
		colorlen[k]++;
	}
	for (i = 0; i < verNum; i++)
		for (j = 0; j < verNum; j++)
			if (edge[i][j])
				if (v2color[i] == v2color[j])//�������Լ���ɫ��ͬ���б������ģ��ͼ���1
					conect[i]++;
	for (i = 0; i < verNum; i++)
	{
		k = v2color[i];
		bian[k] += conect[i];//�ۼӵĻ���������Σ���Ҫ����2
	}
	bian[0] /= 2;
	bian[1] /= 2;
	Sc = min(bian[0], bian[1]);
	Sb = Sc;
}

//�ֲ�����
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
		if (tabu[i] > iter)//��������ˣ���ֱ������
			continue;

		k = bian[1 - v2color[i]] + adjlen[i] - conect[i];//�������һ�������ټ��㷽����������֮���������еĵ㣬��ȥ���Լ���ɫԭ����ͬ�ĵ㣬ʣ�µľ������Լ���ɫԭ����ͬ�ĵ�
		m = bian[v2color[i]] - conect[i];//�Ƴ�ȥ����һ������ȥ���Լ���ɫԭ����ͬ�ĵ�
		delta = min(k, m) - Sc;//����delta
		if (delta > temp_delta)//�������flip����
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
		org_0 = color[0][i];//0ɫ�ĵ�
		if (tabu[org_0] > iter)
			continue;
		for (j = 0; j < colorlen[1]; j++)
		{
			org_1 = color[1][j];//1ɫ�ĵ�
			if (tabu[org_1] > iter)//��Υ����������
				continue;
			k = bian[1] + adjlen[org_0] - conect[org_0] - conect[org_1] - edge[org_0][org_1];//����֮����Ҫ��ע������֮���Ƿ��б�����������б���������Ҫ�����������߼������൱���Ƕ���
			m = bian[0] + adjlen[org_1] - conect[org_1] - conect[org_0] - edge[org_1][org_0];
			delta = min(k, m) - Sc;
			if (delta > swap_delta)//�������swap����
			{
				swap_delta = delta;
				swap_0 = org_0;
				swap_1 = org_1;
			}
		}

	}
	if (swap_delta > temp_delta)//�ж�swap�û���flip��
	{
		//cout << "swap" << endl;
		tabu[swap_0] = iter + tt;
		tabu[swap_1] = iter + tt;
		FLIP(swap_0);//�����ݽṹ���и��£��൱�����ƹ�ȥ�ˣ����ƹ���
		FLIP(swap_1);
		judge_best();//�������Ž�

	}
	else
	{
		tabu[mark] = iter + tt;
		FLIP(mark);//�����ݽṹ���и���
		judge_best();
	}

}


//��ת����
void FLIP(int n)
{
	int org_color;
	int org_idx;
	int i, j, k, m;
	org_color = v2color[n];//�ŵ���ɫ
	org_idx = v2idx[n];//�ڼ���

	color[1 - org_color][colorlen[1 - org_color]] = n;//��ɫ
	v2color[n] = 1 - org_color;
	v2idx[n] = colorlen[1 - org_color];
	colorlen[1 - org_color]++;//��ɫ������1
	bian[org_color] -= conect[n];
	conect[n] = adjlen[n] - conect[n];//����
	bian[1 - org_color] += conect[n];//�ı�����bian

	colorlen[org_color]--;//��ɫ������1
	m = color[org_color][colorlen[org_color]];//���һ������ȥ
	color[org_color][org_idx] = m;
	v2idx[m] = org_idx;

	//����conect
	for (i = 0; i < adjlen[n]; i++)//�͵������ӵĵ��б仯
	{
		k = adj[n][i];
		m = v2color[k];
		if (m == org_color)//���ϵ���ɫһ��
			conect[k]--;
		else//���ϵ���ɫ��һ��
			conect[k]++;
	}
	Sc = min(bian[0], bian[1]);//����Sc
}

void judge_best()
{
	if (Sc > Sb)
	{
		Sb = Sc;
		noimprove = 0;
		endTime = clock();//��ʱ
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
			pertubation();//�Ŷ�
		}
	}
}


void show_result()
{
	int i, j, k, m;

	cout << "��ɫ0[" << colorlen[0] << "]bian0[" << bian[0] << "]: ";
	for (i = 0; i < colorlen[0]; i++)
	{
		cout << color[0][i] + 1 << "[" << conect[color[0][i]] << "] ";
	}
	cout << endl << "��ɫ1[" << colorlen[1] << "]bian1[" << bian[1] << "]: ";
	for (i = 0; i < colorlen[1]; i++)
	{
		cout << color[1][i] + 1 << "[" << conect[color[1][i]] << "] ";
	}
	cout << endl;

}


void pertubation()//�Ŷ�
{
	int i, j, k;

	k = max(int(alpha * verNum), 1);
	//cout <<"iter: "<<iter<< " �Ŷ�ǰ��sc: " << Sc << endl;
	for (i = 0; i < alpha * verNum; i++)
	{
		j = random_int(verNum);//��������ƶ�
		FLIP(j);
		//tabu[j] = iter + 0.2 * tt;
		tabu[j] = 0;
	}
	//cout << "iter: " << iter<< "�Ŷ���sc: " << Sc << endl;
}

//������
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
	read_initial(inFile);//��ȡ����
	ofstream outFile(outFile, ios::app);
	cout << inFile << endl << endl;
	//tt = min((int)(0.2 * verNum), 20);
	//cout << "tt: " << tt << endl;

	for (i = 0; i < 10; i++) {
		startTime = clock();//��ʱ��ʼ	
		endTime = startTime;
		random_initial();//�����ʼ����������ķ�ʽ
		//cout << "Sc: " << Sc << endl;
		iter = 0;//��������
		while ((endTime - startTime) / CLOCKS_PER_SEC < cutting_time)
		{
			//show_result();
			iter++;
			local_search();
			endTime = clock();//��ʱ
		}
		outFile << inFile << " " << verNum << " " << edgeNum << " " << Sb << " " << bestTime << endl;
	}
}




