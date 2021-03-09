#pragma once
#include "Sample_TileMesh.h"
class ExcludeSpan;
class InvSpan 
{
public:
	InvSpan(Sample_TileMesh* sample);
	~InvSpan();
public:
	void doInvCheck(rcHeightfield** solid);
	rcInvSpan*** getInvSpan(){ return m_invspan;}

private:
	void doInvSpan();
	void doNeighbor();
	void doMostFlag();
	void doSolidArea(rcHeightfield** solid);
	void findNeighbor(rcInvSpan* curs , int x, int z,bool flag);
	rcInvSpan* findByPos(int sx, int sz);
	rcInvSpan* findFirst(int& x, int& z);
private:
	void addPool(int sx, int sz, rcInvSpan* span);
	rcInvSpan* popPool(int& x, int& z);
	void   pushRecord(int x, int z);
	void   removeLastRecord();
private:
	Sample_TileMesh* m_sample;
	rcInvSpan*** m_invspan;
	rcInvSpan**  m_neighborPool;
	size_t* m_poolQueue;
	size_t  m_poolSize;
	int m_flag;
	int m_mostFlag;
};