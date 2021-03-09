#include "InvSpan.h"
#include "RecastAlloc.h"
#include <algorithm>
#include <stdlib.h>
#include <ChosePositionTool.h>
#include "InputGeom.h"
size_t rcInvSpan::invspan_count = 0;
InvSpan::InvSpan(Sample_TileMesh* sample):
	m_sample(sample),
	m_invspan(nullptr),
	m_neighborPool(nullptr),
	m_poolQueue(nullptr),
	m_poolSize(0),
	m_mostFlag(0),
	m_flag(0)
{
}

InvSpan::~InvSpan()
{
	if (m_neighborPool)
	{
		delete[] m_neighborPool;
		m_neighborPool = nullptr;
	}
	if (m_poolQueue)
	{
		delete[] m_poolQueue;
		m_poolQueue = nullptr;
	}
}

void InvSpan::doInvSpan()
{
	rcSpan*** span = m_sample->m_ourSpan;
	const rcParam& param   = m_sample->m_rcParam;
	const rcConfig& config = m_sample->m_cfg;

	const float cs = config.cs;
	const float ch = config.ch;

	const int w = config.width  - config.borderSize * 2;
	const int h = config.height - config.borderSize * 2;
	
	rcInvSpan::invspan_count = 0;
	m_invspan = new rcInvSpan**[param.tweight * param.theight];
	memset(m_invspan, 0, sizeof(rcInvSpan**) * param.tweight * param.theight);

	for (size_t tx = 0; tx < param.tweight; ++tx)
	{
		for (size_t ty = 0; ty < param.theight; ++ty)
		{
			size_t tindex = tx + ty * param.tweight;

			m_invspan[tindex] = (rcInvSpan**)rcAlloc(sizeof(rcInvSpan*) * w  * h, RC_ALLOC_PERM);
			memset(m_invspan[tindex], 0, sizeof(rcInvSpan*) * w * h);

			rcSpan** tileSpan = span[tindex];
			for (int x = 0; x < w; ++x)
			{
				for (int y = 0; y < h; ++y)
				{
					const rcSpan* s = tileSpan[x + y * w];
					if (s == nullptr)
					{
						m_invspan[tindex][x + y * w] = new rcInvSpan();
						m_invspan[tindex][x + y * w]->smin = 0;
						m_invspan[tindex][x + y * w]->smax = RC_SPAN_MAX_HEIGHT;
						++rcInvSpan::invspan_count;
					}
					else
					{
						const rcSpan* lastSpan = s;
						//head
						rcInvSpan* curInvSpan = m_invspan[tindex][x + y * w] = new rcInvSpan();
						++rcInvSpan::invspan_count;
						if (s->smin != 0)
						{
							curInvSpan->smin = 0;
							curInvSpan->smax = s->smin;
							curInvSpan = curInvSpan->next = new rcInvSpan();
							++rcInvSpan::invspan_count;
						}
						//middle
						while (s && s->next)
						{
							curInvSpan->smin = s->smax;
							curInvSpan->smax = s->next->smin;
							s = s->next;
							lastSpan = s;
							curInvSpan = curInvSpan->next = new rcInvSpan();
							++rcInvSpan::invspan_count;
						}
						//tail
						if (lastSpan->smax != RC_SPAN_MAX_HEIGHT)
						{
							curInvSpan->smin = lastSpan->smax;
							curInvSpan->smax = RC_SPAN_MAX_HEIGHT;
						}
					}
				}
			}
		}
	}
}
void InvSpan::doInvCheck(rcHeightfield** solid)
{
	doInvSpan();
	doNeighbor();
	doMostFlag();
	doSolidArea(solid);
}

void InvSpan::doNeighbor()
{
	//pool init ,not use position of zero
	if (m_poolQueue) free(m_poolQueue);
	m_poolSize = 0;
	m_poolQueue = (size_t*)malloc(sizeof(size_t) * (m_sample->m_rcParam.swidth * m_sample->m_rcParam.slength + 1)); // size_t[m_sample->m_rcParam.swidth * m_sample->m_rcParam.slength + 1];
	memset(m_poolQueue, 0, sizeof(size_t) * (m_sample->m_rcParam.swidth * m_sample->m_rcParam.slength + 1));
		
	if (m_neighborPool) delete[] m_neighborPool;
	m_neighborPool = new rcInvSpan*[m_sample->m_rcParam.swidth * m_sample->m_rcParam.slength];
	int sx = 0, sz = 0;
	rcInvSpan* first_span = findFirst(sx, sz);
	
	m_flag = 1;
	first_span->flag = m_flag;
	memset(m_neighborPool, 0, sizeof(rcInvSpan*) * m_sample->m_rcParam.swidth * m_sample->m_rcParam.slength);
	//first
	findNeighbor(first_span, sx, sz, 0);
	//other
	int ox = 0 ,oz = 0;
	while (rcInvSpan* popSpan = popPool(ox, oz))
	{
		findNeighbor(popSpan, ox, oz, 0);
	}
	
	delete[] m_neighborPool;
	m_neighborPool = nullptr;

	free(m_poolQueue);
	m_poolQueue = nullptr;
	m_poolSize = 0;
}

void InvSpan::doMostFlag()
{
	size_t* most = (size_t*)malloc(sizeof(size_t) * m_flag); // new size_t[m_flag];
	memset(most, 0, sizeof(size_t) * m_flag);
	int swidth = m_sample->m_rcParam.swidth;
	int slength = m_sample->m_rcParam.slength;
	for (int sx = 0; sx < swidth; ++sx)
	{
		for (int sz = 0; sz < slength; ++sz)
		{
			rcInvSpan* span = findByPos(sx, sz);
			while (span)
			{
				if (span->flag > 0)
				{
					most[span->flag - 1] += 1;
				}
				span = span->next;
			}
		}
	}
	m_mostFlag = static_cast<int>(std::max_element(most, most + m_flag) - most) + 1;
	free(most);
	most = nullptr;
}

void InvSpan::doSolidArea(rcHeightfield** solid)
{
	const rcConfig& config =  m_sample->m_cfg;
	for (size_t th = 0 ; th < m_sample->m_rcParam.theight ; ++th)
	{
		for (size_t tw = 0; tw < m_sample->m_rcParam.tweight; ++tw)
		{
			size_t tindex = tw + th * m_sample->m_rcParam.tweight;
			rcSpan** span = solid[tindex]->spans;
			rcInvSpan** tinvspan  = m_invspan[tindex];
			for (size_t w = config.borderSize; w < config.width - config.borderSize; ++w)
			{
				for (size_t h = config.borderSize; h < config.height - config.borderSize; ++h)
				{
					rcSpan* s = span[w + h * config.width];
					if(s == nullptr) continue;
					rcInvSpan* invs = tinvspan[w - config.borderSize + (h - config.borderSize) * (config.width - 2 * config.borderSize)];
					if(invs == nullptr) continue;

					if (s->smin != 0) invs = invs->next;
					while (s)
					{
						if (s->area == RC_WALKABLE_AREA)
						{
							if (invs)
							{
								if (invs->flag != m_mostFlag || invs->smax - invs->smin < 3)
								{
									s->area = RC_NULL_AREA;
								}
							}
						}
						s = s->next;
						invs = invs->next;
					}
				}
			}
		}
	}
}

void InvSpan::addPool(int sx, int sz, rcInvSpan* span)
{
	size_t index = sx + sz * m_sample->m_rcParam.swidth;

	if (!m_neighborPool[index])
	{
		m_neighborPool[index] = span;
		pushRecord(sx, sz);
	}
	else
	{
		rcInvSpan* s = m_neighborPool[index];
		m_neighborPool[index] = span;
		span->link = s;
	}
}

rcInvSpan* InvSpan::popPool(int& x,int& z)
{
	if (m_poolSize <= 0 || m_poolSize >= m_sample->m_rcParam.swidth *  m_sample->m_rcParam.slength + 1)
	{
		return nullptr;
	}
	rcInvSpan* popSpan = nullptr;
	size_t poolIndex = m_poolQueue[m_poolSize];
	if (m_neighborPool[poolIndex])
	{
		popSpan = m_neighborPool[poolIndex];
		x = static_cast<int>(poolIndex % m_sample->m_rcParam.swidth);
		z = static_cast<int>(poolIndex / m_sample->m_rcParam.swidth);
		m_neighborPool[poolIndex] = m_neighborPool[poolIndex]->link;
		if (!m_neighborPool[poolIndex])
		{
			removeLastRecord();
		}
		return popSpan;
	}
	return nullptr;
}
rcInvSpan* InvSpan::findByPos(int sx, int sz)
{
	if (sx < 0 || sz < 0)
	{
		return nullptr;
	}
	int swidth  = m_sample->m_rcParam.swidth;
	int slength = m_sample->m_rcParam.slength;
	if (sx >= swidth || sz >= slength)
	{
		return nullptr;
	}
	int tx = sx / m_sample->m_rcParam.tsize;
	int ty = sz / m_sample->m_rcParam.tsize;
	rcInvSpan** tileSpan = m_invspan[tx + ty * m_sample->m_rcParam.tweight];
	if (tileSpan == nullptr)
	{
		return nullptr;
	}
	size_t x = sx % m_sample->m_rcParam.tsize;
	size_t z = sz % m_sample->m_rcParam.tsize;
	rcInvSpan* span = tileSpan[x + z * (m_sample->m_cfg.width - m_sample->m_cfg.borderSize * 2)];
	return span;
}

rcInvSpan* InvSpan::findFirst(int& x,int& z)
{
	const float* bmin = m_sample->getInputGeom()->getNavMeshBoundsMin();
	float posx = gBuildbeginPos[0] - bmin[0];
	float posy = gBuildbeginPos[1] - bmin[1];
	float posz = gBuildbeginPos[2] - bmin[2];

	float cellsize  = m_sample->m_cfg.cs;
	float cellheigh = m_sample->m_cfg.ch;

	int swidth  = m_sample->m_rcParam.swidth;
	int slength = m_sample->m_rcParam.slength;

	int index_x = (int)ceil(posx / cellsize);
	int index_z = (int)ceil(posz / cellsize);

	if (index_x < 1 || index_x > swidth) return nullptr;
	if (index_z < 1 || index_z > slength) return nullptr;
	
	--index_x;
	--index_z;
	rcInvSpan* span = findByPos(index_x,index_z);
	while (span)
	{
		if((posy >= span->smin * cellheigh && posy <= span->smax * cellheigh) || posy < span->smin * cellheigh)
		{
			x = index_x;
			z = index_z;
			return span;
		}
		span = span->next;
	}
	return nullptr;
}

void InvSpan::findNeighbor(rcInvSpan* curs,int sx, int sz,bool flag)
{
	for (int i = 0; i < 3 ;++i)
	{
		for (int j = 0 ; j < 3 ;++j)
		{
			int fromx = sx - 1 + i;
			int fromz = sz - 1 + j;
			if (fromx == sx && fromz == sz) continue;
			rcInvSpan* fspan = findByPos(fromx, fromz);
			while (fspan)
			{
				if (fspan->flag == 0)
				{
					if (fspan->smin != curs->smax && curs->smin != fspan->smax)
					{
						if ((fspan->smax <= curs->smax && fspan->smax >= curs->smin) ||
							(fspan->smin <= curs->smax && fspan->smin >= curs->smin) ||
							(curs->smin <= fspan->smax && curs->smin >= fspan->smin) ||
							(curs->smin <= fspan->smax && curs->smin >= fspan->smin))
						{
							fspan->flag = m_flag;
							addPool(fromx, fromz, fspan);
						}
					}
				}
				fspan = fspan->next;
			}
		}
	}
}

void InvSpan::pushRecord(int x, int z)
{
	if (m_poolSize + 1 >= m_sample->m_rcParam.swidth *  m_sample->m_rcParam.slength + 1)
	{
		return;
	}
	m_poolQueue[++m_poolSize] = x + z * m_sample->m_rcParam.swidth;
}

void InvSpan::removeLastRecord()
{
	m_poolQueue[m_poolSize--] = 0;
}
