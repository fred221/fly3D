#include "RecastData.h"
#include <cstring>
RecastData::RecastData():
	m_realswidth(0),
	m_realslength(0),
	m_count(0),
	m_vspan(nullptr),
	m_vspanIndex(nullptr)
{
	memset(m_org, 0, sizeof(float) * 3);
}

RecastData::~RecastData()
{
	if (m_vspan)
	{
		delete[] m_vspan;
		m_vspan = nullptr;
	}

	if (m_vspanIndex)
	{
		delete[] m_vspanIndex;
		m_vspanIndex = nullptr;
	}
}

bool RecastData::find(size_t x, size_t y, size_t z)
{
	if (x >= m_realswidth || z >= m_realslength)
	{
		return false;
	}
	size_t index = x + z * m_realswidth;
	if (m_vspanIndex[index].begin == 0 && m_vspanIndex[index].end == 0)
	{
		return true;
	}
	size_t from = m_vspanIndex[index].begin - 1 ;
	size_t to   = m_vspanIndex[index].end;
	for (size_t i = from; i < to; ++i)
	{
		static float diff = VOXEL_SIZE / 2.0f ;
		float min = m_vspan[i].smin * m_cellHeight - diff; //+ m_org[1]
		float max = m_vspan[i].smax * m_cellHeight + diff; //+ m_org[1]

		min = min < 0.0f ? 0.0f : min;
		max = max < 0.0f ? 0.0f : max;

  		if (y >= min / VOXEL_SIZE && y <= max / VOXEL_SIZE)
		{
			return false;
		}
	}
	return true;
}

void RecastData::setSpan(rcVoexlSpan* s)
{
	if (m_vspan)
	{
		delete[] m_vspan;
	}
	m_vspan = s;
}

void RecastData::setCount(size_t count)
{
	m_count = count;
}

void RecastData::setSpanIndex(rcVoexlSpanIndex* sIndex)
{
	if (m_vspanIndex)
	{
		delete[] m_vspanIndex;
	}
	m_vspanIndex = sIndex;
}

void RecastData::setSceneInfo(size_t realswidth, size_t realslength)
{
	m_realswidth  = realswidth; 
	m_realslength = realslength;
}

void RecastData::setOrg(float* org)
{
	rcVcopy(m_org,org);
}

void RecastData::setCellHeight(float cellHeight)
{
	m_cellHeight = cellHeight;
}
 