#pragma once
#include "Recast.h"
class RecastData
{
public:
	RecastData();
	~RecastData();

	bool find(size_t x, size_t y, size_t z);

	void setSpan(rcVoexlSpan* s);
	void setCount(size_t count);
	void setSpanIndex(rcVoexlSpanIndex* sIndex);
	void setSceneInfo(size_t realswidth, size_t realslength);
	void setOrg(float* org);
	void setCellHeight(float cellHeight);
	inline size_t getswidth() const { return m_realswidth;}
	inline size_t getslength()const { return m_realslength;}
	inline float  getCellHeight() const { return m_cellHeight; }
	inline const float* getOrg()  const { return m_org; }
private:
	size_t m_realswidth;
	size_t m_realslength;
	size_t m_count;
	float  m_cellHeight;
	rcVoexlSpan* m_vspan;
	rcVoexlSpanIndex* m_vspanIndex;
	float m_org[3];
};