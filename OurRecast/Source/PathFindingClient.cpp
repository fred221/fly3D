#define EXPORTBUILD  
#include "PathFindingClient.h"
#include <string.h>

#define CLIENT_STAGE_INFO  ("1") 
#define CLIENT_STAGE_HIGHT_MAX  (1) 

extern float m_polyPickExt[];
extern bool mIsFromClient;
bool  LoadNavData(unsigned char* data)
{
	mIsFromClient = true;
	UnLoadNavData();

	SetPolyPickExtern(0.1, 500, 0.1);

	//客户端 不读取 体素 文件 
	loadNavMesh(CLIENT_STAGE_INFO, CLIENT_STAGE_HIGHT_MAX, data,false);
	mOurBaseSampleBin = new OurSample_TileBin(1);
	
	if (!mOurBaseSampleBin->load(CLIENT_STAGE_INFO,mOurBaseSampleBin->getRcContext()))
	{
		std::cerr << "LoadNavData faild" << std::endl;
		return false;
	}
 
	return true;
}
void UnLoadNavData()
{
	if (mOurBaseSampleBin != NULL)
	{
		delete mOurBaseSampleBin;
		mOurBaseSampleBin = NULL;
	}

	if (!navMeshs.empty())
	{
		for (NavMeshsMap::iterator it = navMeshs.begin(); it != navMeshs.end(); ++it)
		{
			dtFreeNavMesh(it->second);
		}
		navMeshs.clear();
	}

	//删除体素文件
	destoryAllVoxel();
}
bool FindNavPath(float* start, float* end, int jflag, float*  &outarr, int &len)
{
	try
	{
		outarr = new float[MAX_SMOOTH * 3];
		bool isFind = findPath(mOurBaseSampleBin, start, end, jflag /*SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED*/, outarr, &len);
		return isFind;
	}
	catch (...)
	{
		std::cerr << "findPath  error" << std::endl;

	}
	return false; 
}
bool Raycast(float* start, float* end, int jflag, float* &outarr, float* &normalDir)
{
	try
	{
		normalDir = new float[3];
		outarr = new float[3];
		memset(normalDir, 0, sizeof(float) * 3);
		memset(outarr, 0, sizeof(float) * 3);

		bool isRayCast = raycast(mOurBaseSampleBin, start, end, jflag, outarr, normalDir);
		if (isRayCast)
		{
			//oNormalDir = new float[3];
			//memcpy(oNormalDir, normalDir, sizeof(float) * 3);
		}
		return isRayCast;
	}
	catch (...)
	{
		std::cerr << "Raycast  error" << std::endl;
	}
	return false;
}
bool IsPosInBlock(float* pos)
{
	try
	{
		bool isBlock = IsPosInBlock(mOurBaseSampleBin, pos);
		return isBlock;
	}
	catch (...)
	{
		std::cerr << "Raycast  error" << std::endl;
	}
	return false;
}
void ClearIntPtr(void* pBuffer)
{
	if (NULL != pBuffer)
	{
		delete pBuffer;
		pBuffer = NULL;
	}
}
void ClearArrayIntPtr(void* pBuffer)
{
	if (NULL != pBuffer)
	{
		delete[] pBuffer;
		pBuffer = NULL;
	}
}
void FindNearestPoly(float* start, float* end, int flag, int &nearRef, float* &pt)
{
	if (mOurBaseSampleBin == NULL)
	{
		std::cerr << "mOurBaseSampleBin is NULL" << std::endl;
		return;
	}

	if (mOurBaseSampleBin->getNavMeshQuery() == NULL)
	{
		std::cerr << "mOurBaseSampleBin getNavMeshQuery is NULL" << std::endl;
		return; 
	}
	float spos[3];
	dtVcopy(spos, start);
	dtQueryFilter m_filter;
	m_filter.setIncludeFlags((unsigned int)flag);
	m_filter.setExcludeFlags(0xffff ^ flag);
	//dtPolyRef m_startRef = 0;
	pt = new float[3];
	mOurBaseSampleBin->getNavMeshQuery()->findNearestPoly(spos, m_polyPickExt, &m_filter, (dtPolyRef*)&nearRef, pt);
}
bool IsWalkable(float* start, int flag)
{
	try
	{
		if (mOurBaseSampleBin == NULL)
		{
			std::cerr << "mOurBaseSampleBin is NULL" << std::endl;
			return false;
		}

		if (mOurBaseSampleBin->getNavMeshQuery() == NULL)
		{
			std::cerr << "mOurBaseSampleBin getNavMeshQuery is NULL" << std::endl;
			return false;
		}

		float m_spos[3];
		dtVcopy(m_spos, start);
		dtQueryFilter m_filter;
		m_filter.setIncludeFlags((unsigned int)flag);
		m_filter.setExcludeFlags(0xffffffff ^ flag);
		dtPolyRef m_startRef = 0;
		float nearestPt[3] = { 0 };

		mOurBaseSampleBin->getNavMeshQuery()->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef,nearestPt);
		for (short int index = 0; index < 3; ++index)
		{
			//高度不准忽略
			if (index == 1)
			{
				continue;
			}
			if (fabsf(nearestPt[index] - m_spos[index]) > 1e-6f)
			{
				return false;
			}
		}
		if (m_startRef == 0)
		{
			//dtLog("IsWalkable startref==0");
			return false;
		}
		return mOurBaseSampleBin->getNavMeshQuery()->isValidPolyRef(m_startRef, &m_filter);
	}
	catch (...)
	{
		std::cerr << "IsWalkable error" << std::endl;
	}
	return false; 
}
float GetPolyHeight(float* point, int flag)
{
	try
	{
		float result[3] = { 0 };
		bool isHight = posHeight(mOurBaseSampleBin, point, result);
		if (isHight) {
			return result[1];
		}
	}
	catch (...)
	{
		std::cerr << "GetPolyHeight error" << std::endl;
	}
	
	return 0.0f;
}
void SetPolyPickExtern(float x, float y, float z)
{
	m_polyPickExt[0] = x;
	m_polyPickExt[1] = y;
	m_polyPickExt[2] = z;
}
bool NearestByHightPoly(float* pos, float*  &nearPos)
{
	nearPos = new float[3];
	bool isFind = false ;
	bool isSuccess = findNearestByHightPoly(mOurBaseSampleBin,pos, nearPos, isFind);

	return (isSuccess && isFind);
}
void GetNavMeshVertices(float*& outArray, int& len)
{
	int arrayLen = 0;
	for (int n = 0; n < mOurBaseSampleBin->getNavMesh()->getMaxTiles(); ++n)
	{
		const dtMeshTile* tile = mOurBaseSampleBin->getNavMesh()->getTile(n);
		if (!tile->header) continue;
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			const dtPoly* p = &tile->polys[i];
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
				continue;
			const dtPolyDetail* pd = &tile->detailMeshes[i];

			arrayLen += pd->triCount * 3 * 3;
		}
	}

	len = arrayLen;
	outArray = new float[len];
	arrayLen = 0;
	for (int n = 0; n < mOurBaseSampleBin->getNavMesh()->getMaxTiles(); ++n)
	{
		const dtMeshTile* tile = mOurBaseSampleBin->getNavMesh()->getTile(n);
		if (!tile->header) continue;
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			const dtPoly* p = &tile->polys[i];
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
				continue;
			const dtPolyDetail* pd = &tile->detailMeshes[i];

			for (int j = 0; j < pd->triCount; ++j)
			{
				const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
				const float* points;
				for (int k = 0; k < 3; ++k)
				{
					if (t[k] < p->vertCount)
					{
						points = &tile->verts[p->verts[t[k]] * 3];
						outArray[arrayLen++] = points[0];
						outArray[arrayLen++] = points[1];
						outArray[arrayLen++] = points[2];
					}
					else
					{
						points = &tile->detailVerts[(pd->vertBase + t[k] - p->vertCount) * 3];
						outArray[arrayLen++] = points[0];
						outArray[arrayLen++] = points[1];
						outArray[arrayLen++] = points[2];
					}
				}
			}
		}
	}
}