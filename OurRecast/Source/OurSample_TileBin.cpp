#include "OurSample_TileBin.h"
#include "OurBaseSample.h"
#include <iostream>
#include "DetourNavMeshQuery.h"
#include <string.h>
#include <stdio.h>
#include <fstream>
#include "NavMesh.h"
#include <time.h>
static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;


struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
}; 

OurSample_TileBin::OurSample_TileBin(long long stageInfo):
	OurBaseSample(stageInfo)
{
	
}

OurSample_TileBin::~OurSample_TileBin()
{

}
bool OurSample_TileBin::load(std::string name ,rcContext* ctx)
{
	auto&& it = navMeshs.find(name);
	if (it != navMeshs.end())
	{
		m_navMesh = it->second;
	}
	
	if (!m_navMesh)
	{
		std::cerr << "不能创建寻路查询对象Could not create Detour navMeshs, stageInfo:" << std::endl;
		return false;
	}

	dtLog("test20");
	// 初始化查询对象
	m_navQuery = dtAllocNavMeshQuery();
	if (!m_navQuery)
	{
		std::cerr << "不能创建寻路查询对象Could not create Detour navMeshQuery, stageInfo:" << std::endl;
		return false;
	}
	// 初始化寻路查询对象，如果错误 直接返回
	dtLog("test21");
	if (dtStatusFailed(m_navQuery->init(m_navMesh, 2048)))
	{
		std::cerr << "不能初始化查询对象Could not init Detour navMesh query, stageInfo:" << std::endl;
		return false;
	}
	dtLog("test22");
	return true;
}

void OurSample_TileBin::handleSettings()
{

}

bool OurSample_TileBin::handleBuild()
{
	return true;
}

void OurSample_TileBin::collectSettings(struct BuildSettings& settings)
{

}

dtNavMesh* OurSample_TileBin::dtLoadNavMesh(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		fread(&tileHeader, sizeof(tileHeader), 1, fp);

		// dtNavMeshHeaderSwapEndian((unsigned char*)&tileHeader,sizeof(NavMeshTileHeader));

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		fread(data, tileHeader.dataSize, 1, fp);

		// dtNavMeshDataSwapEndian(data,tileHeader.dataSize);

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}
//客户端只能读取 assesBundle 文件
dtNavMesh* dtLoadNavMeshBundle(std::string& name,int sceneHighMax ,unsigned char* data)
{
	NavMeshSetHeader header;

	const unsigned char* pos = data;
	memcpy(&header, pos, sizeof(NavMeshSetHeader));

	pos += sizeof(NavMeshSetHeader);
	if (header.magic != NAVMESHSET_MAGIC)
	{
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		return 0;
	}

	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		memcpy(&tileHeader, pos, sizeof(NavMeshTileHeader));

		pos += sizeof(NavMeshTileHeader);

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data)
		{
			break;
		}
		memset(data, 0, tileHeader.dataSize);
		memcpy(data, pos, tileHeader.dataSize);

		pos += tileHeader.dataSize;
		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}
	return mesh;
}