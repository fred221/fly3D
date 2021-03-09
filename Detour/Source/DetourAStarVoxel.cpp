#include "DetourAStarVoxel.h"
#include "../../Recast/Include/Recast.h"
#include <cstring>

std::unordered_map<std::string, AStarVoxel*> allAStarVoxelMap;
AStarVoxel::AStarVoxel()
{
	mAStarVoxelData = new RecastData();
}

AStarVoxel::~AStarVoxel()
{
	if (mAStarVoxelData)
	{
		delete mAStarVoxelData;
		mAStarVoxelData = nullptr ;
	}
}

bool AStarVoxel::loadAstarData(const unsigned char* pos)
{
	if (pos == nullptr) return false;

	float org[3] = { 0 };
	memcpy(&org, pos, sizeof(float) * 3);
	pos += sizeof(float) * 3;

	int realwith = 0;
	memcpy(&realwith, pos, sizeof(int));
	pos += sizeof(int);

	int realslength = 0;
	memcpy(&realslength, pos, sizeof(int));
	pos += sizeof(int);

	rcVoexlSpanIndex* vspanIndex = new rcVoexlSpanIndex[realwith * realslength];
	memcpy(vspanIndex, pos, sizeof(rcVoexlSpanIndex) * realwith * realslength);
	pos += sizeof(rcVoexlSpanIndex) * realwith * realslength;

	size_t count = 0;
	memcpy(&count, pos, sizeof(size_t));
	pos += sizeof(size_t);

	rcVoexlSpan* vspan = new rcVoexlSpan[count];
	memcpy(vspan, pos, sizeof(rcVoexlSpan) * count);
	pos += sizeof(rcVoexlSpan) * count;

	float cellheight = 0.0f;
	memcpy(&cellheight, pos, sizeof(float));

	mAStarVoxelData->setOrg(org);
	mAStarVoxelData->setSpan(vspan);
	mAStarVoxelData->setSpanIndex(vspanIndex);
	mAStarVoxelData->setCount(count);
	mAStarVoxelData->setSceneInfo(realwith, realslength);
	mAStarVoxelData->setCellHeight(cellheight);

	return true;
}

astar_long AStarVoxel::getIndex(astar_long x, astar_long y , astar_long z) const
{
	return x + z * mAStarVoxelData->getswidth() + y * mAStarVoxelData->getswidth() * mAStarVoxelData->getslength();
}

bool AStarVoxel::checkArea(astar_long x, astar_long y, astar_long z) const
{
	if (mAStarVoxelData->find(x, y, z))
	{
		return true;
	}
	return false;
}

bool AStarVoxel::checkArea(astar_long index) const
{
	astar_long xz = mAStarVoxelData->getswidth() * mAStarVoxelData->getslength();
	astar_long y = index / xz;
	astar_long z = index % xz / mAStarVoxelData->getswidth();
	astar_long x = index % xz % mAStarVoxelData->getswidth();

	return checkArea(x, y, z);
}

//--------------------------------globe func--------------------------------------------------- 
const AStarVoxel* getAStarVoxelByName(std::string& name)
{
	auto&& it = allAStarVoxelMap.find(name);
	if (it == allAStarVoxelMap.end())
	{
		return nullptr;
	}
	return it->second;
}
void loadAStarVoxel(std::string& name,const unsigned char* pos, bool isDeleteAll /*= false*/)
{
	if (isDeleteAll)
	{
		destoryAllVoxel();
	}
	if (allAStarVoxelMap.find(name) != allAStarVoxelMap.end())
	{
		return;
	}
	AStarVoxel* voxel = new AStarVoxel();
	voxel->loadAstarData(pos);
	allAStarVoxelMap.insert(std::make_pair(name, voxel));
}

void destoryAllVoxel()
{
	for (auto&& it : allAStarVoxelMap)
	{
		delete it.second;
		it.second = nullptr;
	}
	allAStarVoxelMap.clear();
}
