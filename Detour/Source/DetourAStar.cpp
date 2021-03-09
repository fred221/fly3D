#include "DetourAStar.h"
#include <assert.h>
#include <algorithm>
#include <string.h>
#include "DetourCommon.h"
#include "DetourAStarVoxel.h"
#define CHECK_VOXEL_NULL   if (mAStarVoxel == nullptr) return nullptr;
#define CHECK_VOXEL_VOID   if (mAStarVoxel == nullptr) return ;
#define CHECK_VOXEL_FALSE  if (mAStarVoxel == nullptr) return false ;

bool sortFun(const AstarCellDoing* curCell, const AstarCellDoing* otherCell) 
{
	return curCell->getF() < otherCell->getF();
}

AstarCellDoing::AstarCellDoing()
{
	rest(); 
}
AstarCellDoing::~AstarCellDoing()
{
	rest();
}
void AstarCellDoing::setParentDoingIndex(astar_long index ) const
{
	mParentIndex = index + 1;
}
astar_long AstarCellDoing::getParentDoingIndex() const
{
	return mParentIndex;
}
void AstarCellDoing::resetDir(const AStarVoxel* astarVoxel)
{
	astar_long index = getIndex();
	astar_long xz = astarVoxel->getRecastData()->getswidth() * astarVoxel->getRecastData()->getslength();
	mDir.mY = static_cast<astar_short>(index / xz);
	mDir.mZ = static_cast<astar_short>((index % xz) / astarVoxel->getRecastData()->getswidth());
	mDir.mX = static_cast<astar_short>((index % xz) % astarVoxel->getRecastData()->getswidth());
}
const void AstarCellDoing::rest() const
{
	mParentIndex = 0;
	mF = 0;
	mIndex = 0;
}
//=========================================================================================
DetourAstar::DetourAstar()
{
	clear();
	mOpenList.reserve(1000);
	mAStarDoingPool = new AstarCellDoing[DOING_ARRAY_MAX];
}
DetourAstar::~DetourAstar()
{
	clear();
	delete[] mAStarDoingPool;
	mAStarDoingPool = nullptr;
}
void DetourAstar::clear()
{
	mOpenList.clear();
	mCloseSet.clear(); 
	mAStarDoingMap.clear();
	mAStarVoxel = nullptr;
	mPoolIndex = 0;
} 
bool DetourAstar::initDetourAstar(std::string& name)
{
	clear();
	mAStarVoxel = getAStarVoxelByName(name);
	if (!mAStarVoxel) return false;;
	return true;
}

const AstarCellDoing* DetourAstar::getCellDoing(float* beginPos) const
{
	const float* orgPos = mAStarVoxel->getRecastData()->getOrg();
	astar_int xBegin = static_cast<astar_int>((beginPos[0] - orgPos[0]) / VOXEL_SIZE);
	astar_int yBegin = static_cast<astar_int>((beginPos[1] - orgPos[1]) / VOXEL_SIZE);
	astar_int zBegin = static_cast<astar_int>((beginPos[2] - orgPos[2]) / VOXEL_SIZE);

	const AstarCellDoing* startCell = getCellDoing(xBegin, yBegin, zBegin);
	return startCell;
}
const AstarCellDoing* DetourAstar::getCellDoing(astar_int x, astar_int y, astar_int z, bool isBuild ) const
{
	CHECK_VOXEL_NULL
	if (x < 0 || y < 0 || z < 0)
	{
		return nullptr;
	}
	if (x > mAStarVoxel->getRecastData()->getswidth() || z > mAStarVoxel->getRecastData()->getslength())
	{
		return nullptr;
	}
	astar_long realIndex = mAStarVoxel->getIndex(x, y, z);
	return getCellDoing(realIndex);
}
const AstarCellDoing* DetourAstar::getCellDoing(astar_long index) const
{
	if (mAStarDoingMap[index] == nullptr)
	{
		if (mPoolIndex >= 0 && mPoolIndex < DOING_ARRAY_MAX)
		{
			AstarCellDoing* doing = &mAStarDoingPool[mPoolIndex++];
			doing->setIndex(index);
			doing->resetDir(mAStarVoxel);
			mAStarDoingMap[index] = doing;
		}
	}
	return mAStarDoingMap[index];
}
bool DetourAstar::checkArea(const AstarCellDoing* doing) const
{
	if (doing == nullptr)
	{
		return false;
	}
	return mAStarVoxel->checkArea(doing->getIndex());
}

bool DetourAstar::checkArea(astar_int x, astar_int y, astar_int z) const
{
	const AstarCellDoing* doing = getCellDoing(x, y, z);
	if (doing == nullptr)
	{
		return false;
	}
	return checkArea(doing);
}

bool DetourAstar::rayFly(float* start, float* end, float* hit) const
{
	memset(hit, 0, sizeof(float) * 3);
	
	float startPos[3] = { 0 };
	float endPos[3]   = { 0 };

	dtVcopy(startPos, start);
	dtVcopy(endPos, end);

	const AstarCellDoing* startDoing = getCellDoing(startPos);
	if (startDoing == nullptr)
	{
		return false;
	}

	float dir[3] = { 0 };
	dtVsub(dir, endPos, startPos);

	dtVnormalize(dir);
	float distanceMove[3] = { 0 };
	dtVscale(distanceMove, dir, 1);
	//最多只找100 个格子
	float beforPos[3] = { 0 };
	dtVcopy(beforPos, startPos);

	const float* org = mAStarVoxel->getRecastData()->getOrg();
	for (int index = 0; index < 500; ++index)
	{
		const AstarCellDoing* startDoing = getCellDoing(startPos);
		if (startDoing == nullptr)
		{
			hit[0] = startPos[0] ;
			hit[1] = startPos[1] ;
			hit[2] = startPos[2] ;

			return true;
		}
		else if(!checkArea(startDoing))
		{
			//中心点
			float centerX = startDoing->getDir().mX * VOXEL_SIZE + org[0] + (float)VOXEL_SIZE / 2;
			float centerY = startDoing->getDir().mY * VOXEL_SIZE + org[1] + (float)VOXEL_SIZE / 2;
			float centerZ = startDoing->getDir().mZ * VOXEL_SIZE + org[2] + (float)VOXEL_SIZE / 2;

			float border[6] = { 0 };
			//maxX
			border[0] = centerX + static_cast<float>(VOXEL_SIZE) / 2;
			//minX
			border[1] = centerX - static_cast<float>(VOXEL_SIZE) / 2;
			//maxY
			border[2] = centerY + static_cast<float>(VOXEL_SIZE) / 2;
			//minY
			border[3] = centerY - static_cast<float>(VOXEL_SIZE) / 2;
			//maxZ
			border[4] = centerZ + static_cast<float>(VOXEL_SIZE) / 2;
			//mixZ
			border[5] = centerZ - static_cast<float>(VOXEL_SIZE) / 2;
			
			float targetPos[18] = { 0 };
			float* minDisTarget = nullptr;

			for (int index = 0; index < 18;)
			{
				float move = (border[index / 3]  - start[index / 3 / 2]) / dir[index / 3 / 2];

				targetPos[index++] = move * dir[0] + start[0];
				targetPos[index++] = move * dir[1] + start[1];
				targetPos[index++] = move * dir[2] + start[2];

				// x 轴范围内
				if (targetPos[index - 3] <= border[0] && targetPos[index - 3] >= border[1] &&
					// y轴范围内
					targetPos[index - 2] <= border[2] && targetPos[index - 2] >= border[3] &&
					// z轴范围内
					targetPos[index - 1] <= border[4] && targetPos[index - 1] >= border[5])
				{
					if(minDisTarget == nullptr)
					{
						minDisTarget = &targetPos[index - 3];
					}
					else
					{
						if(dtVdist(start,&targetPos[index - 3]) < dtVdist(start, minDisTarget))
						{
							minDisTarget = &targetPos[index - 3];
						}
					}
				}
			}
			if (minDisTarget)
			{
				hit[0] = *(minDisTarget);
				hit[1] = *(minDisTarget + 1);
				hit[2] = *(minDisTarget + 2);
				return true;
			}
		}
		dtVcopy(beforPos, startPos);
		dtVadd(startPos, startPos, distanceMove);
	}
	return false;
}
bool DetourAstar::correctPos(float* fromPos,float* toPos)
{
	const float* org = mAStarVoxel->getRecastData()->getOrg();
	fromPos[0] -= org[0];
	fromPos[1] -= org[1];
	fromPos[2] -= org[2];

	toPos[0] -= org[0];
	toPos[1] -= org[1];
	toPos[2] -= org[2];

	float dir[3] = { 0 };
	dtVsub(dir, toPos, fromPos);
	dtVnormalize(dir);

	float checkMax[3] = { static_cast<float>(mAStarVoxel->getRecastData()->getswidth()),Y_MAX,static_cast<float>(mAStarVoxel->getRecastData()->getslength())};

	for (astar_int index = 0 ; index < 5 ; ++index)
	{
		astar_int maxIndex = dtCheck(&toPos[0], &toPos[1], &toPos[2], checkMax);

		float move = 0;
		if (maxIndex == -1)
		{
			break;
		}
		else if (toPos[maxIndex] >= checkMax[maxIndex] )
		{
			move = ((checkMax[maxIndex] - fromPos[maxIndex] - 1) / dir[maxIndex]);
		}
		else if (toPos[maxIndex] < 0)
		{
			move = (0 - fromPos[maxIndex]) / dir[maxIndex];
		}
		toPos[0] = move * dir[0] + fromPos[0];
		toPos[1] = move * dir[1] + fromPos[1];
		toPos[2] = move * dir[2] + fromPos[2];
	}
	toPos[0] += org[0];
	toPos[1] += org[1];
	toPos[2] += org[2];

	return true;
}
bool DetourAstar::isPosBlock(float* pos) const
{
	const float* org = mAStarVoxel->getRecastData()->getOrg();

	pos[0] = (pos[0] - org[0]);
	pos[1] = (pos[1] - org[1]);
	pos[2] = (pos[2] - org[2]);

	astar_int x = static_cast<astar_int>(pos[0] / VOXEL_SIZE);
	astar_int y = static_cast<astar_int>(pos[1] / VOXEL_SIZE);
	astar_int z = static_cast<astar_int>(pos[2] / VOXEL_SIZE);

	const AstarCellDoing* beginDoing = getCellDoing(x, y, z);
	if (beginDoing == nullptr || !checkArea(beginDoing)) {
		return true;
	}
	return false;
}
void DetourAstar::findPath(float* begin, float* end, float* result, astar_int& resultLen)
{
	CHECK_VOXEL_VOID

	const float* org = mAStarVoxel->getRecastData()->getOrg();
	float beginCopy[3] = { (begin[0] - org[0]) ,(begin[1] - org[1]) ,(begin[2] - org[2]) };
	float endCopy[3]   = { (end[0] - org[0]) ,(end[1] - org[1]) ,(end[2] - org[2]) };

	astar_int xBegin = static_cast<astar_int>(beginCopy[0] / VOXEL_SIZE);
	astar_int yBegin = static_cast<astar_int>(beginCopy[1] / VOXEL_SIZE);
	astar_int zBegin = static_cast<astar_int>(beginCopy[2] / VOXEL_SIZE);

	astar_int xEnd = static_cast<astar_int>(endCopy[0] / VOXEL_SIZE);
	astar_int yEnd = static_cast<astar_int>(endCopy[1] / VOXEL_SIZE);
	astar_int zEnd = static_cast<astar_int>(endCopy[2] / VOXEL_SIZE);

	mAStarDoingMap.clear();
	mPoolIndex = 0;
	const AstarCellDoing* beginDoing = getCellDoing(xBegin, yBegin, zBegin);
	const AstarCellDoing* endDoing   = getCellDoing(xEnd, yEnd, zEnd);
	if (beginDoing == nullptr || endDoing == nullptr)
	{
		return;
	}
	if (!checkArea(beginDoing)|| !checkArea(endDoing))
	{
		return;
	}
	mOpenList.clear();
	mCloseSet.clear();
	addOpenList(beginDoing);

	AstarCellDir curDir;
	std::vector<const AstarCellDoing*> neighborhoodList;
	const AstarCellDoing* curDoing = nullptr;

	int checkTimes = 0;
	while (!mOpenList.empty())
	{
		if (checkTimes > 1000) 
		{
			return;
		}
		++checkTimes;
		AstarCellVec::iterator It = findMin(endDoing);//mOpenList[0];
		if (It == mOpenList.end())
		{
			continue;
		}
		curDoing = *It;
		removeOpenList(It);
		if (curDoing == NULL )
		{
			continue;
		}
		
		if (curDoing == endDoing)
		{
			resonstructPath(endDoing, result, resultLen);
			return;
		}
		mCloseSet.insert(curDoing->getIndex());
		getNeighborhood(curDoing, neighborhoodList);
		if (!neighborhoodList.empty())
		{
			std::vector<const AstarCellDoing*>::iterator it = neighborhoodList.begin();
			for (; it != neighborhoodList.end(); ++it)
			{
				if (addOpenList(*it))
				{
					(*it)->setParentDoingIndex(curDoing->getIndex());
					(*it)->setF(getDistance(&((*it)->getDir()),&endDoing->getDir()));
				}
			}
		}
	}
	return;
}
const AstarCellVec::iterator DetourAstar::findMin(const AstarCellDoing* endDoing)
{
	if (!mOpenList.empty())
	{
		return (std::min_element(mOpenList.begin(), mOpenList.end(), sortFun));
	}
	return mOpenList.end();
}
void DetourAstar::resonstructPath(const AstarCellDoing* curDoing, float* result, astar_int& resultLen)
{
	CHECK_VOXEL_VOID
	if (curDoing == nullptr)
	{
		return;
	}
	const AstarCellDoing* pathDoing = curDoing;
	astar_int index = 0;
	const float* org = mAStarVoxel->getRecastData()->getOrg();
	for( ; pathDoing; ++index)
	{
		float x = pathDoing->getDir().mX * VOXEL_SIZE + (float)VOXEL_SIZE / 2;
		float y = pathDoing->getDir().mY * VOXEL_SIZE + (float)VOXEL_SIZE / 2;
		float z = pathDoing->getDir().mZ * VOXEL_SIZE + (float)VOXEL_SIZE / 2;

		result[index * 3] = x + org[0];
		result[index * 3 + 1] = y + org[1];
		result[index * 3 + 2] = z + org[2];

		astar_long  parentIndex = pathDoing->getParentDoingIndex();
		if (parentIndex == 0)
		{
			resultLen = index + 1;
			return;
		}
		pathDoing = getCellDoing(parentIndex -1);
		if (index >= 1000)
		{
			assert(!"too many point");
			resultLen = index + 1;
			return;
		}
	}
	resultLen = index + 1;
}
void DetourAstar::removeOpenList(AstarCellVec::iterator& It)
{
	if (It == mOpenList.end())
	{
		return;
	}
	mOpenList.erase(It);
}
bool DetourAstar::addOpenList( const AstarCellDoing* neighborCell)
{
	if (neighborCell == NULL)
	{
		return false;
	}
	if (std::find(mOpenList.begin(), mOpenList.end(), neighborCell) != mOpenList.end())
	{
		return false;
	}
	mOpenList.push_back(neighborCell);
	return true;
}
bool DetourAstar::addCloseList(const AstarCellDoing* neighborCell)
{
	mCloseSet.insert(neighborCell->getIndex());
	return true;
}
astar_int DetourAstar::getDistance(const AstarCellDir* begin, const AstarCellDir* end)
{
	if (begin == NULL || end == NULL)
	{
		return -1;
	}
	return abs(begin->mX - end->mX) + abs(begin->mY - end->mY) + abs(begin->mZ - end->mZ) ;
}
void DetourAstar::getNeighborhood(const AstarCellDoing* curDoing, std::vector<const AstarCellDoing*>& neighborList) 
{
	CHECK_VOXEL_VOID
	neighborList.clear();
	if (curDoing == NULL)
	{
		return ;
	}

	//获得包含自身的27个节点
	astar_int swidth  = static_cast<astar_int>(mAStarVoxel->getRecastData()->getswidth());
	astar_int slength = static_cast<astar_int>(mAStarVoxel->getRecastData()->getslength());

	astar_int startX = std::max(0, curDoing->getDir().mX - 1);
	astar_int endX   = std::min(swidth - 1 , curDoing->getDir().mX + 1);
	astar_int startY = std::max(0, curDoing->getDir().mY - 1);
	astar_int endY   = std::min(Y_MAX - 1, curDoing->getDir().mY + 1);
	astar_int startZ = std::max(0, curDoing->getDir().mZ - 1);
	astar_int endZ   = std::min(slength - 1, curDoing->getDir().mZ + 1);

	for (astar_int i = startX; i <= endX; ++i)
	{
		for (astar_int j = startY; j <= endY; ++j)
		{
			for (astar_int k = startZ; k <= endZ; ++k)
			{
				const AstarCellDoing* compareDoing = getCellDoing(i, j, k);
				if (compareDoing == nullptr)
				{
					continue;
				}
				if (!isCanWalk(curDoing, compareDoing))
				{
					continue;
				}
				neighborList.push_back(compareDoing);
			}
		}
	}
}
bool DetourAstar::isCanWalk(const AstarCellDoing* curDoing,const AstarCellDoing* compareDoing) const
{
	CHECK_VOXEL_FALSE
	if (curDoing == nullptr || compareDoing == nullptr)
	{
		return false;
	}
	if (!checkArea(curDoing))
	{
		return false;
	}
	if (!checkArea(compareDoing))
	{
		return false;
	}
	if (mCloseSet.find(compareDoing->getIndex()) != mCloseSet.end())
	{
		return false;
	}

	bool isCell1 = mAStarVoxel->checkArea(curDoing->getDir().mX, compareDoing->getDir().mY, compareDoing->getDir().mZ);
	bool isCell2 = mAStarVoxel->checkArea(compareDoing->getDir().mX, curDoing->getDir().mY, compareDoing->getDir().mZ);
	bool isCell3 = mAStarVoxel->checkArea(compareDoing->getDir().mX, compareDoing->getDir().mY, curDoing->getDir().mZ);

	if (!isCell1 || !isCell2 || !isCell3 )
	{
		return false;
	}
	return true;
}
