
#ifndef DETOUR_ASTAR_H
#define DETOUR_ASTAR_H
#include "DetourAStarVoxel.h"
#include <map>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#define DOING_ARRAY_MAX (9999)
class DetourAstar;
class AstarCellDoing
{
public:
	AstarCellDoing();
	~AstarCellDoing();
	void setParentDoingIndex(astar_long index) const;
	astar_long getParentDoingIndex() const;
	//f
	inline void setF(astar_int f) const { mF = f; }
	inline astar_int getF() const { return mF;}
	//index
	inline astar_long getIndex() const { return mIndex; }
	inline void setIndex(astar_long index) const { mIndex = index;}

	inline const void rest() const;
	inline void resetDir(const AStarVoxel * astarVoxel);
	inline const AstarCellDir& getDir() const { return mDir; }
private:
	mutable astar_long  mParentIndex;
	mutable astar_int   mF;
	mutable astar_long  mIndex;
	mutable AstarCellDir mDir;
};

typedef std::vector<const AstarCellDoing* >    AstarCellVec;
typedef std::unordered_set<astar_long>	 AstarCellSet;
typedef std::unordered_map<astar_long,AstarCellDoing* >  AStarDoingMap;
class  DetourAstar
{	
public:

	friend bool sortFun(const AstarCellDoing* curCell, const AstarCellDoing* otherCell) ;
	
	DetourAstar();
	virtual ~DetourAstar();

	virtual void findPath(float* begin, float* end, float* result, astar_int& resultLen);
	virtual bool initDetourAstar(std::string& name);

	const AStarVoxel* getAStarVoxel() const { return mAStarVoxel; }
	bool checkArea(const AstarCellDoing* doing) const;
	bool checkArea(astar_int x, astar_int y, astar_int z) const;
	bool rayFly(float* start, float* dir, float* hit) const;
	bool isPosBlock(float* pos) const;
	bool correctPos(float* fromPos,float* toPos);
public:
	const AstarCellDoing* getCellDoing(astar_int x, astar_int y, astar_int z,bool isBuild = false /*是否考虑场景体素高*/) const;
	inline const AstarCellDoing* getCellDoing(float* beginPos) const;
	const AstarCellDoing* getCellDoing(astar_long index) const;

	const AstarCellVec::iterator findMin(const AstarCellDoing* endDoing);
	
	bool isCanWalk(const AstarCellDoing* curCell, const AstarCellDoing* compareCell) const;
	bool addOpenList(const AstarCellDoing* neighborCell);
	bool addCloseList(const AstarCellDoing* neighborCell);
	void getNeighborhood(const AstarCellDoing* curCell, std::vector<const AstarCellDoing*>& neighBorHood) ;
	void resonstructPath(const AstarCellDoing* curCell, float* result, astar_int& resultLen);
	void removeOpenList(AstarCellVec::iterator& constIt);
	void clear();
	astar_int getDistance(const AstarCellDir* begin, const AstarCellDir* end);

protected:
	AstarCellVec			mOpenList;
	AstarCellSet			mCloseSet;
	AstarCellDoing*			mAStarDoingPool;
	mutable astar_int		mPoolIndex;
	mutable AStarDoingMap	mAStarDoingMap;
	const AStarVoxel*		mAStarVoxel;
};

#endif // DETOUR_ASTAR_H
