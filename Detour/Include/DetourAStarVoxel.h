#ifndef DETOURASTARDATA
#define DETOURASTARDATA
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "../../Recast/Include/RecastData.h"
//===============================================================
struct HeightSpan
{
public:
	HeightSpan() :id(0), x(0), y(0), z(0), area(0)
	{};
	unsigned int id;
	unsigned int x;
	unsigned int y;
	unsigned int z;
	int area : 6;

	bool operator == (const HeightSpan& span) const {

		if ((this->x == span.x) && (this->y == span.y) && (this->z == span.z))
		{
			return true;
		}
		return false;
	};
};
////===============================================================
struct AstarCellDir
{
	AstarCellDir() :mX(-1), mY(-1), mZ(-1)
	{}
	astar_short mX;
	astar_short mY;
	astar_short mZ;
};
class AstarCell
{
public:
	AstarCell() :mArea(0)
	{}
	~AstarCell()
	{
		mArea = 0;
	}
	astar_char  mArea;
};

class AStarVoxel
{
public:
	AStarVoxel();
	~AStarVoxel();
	bool loadAstarData(const unsigned char* pos);
	bool checkArea(astar_long x, astar_long y, astar_long z) const;
	bool checkArea(astar_long index) const;
	const RecastData* getRecastData() const { return mAStarVoxelData;} 
	astar_long getIndex(astar_long x, astar_long y, astar_long z) const;
private:
	RecastData* mAStarVoxelData;
};
//================================globe func=========================================
void loadAStarVoxel(std::string& name,const unsigned char* pos, bool isDeleteAll = false);
void destoryAllVoxel();
const AStarVoxel* getAStarVoxelByName(std::string& name);
//delete all voxel when java close ..
extern std::unordered_map<std::string, AStarVoxel*> allAStarVoxelMap;
#endif // DETOURASTARDATA
