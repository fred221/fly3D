/*
 * NavMesh.h
 *
 */

#ifndef NAVMESH_H_
#define NAVMESH_H_

#include <iterator>
#include <utility>
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
//#include <unordered_map>
#include "OurBaseSample.h"
using namespace std;

typedef unordered_map<std::string, dtNavMesh*>  NavMeshsMap;
//typedef unordered_map<int, dtNavMeshQuery*>  NavMeshsQueryMap;

//typedef unordered_map<int, OurBaseSample*>  OurBaseSampleMap;
	// 寻路用到的最大值数据
	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;

	// 导航网格数据，从.nav文件中读取到的数据存在这里。
	extern NavMeshsMap  navMeshs;
	// 用于寻路的对象, 提供寻路算法和Dijkstra寻路算法。 portId_sn -> navMeshQuery
	//NavMeshsQueryMap navMeshQuerys;

	extern bool loadNavMesh(std::string name , int sceneHighMax,unsigned char* data,bool isAstar = true);
	extern bool findPath(OurBaseSample* ourSample, float* beginPos, float* endPos, long flag, float* pathArray, int* pathArrayLen);
	extern bool raycast(OurBaseSample* ourSample, float* beginPos, float* endPos, long flag,float* result,float* hitNormal);
	extern bool IsPosInBlock(OurBaseSample* ourSample, float* pos);
	extern bool posHeight(OurBaseSample* ourSample, float* pos,float* result);
	extern bool rondomPoint(OurBaseSample* ourSample, float* beginPos,float maxRedius, float* result,long flag);
	extern bool findNearestByHightPoly(OurBaseSample* ourSample, float* pos, float* nearPos,bool& isFind);
	
	extern OurBaseSample* loadOurRecast(std::string& navName, std::string& voxelName,int sceneHigh , long long stageInfo);
	bool destory(OurBaseSample* ourSample);
	
#endif /* NAVMESH_H_ */
