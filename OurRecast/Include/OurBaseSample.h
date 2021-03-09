//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef OUR_RECASTSAMPLE_H
#define OUR_RECASTSAMPLE_H

#include "Recast.h"
#include "OurGeom.h"
#include "DetourAStar.h"
//#include "DetourAStarJump.h"
//#include "SampleInterfaces.h"

/// Tool types.
enum SampleToolType
{
	TOOL_NONE = 0,
	TOOL_TILE_EDIT,
	TOOL_TILE_HIGHLIGHT,
	TOOL_TEMP_OBSTACLE,
	TOOL_NAVMESH_TESTER,
	TOOL_NAVMESH_PRUNE,
	TOOL_OFFMESH_CONNECTION,
	TOOL_CONVEX_VOLUME,
	TOOL_CROWD,
	TOOL_SET_FLAGS,
	MAX_TOOLS
};

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,

	SAMPLE_PARTITION_DISABLE,  //这个在这个版本中 应该没用 ， 但此处还是 加上吧 。

	SAMPLE_PARTITION_BLOCK1 = 10,
	SAMPLE_PARTITION_BLOCK2,
	SAMPLE_PARTITION_BLOCK3,
	SAMPLE_PARTITION_BLOCK4,
	SAMPLE_PARTITION_BLOCK5,
	SAMPLE_PARTITION_BLOCK6,
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP = 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_BUILD = 0x10,
	SAMPLE_POLYFLAGS_DISABLED = 0x20,		// Disabled polygon

	SAMPLE_POLYFLAGS_PWNGS_BLOCK1 = 0x40,
	SAMPLE_POLYFLAGS_PWNGS_BLOCK2 = 0x80,
	SAMPLE_POLYFLAGS_PWNGS_BLOCK3 = 0x100,
	SAMPLE_POLYFLAGS_PWNGS_BLOCK4 = 0x200,
	SAMPLE_POLYFLAGS_PWNGS_BLOCK5 = 0x400,
	SAMPLE_POLYFLAGS_PWNGS_BLOCK6 = 0x800,
	SAMPLE_POLYFLAGS_ALL = 0xffff	// All abilities.
};

/*class SampleDebugDraw : public DebugDrawGL
{
public:
	virtual unsigned int areaToCol(unsigned int area);
}; */

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS,
};

struct SampleTool
{
	virtual ~SampleTool() {}
	virtual int type() = 0;
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleMenu() = 0;
	virtual void handleClick(const float* s, const float* p, bool shift) = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleToggle() = 0;
	virtual void handleStep() = 0;
	virtual void handleUpdate(const float dt) = 0;
};

struct SampleToolState {
	virtual ~SampleToolState() {}
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleUpdate(const float dt) = 0;
};

class OurBaseSample
{
protected:
	class OurGeom* m_Ourgeom;
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;
	class dtCrowd* m_crowd;

	unsigned char m_navMeshDrawFlags;

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	int m_partitionType;

	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;
	
	SampleToolState* m_toolStates[MAX_TOOLS];
	
	rcContext m_ctx;
	//BuildContext* m_ctx;

	//SampleDebugDraw m_dd;
	
	//dtNavMesh* loadAll(const char* path);
	//void saveAll(const char* path, const dtNavMesh* mesh);

public:
	OurBaseSample(long long stageInfo);
	virtual ~OurBaseSample();
	
	//void setTool(SampleTool* tool);
	SampleToolState* getToolState(int type) { return m_toolStates[type]; }
	void setToolState(int type, SampleToolState* s) { m_toolStates[type] = s; }
	//获得场景信息
	long long  getStageInfo(){ return mStageInfo; }
	rcContext* getRcContext() { return &m_ctx; } 
	//SampleDebugDraw& getDebugDraw() { return m_dd; }

	DetourAstar* getDetourAstar() { return mDetourAstar; } 
	virtual bool load(std::string name, rcContext* ctx) = 0;
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual bool handleBuild();
	virtual void handleMeshChanged();

	virtual class OurGeom* getOurGeom() { return m_Ourgeom; }
	virtual class dtNavMesh* getNavMesh() { return m_navMesh; }
	virtual class dtNavMeshQuery* getNavMeshQuery() { return m_navQuery; }
	virtual class dtCrowd* getCrowd() { return m_crowd; }
	virtual float getAgentRadius() { return m_agentRadius; }
	virtual float getAgentHeight() { return m_agentHeight; }
	virtual float getAgentClimb() { return m_agentMaxClimb; }
	
	unsigned char getNavMeshDrawFlags() const { return m_navMeshDrawFlags; }
	void setNavMeshDrawFlags(unsigned char flags) { m_navMeshDrawFlags = flags; }

	void updateToolStates(const float dt);
	void resetToolStates();
	void renderToolStates();
	void renderOverlayToolStates(double* proj, double* model, int* view);

	void resetCommonSettings();
	void collectSettings(BuildSettings& settings);
	unsigned int duRGBA(int r, int g, int b, int a);
	
	void dtLog(const char* log);

	void createDetourAstar();
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	OurBaseSample(const OurBaseSample&);
	OurBaseSample& operator=(const OurBaseSample&);

	//场景sn 或者 线程标识符
	long long mStageInfo;
	//三维寻路使用
	DetourAstar* mDetourAstar;
};


#endif // OUR_RECASTSAMPLE_H
