#include "Sample_TileMesh.h"
#include "InputGeom.h"
#include "ChosePositionTool.h"
#include "Sample.h"

void autoBuild(const char* file_name, float pos_x, float pos_y, float pos_z)
{
	gBuildbeginPos[0] = -pos_x;
	gBuildbeginPos[1] = pos_y;
	gBuildbeginPos[2] = pos_z;
	gChoosePos = true;
	static const std::string meshesFolder = "Meshes";
	BuildContext ctx;
	InputGeom* geom = new InputGeom();
	geom->load(&ctx, meshesFolder + "/" + file_name);
	Sample_TileMesh* sample = new Sample_TileMesh();
	sample->setContext(&ctx);
	sample->setSourceMeshName(file_name);
	sample->handleMeshChanged(geom);
	sample->handleSettings();
	sample->handleBuild();

	std::string realName = sample->getSaveName(0);
	sample->saveAll(realName.c_str(), sample->getNavMesh());
	sample->saveData3D();
}
