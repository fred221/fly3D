#ifndef PATHFINDING_H_
#define PATHFINDING_H_
#if  defined(WIN32) || defined(__WIN32__) || defined(_WIN32) || defined(_MSC_VER)
# define _DLLExport __declspec (dllexport)  
# else  
# define _DLLExport __attribute__ ((visibility("default")))
#endif  

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


#include "DetourCommon.h"
#include <iosfwd>
#include "OurSample_TileBin.h"
#include "OurBaseSample.h"
#include "NavMesh.h"
//#include "DetourInit.h"


/*
typedef void(* DebugCallback) (const char *str);

extern "C" void _DLLExport RegisterDebugCallback(DebugCallback callback);
*/
extern "C" bool _DLLExport LoadNavData(unsigned char* data);

extern "C" void _DLLExport UnLoadNavData();

extern "C" bool _DLLExport FindNavPath(float* start, float* end, int jflag, float*  &outarr, int &len);

extern "C" bool _DLLExport Raycast(float* start, float* end, int jflag, float* &outarr, float* &normalDir);

extern "C" bool _DLLExport IsPosInBlock(float* jpos);

extern "C" void _DLLExport ClearIntPtr(void* pBuffer);

extern "C" void _DLLExport ClearArrayIntPtr(void* pBuffer);

extern "C" void _DLLExport FindNearestPoly(float* start, float* end, int flag, int &nearRef, float* &pt);

extern "C" bool _DLLExport IsWalkable(float* start, int flag);

extern "C" float _DLLExport GetPolyHeight(float* point, int flag);

extern "C" void _DLLExport SetPolyPickExtern(float x, float y, float z);

extern "C" bool _DLLExport NearestByHightPoly(float* pos, float*  &nearPos);

extern "C" void _DLLExport GetNavMeshVertices(float*& outArray, int& len);
OurBaseSample* mOurBaseSampleBin = NULL;




#endif