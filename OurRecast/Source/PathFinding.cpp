#include <jni.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "NavMesh.h"

#include "DetourCommon.h"
#include "org_gof_server_support_PathFinding.h"
#include <string.h>
#include "DetourCrowd.h"
#include "OurSample_TileBin.h"
#include "../../Detour/Source/DetourNavMeshQuery.cpp"
#include <fstream>
#include <time.h>
#include <random>
#include <limits.h>
//#include "DetourInit.h"

//#include "DetourInit.h"

static float frandEx()
{
	// default_random_engine
	std::random_device   e;
	//std::uniform_real_distribution<float> u(min, max); //随机数分布对象 
	std::uniform_real_distribution<float> u(0.0f, 1.0f);
	return u(e);
	//return ((float)(rand() & 0xffff)/(float)0xffff);
	//return (float)rand() / (float)RAND_MAX;
}
static int irand()
{
	std::random_device   e;
	std::uniform_int_distribution<> u; //随机数分布对象 
	return u(e);
}

NavMeshsMap  navMeshs;
extern float m_polyPickExt[];
extern float m_polyPickExt1[];

float m_polyPickExt[] = { 2, 4, 2 };
//float m_polyPickExt[] = { 2, 500, 2 };
// 射线使用 为什么 高 要 4000 ？ 
extern float m_polyPickExt1[];
//float m_polyPickExt1[] = { 2, 4000, 2 };
float m_polyPickExt1[] = { 2, 4, 2 };
//OurBaseSampleMap mOurBaseSampleMap;
using namespace std;

extern bool mIsFromClient;
bool mIsFromClient = false;
/*
 * 类型转换：jstring -> char*
 */


//#define  GET_OUR_SAMPLE(jstage) OurBaseSampleMap::iterator baseSampleIt = mOurBaseSampleMap.find(jstage);  \
//	if (baseSampleIt == mOurBaseSampleMap.end()) \
//	{ \
//		std::cerr << "can not find stageInfo:" << jstage << std::endl; \
//		return NULL; \
//	}

#define GET_OUR_QUERY  if(ourSample == NULL) \
	{ \
		std::cerr << "ourSample is null" << std::endl; \
		return false ; \
	} \
	dtNavMeshQuery* query = ourSample->getNavMeshQuery();\
	if (query == NULL) \
	{  \
		std::cerr << "dtNavMeshQuery is null" << std::endl; \
		return false; \
	} 

#define JAVA_POINT_NAME ("ourRecastObj")

#define GET_JAVA_POINT() jclass clazz = env->GetObjectClass(obj); \
	jfieldID fid = env->GetFieldID(clazz, JAVA_POINT_NAME, "J"); \
	jlong javaPoint = env->GetLongField(obj, fid); 

#define  GET_REAL_POINT(javaPoint,error) if(javaPoint == 0) \
	{ \
		std::cerr << "can not find stageInfo:" << jstage << std::endl;  \
		return error; \
	} \
	OurBaseSample* ourSample = (OurBaseSample*)(javaPoint) ; 

#define SAVE_JAVA_POINT()  env->SetLongField(obj, fid, javaPoint); 

string jstring2String(JNIEnv *env, jstring jstringInfo)
{
	char* result;

	// 找到String的class
	jclass jcls = env->FindClass("java/lang/String");

	// 新建String对象
	jstring jstr = env->NewStringUTF("utf-8");

	// 反射找到getBytes("")方法
	jmethodID jmid = env->GetMethodID(jcls, "getBytes", "(Ljava/lang/String;)[B");

	// 调用getBytes("")方法 返回bytes
	jbyteArray jbytes = (jbyteArray)env->CallObjectMethod(jstringInfo, jmid, jstr);

	// 获取数组长度
	jsize jlen = env->GetArrayLength(jbytes);

	// 获取数组具体元素
	jbyte * jba = env->GetByteArrayElements(jbytes, JNI_FALSE);
	if (jlen > 0)
	{
		// 申请String对应的bytes数组长度的内存
		result = (char*)malloc(jlen + 1);

		// 复制数组到char中
		memcpy(result, jba, jlen);

		// 补充最后一个字节'\0'
		result[jlen] = 0;
	}

	// 释放内存  避免造成内存泄露   
	env->DeleteLocalRef(jcls);
	env->DeleteLocalRef(jstr);
	env->ReleaseByteArrayElements(jbytes, jba, 0);
	env->DeleteLocalRef(jbytes);

	string str = result;
	free(result);

	return str;
}

JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_loadVoxelData(JNIEnv * env, jclass, jint stageSn, jint stageHightMax , jstring jpath)
{
	try
	{
		// 数据转换
		string path = jstring2String(env, jpath);

		std::ifstream fileStream;
		fileStream.open(path, ios::ios_base::binary);
		fileStream.seekg(0, ios::ios_base::end);

		streamoff  nFileLen = fileStream.tellg();
		fileStream.clear();
		fileStream.seekg(0, std::ios_base::beg);
		unsigned char* cc = (unsigned char*)malloc(sizeof(unsigned char) * nFileLen);

		fileStream.read((char*)cc, nFileLen);

		const unsigned char* pos = cc;
		loadAStarVoxel(path,pos);

		fileStream.close();
		free(cc);

		return true;
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ loadVoxelData() method  error" << std::endl;
	}
	return false;
}
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_loadOurRecast(JNIEnv *env, jobject obj, jstring jnavpath, jstring jvoxelpath ,jlong jstage, jint stageHigh)
{
	try
	{
		GET_JAVA_POINT();

		if (javaPoint != 0)
		{
			return false;
		}

		string navPath	 = jstring2String(env, jnavpath);
		string voxelPath = jstring2String(env, jvoxelpath);

		OurBaseSample* ourBaseSample = loadOurRecast(navPath,voxelPath, stageHigh, jstage);
		if (!ourBaseSample)
		{
			return false;
		}

		javaPoint = (jlong)ourBaseSample;

		SAVE_JAVA_POINT();

		return true;
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ loadOurRecast() method  error" << std::endl;
	}
	return false;
}
/**
 * 加载nav数据并初始化
 */
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_loadNavData(JNIEnv *env, jclass ,jint stageSn, jint stageHightMax ,jstring jpath)
{
	try
	{
		// 数据转换
		string path = jstring2String(env, jpath);

		std::ifstream fileStream;
		fileStream.open(path, ios::ios_base::binary);
		fileStream.seekg(0, ios::ios_base::end);

		streamoff  nFileLen = fileStream.tellg();
		fileStream.clear();
		fileStream.seekg(0, std::ios_base::beg);
		unsigned char* cc = (unsigned char*)malloc(sizeof(unsigned char) * nFileLen);

		fileStream.read((char*)cc, nFileLen);

		if (!cc)
		{
			std::cerr << "===================================Java loadNavMesh Null============================== " << std::endl;
			std::cerr << stageSn << std::endl;
			return false;
		}
		bool isSuccess = loadNavMesh(path, stageHightMax, cc);

		fileStream.close();
		free(cc);

		return isSuccess;
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ loadNavData() method  error" << std::endl;
	}
	return false;
}

JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_findPath(JNIEnv * env, jobject obj, jlong jstage, jfloatArray jstart, jfloatArray jend, jint jflag)
{
	_jfloatArray* resultArr = NULL;
	try {

		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint, NULL);
		
		float path[MAX_SMOOTH * 3] = { 0 };
		int pathLen = 0;

		// 起始位置,终点位置
		float spos[3], epos[3];

		// 格式转换，从jfloatArray转为jfloat
		jfloat* start = env->GetFloatArrayElements(jstart, JNI_FALSE);
		jfloat* end = env->GetFloatArrayElements(jend, JNI_FALSE);

		// 拷贝数据，转为float数组
		dtVcopy(spos, start);
		dtVcopy(epos, end);

		// 释放资源
		env->ReleaseFloatArrayElements(jstart, start, 0);
		env->ReleaseFloatArrayElements(jend, end, 0);

		findPath(ourSample, spos, epos, jflag /*SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED*/, path, &pathLen);

		resultArr  = env->NewFloatArray(pathLen);
		if (resultArr == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}
		env->SetFloatArrayRegion(resultArr, 0, pathLen, path);
	}

	catch (...)
	{
		std::cerr << "Java native interface call c++ findPath() method  error" << std::endl;
	}
	
	return resultArr;
}
/**
 * 判断某坐标是否在阻挡区域内
 */
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_isPosInBlock(JNIEnv * env, jobject obj, jlong jstage, jfloatArray jpos )
{
	try
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint,false);

		// 格式转换
		float pos[3];
		jfloat* tmp = env->GetFloatArrayElements(jpos, JNI_FALSE);
		dtVcopy(pos, tmp);

		// 释放资源
		env->ReleaseFloatArrayElements(jpos, tmp, 0);

	
		bool isBlock = IsPosInBlock(ourSample, pos);
		return isBlock;
	}
	catch (...)
	{
		std::cerr << "stageInfo" << jstage << ",find poly exception " << std::endl;
	}
	
	return false;
}

JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_raycast(JNIEnv * env, jobject obj, jlong jstage, jfloatArray beginPos, jfloatArray endPos, jint jflag)
{
	_jfloatArray* resultArr = NULL;
	try
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint, NULL);

		// 起始位置,终点位置
		float spos[3], epos[3];

		// 格式转换，从jfloatArray转为jfloat
		jfloat* start = env->GetFloatArrayElements(beginPos, JNI_FALSE);
		jfloat* end = env->GetFloatArrayElements(endPos, JNI_FALSE);

		// 拷贝数据，转为float数组
		dtVcopy(spos, start);
		dtVcopy(epos, end);

		// 释放资源
		env->ReleaseFloatArrayElements(beginPos, start, 0);
		env->ReleaseFloatArrayElements(endPos, end, 0);

		int const arrayLen = 3;		// 数组长度
		float result[arrayLen];
		float hitNormal[arrayLen];

		memset(result, 0, sizeof(float)* arrayLen);
		memset(hitNormal, 0, sizeof(float) * arrayLen);

		raycast(ourSample, spos, epos, jflag, result, hitNormal);

		/* 将float数组转为jfloatArray作为结果返回 */
		resultArr = env->NewFloatArray(3);
		if (resultArr == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}

		env->SetFloatArrayRegion(resultArr, 0, 3, result);
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ rayCast() method error" << std::endl;
	}
	return resultArr;
}
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_randomPosition(JNIEnv * env, jobject obj, jlong jstage,jfloatArray jpos , jfloat maxRadio, jint flag)
{
	_jfloatArray* resultArr = NULL;
	try
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint, NULL);

		// 格式转换
		float pos[3];
		jfloat* tmp = env->GetFloatArrayElements(jpos, JNI_FALSE);
		if (tmp == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}

		dtVcopy(pos, tmp);

		// 释放资源
		env->ReleaseFloatArrayElements(jpos, tmp, 0);

		float result[3] = {0};
		rondomPoint(ourSample, pos,maxRadio, result,flag);

		/* 将float数组转为jfloatArray作为结果返回 */
		resultArr = env->NewFloatArray(3);
		if (resultArr == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}

		env->SetFloatArrayRegion(resultArr, 0, 3, result);

	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ posHeight() method error" << std::endl;
	}
	return resultArr;



}
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_posHeight(JNIEnv * env, jobject obj, jlong jstage, jfloatArray jpos)
{
	_jfloatArray* resultArr = NULL;
	try
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint,NULL);

		// 格式转换
		float pos[3];
		jfloat* tmp = env->GetFloatArrayElements(jpos, JNI_FALSE);
		if (tmp == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}

		dtVcopy(pos, tmp);

		// 释放资源
		env->ReleaseFloatArrayElements(jpos, tmp, 0);


		float result[3];
		posHeight(ourSample,pos, result);

		/* 将float数组转为jfloatArray作为结果返回 */
		resultArr = env->NewFloatArray(3);
		if (resultArr == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}

		env->SetFloatArrayRegion(resultArr, 0, 3, result);
		
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ posHeight() method error" << std::endl;
	}
	return resultArr;
}

JNIEXPORT void JNICALL Java_org_gof_server_support_PathFinding_test(JNIEnv *, jobject, jlong test1 , jint test2 )
{
	cerr << "test " << test1 << test2  << endl;

	//测试下其他 库是否可用
	dtCrowdNeighbour a;
	a.dist = 5.0f;
	a.idx = 10;

	cerr << "dtCrowdNeighbour " << a.dist << a.idx << endl;
	dtNavMeshQuery* navMeshQuery = dtAllocNavMeshQuery();
	navMeshQuery->getNodePool();

	cerr << "dtNavMeshQuery  getNodePool " <<  endl;
}
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_destory(JNIEnv *env, jobject obj, jlong jstage)
{
	GET_JAVA_POINT();
	GET_REAL_POINT(javaPoint,false);


	bool isDestory = destory(ourSample);

	//避免执行多次，重复删除内存
	javaPoint = 0;
	SAVE_JAVA_POINT();
	return isDestory;
}
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_findPath3D(JNIEnv *env , jobject obj , jlong jstage, jfloatArray beginPos, jfloatArray endPos)
{
	_jfloatArray* resultArr = NULL;
	try 
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint, NULL);

		jfloat* begin = env->GetFloatArrayElements(beginPos, JNI_FALSE);
		jfloat* end = env->GetFloatArrayElements(endPos, JNI_FALSE);

		float path[1000 * 3] = { 0 };
		int pathLen = 0;

		ourSample->getDetourAstar()->findPath(begin, end, path, pathLen);

		env->ReleaseFloatArrayElements(beginPos, begin, 0);
		env->ReleaseFloatArrayElements(endPos, end, 0);

		resultArr = env->NewFloatArray(pathLen * 3);
		if (resultArr == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}
		env->SetFloatArrayRegion(resultArr, 0, pathLen * 3, path);
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ findPath3D() method error" << std::endl;
	}
	return resultArr;
}
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_isPosInBlock3D(JNIEnv *env, jobject obj, jlong jstage, jfloatArray jpos)
{
	try
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint, false);

		jfloat* pos = env->GetFloatArrayElements(jpos, JNI_FALSE);

		bool isBlock = ourSample->getDetourAstar()->isPosBlock(pos);

		env->ReleaseFloatArrayElements(jpos, pos, 0);
		return isBlock;
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ isPosInBlock3D() method error" << std::endl;
	}
	return false;
}
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_correctClientPos(JNIEnv *env, jobject obj, jlong jstage, jfloatArray jfromPos, jfloatArray jtoPos )
{
	_jfloatArray* resultPos = NULL;
	try
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint, NULL);

		jfloat* fromPos = env->GetFloatArrayElements(jfromPos, JNI_FALSE);
		jfloat* toPos   = env->GetFloatArrayElements(jtoPos, JNI_FALSE);

		ourSample->getDetourAstar()->correctPos(fromPos,toPos);

		resultPos = env->NewFloatArray(3);
		env->SetFloatArrayRegion(resultPos, 0, 3, toPos);

		env->ReleaseFloatArrayElements(jfromPos, fromPos, 0);
		env->ReleaseFloatArrayElements(jtoPos, toPos, 0);

		if (resultPos == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ correctClientPos() method error" << std::endl;
	}
	return resultPos;
}
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_findNearestByHightPoly(JNIEnv *env, jobject obj , jlong jstage , jfloatArray jpos)
{
	_jfloatArray* resultPos = NULL;
	try
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint, NULL);

		jfloat* pos = env->GetFloatArrayElements(jpos, JNI_FALSE);

		float nearPos[3] = { 0 };
		bool isFind = false;
		findNearestByHightPoly(ourSample, pos, nearPos, isFind);

		env->ReleaseFloatArrayElements(jpos, pos, 0);
		
		if (isFind)
		{
			resultPos = env->NewFloatArray(3);
			if (resultPos == NULL)
			{
				std::cerr << "out of memory!" << std::endl;
				return NULL;
			}
			env->SetFloatArrayRegion(resultPos, 0, 3, nearPos);
		}
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ findNearestByHightPoly() method error" << std::endl;
	}
	return resultPos;
}
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_rayfly(JNIEnv *env, jobject obj, jlong jstage, jfloatArray jfromPos, jfloatArray jtoPos)
{
	_jfloatArray* resultArr = NULL;
	try
	{
		GET_JAVA_POINT();
		GET_REAL_POINT(javaPoint, NULL);

		jfloat* fromPos = env->GetFloatArrayElements(jfromPos, JNI_FALSE);
		jfloat* toPos = env->GetFloatArrayElements(jtoPos, JNI_FALSE);

		float result[3] = { 0 };
		ourSample->getDetourAstar()->rayFly(fromPos, toPos, result);

		env->ReleaseFloatArrayElements(jfromPos, fromPos, 0);
		env->ReleaseFloatArrayElements(jtoPos, toPos, 0);

		/* 将float数组转为jfloatArray作为结果返回 */
		resultArr = env->NewFloatArray(3);
		if (resultArr == NULL)
		{
			std::cerr << "out of memory!" << std::endl;
			return NULL;
		}
		env->SetFloatArrayRegion(resultArr, 0, 3, result);
	}
	catch (...)
	{
		std::cerr << "Java native interface call c++ rayfly() method error" << std::endl;
	}
	return resultArr;
}
//========================================================================================================================================================================================================
bool loadNavMesh(std::string name, int sceneHighMax ,unsigned char* data, bool isAstar)
{
	if (navMeshs.find(name) != navMeshs.end())
	{
		return true;
	}

	dtNavMesh* mesh = dtLoadNavMeshBundle(name, sceneHighMax,data);
	if (!mesh)
	{
		return false;
	}


	navMeshs.insert(std::make_pair(name, mesh));
	return true;
}

OurBaseSample* loadOurRecast(std::string& navName, std::string& voxelName, int sceneHigh, long long stageInfo)
{
	//此处直接New 了， 场景种类应该不会对性能产生影响
	OurBaseSample* ourMesh = new OurSample_TileBin(stageInfo);
	ourMesh->createDetourAstar();
	if (!ourMesh->load(navName,ourMesh->getRcContext()))
	{
		std::cerr << "Java native interface call c++ loadNavData() method faild " << std::endl;
		return NULL;
	}
	if (!ourMesh->handleBuild())
	{
		std::cerr << "Java native interface call c++ loadNavData() handleBuild faild " << std::endl;
		return NULL;
	}
	
	if (!ourMesh->getDetourAstar()->initDetourAstar(voxelName))
	{
		std::cerr << "Java native interface call c++ loadNavData() A* faild " << std::endl;
		return NULL;
	}
	return ourMesh;
}
bool findPath(OurBaseSample* ourSample, float* beginPos, float* endPos, long flag , float* pathArray ,int* pathArrayLen)
{
	GET_OUR_QUERY

	// 距离起始坐标和终点坐标最近的polygon多边形的id
	dtPolyRef startRef;
	dtPolyRef endRef;
	
	// 过滤条件
	dtQueryFilter filter;
	filter.setIncludeFlags((unsigned short)flag);
	filter.setExcludeFlags(0);


	if (dtStatusFailed(query->findNearestPoly(beginPos, m_polyPickExt, &filter, &startRef, 0)))
	{
		std::cerr << "stageInfo:"  << " find start poly error, startPos(" << beginPos[0] << "," << beginPos[1] << "," <<  beginPos[2] << ")" << std::endl;
		return false;
	}
	if (dtStatusFailed(query->findNearestPoly(endPos, m_polyPickExt, &filter, &endRef, 0)))
	{
		std::cerr << "stageInfo:" << " find start poly error, startPos(" << endPos[0] << "," << endPos[1] << "," << endPos[2] << ")" << std::endl;
		return false;
	}

	// 如果没找到起点
	if (!startRef) {
		std::cerr << "stageInfo:"  << " Could not find startPoly: " << startRef << ", from startPos(" << beginPos[0] << "," << beginPos[2] << ") to endPos(" << endPos[0] << "," << endPos[2] << ")" << std::endl;
		return false;
	}

	float realEndPos_[3];
	// 如果没找到终点，逻辑层可以通过raycast寻路到射线点
	if (!endRef) 
	{
		//std::cerr << "stageInfo:"  << " Could not find endPoly: " << endRef << ", from startPos(" << beginPos[0] << "," << beginPos[2] << ") to endPos(" << endPos[0] << "," << endPos[2] << ")" << std::endl;
		//return false;


		// 如果没找到终点, 说明终点在阻挡区域, raycast找直线到终点被阻挡的点, 重新找多边形
		// raycast找直线到终点被阻挡的点
	
		int const arrayLen = 3;		// 数组长度
		float result[arrayLen];
		float hitNormal[arrayLen];

		raycast(ourSample,beginPos, endPos, flag, result, hitNormal);
		
		query->findNearestPoly(result, m_polyPickExt, &filter, &endRef, 0);
		
		// 找起点随机能到的附近的点
		//navMeshQuery->findRandomPointAroundCircle(m_startRef, m_spos, 2, &m_filter, frand, &m_endRef, m_epos);
		if (!endRef) {
			std::cerr << "stageInfo:" << ", Could not find endPoly: " << endRef << ", from startPos(" << beginPos[0] << "," << beginPos[2] << ") to endPos(" << endPos[0] << "," << endPos[2] << ")" << std::endl;
			return false;
		
		}
		dtVcopy(realEndPos_, result);
	}
	else
	{
		dtVcopy(realEndPos_, endPos);
	}

	// 记录找到的路径上的多边形id的数组，起点->终点
	dtPolyRef polys[MAX_POLYS] = { 0 };
	// 多边形个数
	int npolys = 0;

	// 查询起点多边形->终点多边形的路径
	if (dtStatusFailed(query->findPath(startRef, endRef, beginPos, realEndPos_, &filter, polys, &npolys, MAX_POLYS)))
	{
		std::cerr << "stageInfo:"  << ", findPath error, from startPos(" << beginPos[0] << "," << beginPos[2] << ") to endPos(" << realEndPos_[0] << "," << realEndPos_[2] << ")" << std::endl;
		return false;
	}

	// 直线路径上坐标数组
	float straightPath[MAX_POLYS * 3] = { 0 };
	unsigned char straightPathFlags[MAX_POLYS] = { 0 };
	// 找到的直线路径上多边形id的数组 
	dtPolyRef straightPathPolys[MAX_POLYS] = { 0 };
	// 找到的直线距离坐标个数
	int nstraightPath = 0;

	float endPos_[3];
	dtVcopy(endPos_, realEndPos_);

	// 如果大于0找到了 则找直线路径
	if (npolys > 0) {
		if (polys[npolys - 1] != endRef)
		{
			//只更新最后一个点
			query->closestPointOnPoly(polys[npolys - 1], realEndPos_, endPos_, NULL);
		}

		// 找直线路径
		if (dtStatusFailed(query->findStraightPath(beginPos, endPos_, polys, npolys, straightPath, straightPathFlags, straightPathPolys, &nstraightPath, MAX_POLYS)))
		{
			std::cerr << "stageInfo:" << ", findStraightPath error, from startPos(" << beginPos[0] << "," << beginPos[2] << ") to endPos(" << endPos[0] << "," << endPos[2] << ")" << std::endl;
		}
	}

	if (nstraightPath > MAX_SMOOTH)
	{
		std::cerr << "findPath too many:" <<  std::endl;
		return false;
	}
	//float path[MAX_SMOOTH * 3] = { 0 };

	int pos = 0;
	for (int i = 0; i < nstraightPath * 3;) {
		pathArray[pos++] = straightPath[i++];
		pathArray[pos++] = straightPath[i++];
		pathArray[pos++] = straightPath[i++];
	}

	// 如果最后一个路径点不是终点，把终点加入
	if (pathArray[nstraightPath * 3 - 3]    != endPos_[0]
		|| pathArray[nstraightPath * 3 - 2] != endPos_[1]
		|| pathArray[nstraightPath * 3 - 1] != endPos_[2])
	{
		pathArray[pos++] = endPos_[0];
		pathArray[pos++] = endPos_[1];
		pathArray[pos++] = endPos_[2];
	}

	*pathArrayLen = pos;

	return true;
}
bool IsPosInBlock(OurBaseSample* ourSample, float* pos)
{
	GET_OUR_QUERY

	// 过滤条件
	dtQueryFilter filter;
	filter.setIncludeFlags(0xffff);
	filter.setExcludeFlags(0);

	// 距离该坐标最近的polygon多边形的id
	dtPolyRef posRef = 0;

	// 查找距离最近的多边形
	if (dtStatusFailed(query->findNearestPoly(pos, m_polyPickExt1, &filter, &posRef, 0)))
	{
		std::cerr << "stageInfo:" << ourSample->getStageInfo() << ", find poly error, pos(" << pos[0] << "," << pos[2] << ")" << std::endl;
		return true;
	}

	if (!posRef)
	{
		return true;
	}

	const dtMeshTile* tile = NULL;
	const dtPoly* poly = NULL;

	query->getAttachedNavMesh()->getTileAndPolyByRefUnsafe(posRef, &tile, &poly);

	if (tile == NULL)
	{
		return true;
	}
	//return true;
	return !filter.passFilter(posRef, tile, poly);

}
bool raycast(OurBaseSample* ourSample, float* beginPos, float* endPos, long flag, float* result, float* hitNormal)
{
	GET_OUR_QUERY

	// 过滤条件
	dtQueryFilter filter;
	filter.setIncludeFlags((unsigned short)flag);
	filter.setExcludeFlags(0);

	// 距离起始坐标和终点坐标最近的polygon多边形的id
	dtPolyRef startRef;

	//ourSample->dtLog(ourSample->getNavMesh()->)
	ourSample->dtLog("raycast1");
	// 查找距离最近的多边形
	if (dtStatusFailed(query->findNearestPoly(beginPos, m_polyPickExt, &filter, &startRef, 0)))
	{
		std::cerr << "findNearestPoly error stageInfo:" << ourSample->getStageInfo()
			<< ", startPos(" << beginPos[0] << "," << beginPos[1] << "," << beginPos[2] << ")"
			<< ", endPos(" << endPos[0] << "," << endPos[1] << "," << endPos[2] << ")"
			<< std::endl;
		return false;
	}
	ourSample->dtLog("raycast2");
	// 如果没找到
	if (!startRef) {
		std::cerr << "stageInfo:" << ourSample->getStageInfo() << ", Could not find startPos:(" << beginPos[0] << "," << beginPos[2] << "), startPoly:" << startRef << std::endl;
		return false;
	}

	ourSample->dtLog("raycast3");
	//float hitNormal[3] = { 0 };9
	float t = 0.0f;
	dtPolyRef path[MAX_POLYS] = { 0 };
	int pathCount = 0;
	// raycast判断是否阻挡
	if (dtStatusFailed(query->raycast(startRef, beginPos, endPos, &filter, &t, hitNormal, path, &pathCount, MAX_POLYS)))
	{
		std::cerr << "raycast error stageInfo:" << ourSample->getStageInfo()
			<< ", startPos(" << beginPos[0] << "," << beginPos[1] << "," << beginPos[2] << ")"
			<< ", endPos(" << endPos[0] << "," << endPos[1] << "," << endPos[2] << ")"
			<< std::endl;
		return false;
	}
	ourSample->dtLog("raycast4");
	// 如果t=FLT_MAX则起点到终点无阻挡，能通过, 如果t=0则起点在阻挡区域, 返回终点坐标
	// 否则0<t<1.0则有阻挡，hitPoint = startPos + (endPos - startPos) * t
	// 以上是官网API附加说明，这里根据raycast的方法注释The hit parameter. (FLT_MAX if no wall hit.)简化处理
	// 此处 t > 1 即为 t=FLT_MAX 情况 ，判定 if (t == FLT_MAX) 会产生float 失真，永远不成立

	if (t > 1)
	{
		dtVcopy(result, endPos);
	}

	// t == 0 的情况  起点在阻挡区域, 返回终点坐标
	else if (t < 0.000001)
	{
		//直接返回起点坐标
		dtVcopy(result, beginPos);

		// 找终点所在多边形
		//dtPolyRef endRef = 0;
		//query->findNearestPoly(epos, m_polyPickExt, &filter, &endRef, 0);

		// 如果终点在阻挡，返回起点，否则返回终点
		//if (!endRef) {
		//	dtVcopy(result, spos);
		//}
		//else {
		//	dtVcopy(result, epos);
		//}
	}

	else if (t > 0 && t < 1)
	{
		result[0] = beginPos[0] + (endPos[0] - beginPos[0]) * t;
		result[1] = beginPos[1] + (endPos[1] - beginPos[1]) * t;
		result[2] = beginPos[2] + (endPos[2] - beginPos[2]) * t;

		if (pathCount > 0)
		{
			float h = 0.0f;
			//query->getPolyHeight(path[pathCount - 1], result, &h);
			//result[1] = h;
			//上面函数 会 由于 小数点 失真 判断 点 不在 Poly 中 ，下面重新计算下新Poly ，所在高度 
			float temPos[3] = { 0 };
			dtVcopy(temPos, result);
			posHeight(ourSample, temPos, result);
		}
	}
	
	return true;
}
bool posHeight(OurBaseSample* ourSample, float* pos, float* result)
{
	GET_OUR_QUERY
	// 距离该坐标最近的polygon多边形的id
	dtPolyRef posRef = 0;

	// 过滤条件
	dtQueryFilter filter;
	filter.setIncludeFlags(0xffff);
	filter.setExcludeFlags(0);

	//float result[3];
	// 查找距离最近的多边形
	if (dtStatusFailed(query->findNearestPoly(pos, m_polyPickExt1, &filter, &posRef, result)))
	{
		std::cerr << "stageInfo:" << ourSample->getStageInfo() << ", find poly error, pos(" << pos[0] << "," << pos[2] << ")" << std::endl;
		return false;
	}

	if (!posRef)
	{
		std::cerr << "stageSn:" << ourSample->getStageInfo() << ", Could not find pos:(" << pos[0] << "," << pos[1] << " , " << pos[2] << "), posRef:" << posRef << std::endl;
		return false;
	}

	query->getPolyHeight(posRef, pos, &result[1]);
	return true;
}
bool rayFly(OurBaseSample* ourSample, float* beginPos,float* dir ,float* hitPos)
{
	//endPos - beginPos 
	GET_OUR_QUERY
	memset(hitPos, 0, sizeof(float) * 3);
	
	ourSample->getDetourAstar()->rayFly(beginPos, dir, hitPos);

	return true;
}
bool findNearestByHightPoly(OurBaseSample* ourSample, float* pos, float* nearPos,bool& isFind)
{
	GET_OUR_QUERY

	// 过滤条件
	dtQueryFilter filter;
	filter.setIncludeFlags(0xffff);
	filter.setExcludeFlags(0);

	dtPolyRef nearRef = 0;
	memset(nearPos, 0, sizeof(float) * 3);
	float halfExtents[] = { 100,100,100 };
	//服務器的話 支持到 1024
	if (!mIsFromClient) {
		halfExtents[0] = 1024;
		halfExtents[1] = 1024;
		halfExtents[2] = 1024;
	}
	if (dtStatusFailed(query->findNearestByHightPoly(pos, halfExtents, &filter, &nearRef, nearPos, isFind)))
	{
		return false;
	}

	
	return true;
}
bool rondomPoint(OurBaseSample* ourSample,float* beginPos, float maxRedius,float* result,long flag)
{
	GET_OUR_QUERY
	// 距离该坐标最近的polygon多边形的id
	dtPolyRef startRef = 0;

	// 过滤条件
	dtQueryFilter filter;
	filter.setIncludeFlags(static_cast<unsigned short>(flag));
	filter.setExcludeFlags(0);

	if (dtStatusFailed(query->findNearestPoly(beginPos, m_polyPickExt, &filter, &startRef, 0)))
	{
		std::cerr << "stageInfo:" << ourSample->getStageInfo() << ", find poly error, pos(" << beginPos[0] << "," << beginPos[2] << ")" << std::endl;
		return false;
	}
	if (!startRef)
	{
		std::cerr << " rondomPoint stageSn:" << ourSample->getStageInfo() << ", Could not find pos:(" << beginPos[0] << "," << beginPos[1] << " , " << beginPos[2] << "), posRef:" << startRef << std::endl;
		return false;
	}
	static const int MAX_RAND_POINTS = 64;
	dtPolyRef randomRef = 0;
	if (dtStatusFailed(query->findRandomPointAroundCircle(startRef, beginPos,maxRedius, &filter, frandEx, &randomRef, result)))
	{
		return false;
	} 
	return true;
}
bool destory(OurBaseSample* ourSample)
{
	try
	{
		if (ourSample)
		{
			delete ourSample;
			ourSample = NULL;
		}
	}
	catch (...)
	{
		std::cerr << "destory fail" << std::endl;
		return false;
	}
	return true; 
}
//#include "DetourAstar.h"

//#include <windows.h>
//#include <winnt.h>
//int WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
//int WinMain(int /*argc*/, char** /*argv*/)
 /*{
	std::string filepath = "D:/data/Project/eos/trunk/code/tool/recastnavigation/RecastDemo/Bin/solo_navmesh.bin";

	// 数据转换
	std::ifstream fileStream;
	fileStream.open(filepath, ios::ios_base::binary);
	fileStream.seekg(0, ios::ios_base::end);

	streamoff  nFileLen = fileStream.tellg();
	fileStream.clear();
	fileStream.seekg(0, std::ios_base::beg);
	unsigned char* cc = (unsigned char*)malloc(sizeof(unsigned char) * nFileLen);

	fileStream.read((char*)cc, nFileLen);

	bool isSuccess = loadNavMesh(1, cc);

	fileStream.close();
	free(cc);

	OurBaseSample* ourMesh = new OurSample_TileBin(1);
	//ourMesh->createDetourAstar();
	ourMesh->load(1, ourMesh->getRcContext());
}  */


	//std::string filepath = "D:/data/Project/eos/trunk/code/tool/recastnavigation/RecastDemo/Bin/solo_navmesh.bin";

	// 数据转换
	//std::ifstream fileStream;
	//fileStream.open(filepath, ios::ios_base::binary);
	//fileStream.seekg(0, ios::ios_base::end);

	//streamoff  nFileLen = fileStream.tellg();
	//fileStream.clear();
	//fileStream.seekg(0, std::ios_base::beg);
	//unsigned char* cc = (unsigned char*)malloc(sizeof(unsigned char) * nFileLen);

	//fileStream.read((char*)cc, nFileLen);

	//bool isSuccess = loadNavMesh(1, cc);

	//fileStream.close();
	//free(cc);

	//OurBaseSample* ourMesh = new OurSample_TileBin(1);
	//ourMesh->load(1, ourMesh->getRcContext());

	
	//destoryAllVoxel();

	//delete ourMesh;
	//ourMesh = NULL;
	//_CrtDumpMemoryLeaks();
	//return 0;
//}
	//ourMesh->load(ourMesh->getRcContext(), "D:/data/Project/eos/trunk/code/tool/recastnavigation/RecastDemo/Bin/all_tiles_navmesh.bin", NULL);
//}
	//OurBaseSample* ourMesh = new OurSample_TileBin(1);
	// D:\\data\\Project\\eos\\trunk\\code\\tool\\recastnavigation\\RecastDemo\\Bin\\Meshes\\dungeon.gset
	//ourMesh->load(ourMesh->getRcContext(), "D:/data/Project/eos/trunk/code/tool/recastnavigation/RecastDemo/Bin/Meshes/nav_test",NULL);
	
	//ourMesh->load(ourMesh->getRcContext(), "D:/data/Project/eos/trunk/code/tool/recastnavigation/RecastDemo/Bin/all_tiles_navmesh.bin",NULL);

	//__int64 beginTime;
	//QueryPerformanceCounter((LARGE_INTEGER*)&beginTime);

	//ourMesh->handleBuild();

	//__int64 endTime;
	//QueryPerformanceCounter((LARGE_INTEGER*)&endTime);


	//static __int64 freq = 0;
	//if (freq == 0)
	//	QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
	//int cost =  (int)((endTime - beginTime) * 1000000 / freq) / 1000;


	//ourMesh->getRcContext()->stopTimer(RC_TIMER_TEMP);

	
	//float beginPos[3] = { 19.3154640, 14.5082016, -63.5296059 };
	//float endPos[3] = { 41.5098114, 9.99818420, 0.673915863 };

	//float path[MAX_SMOOTH * 3] = { 0 };
	//int pathLen = 0;

	//findPath(ourMesh, beginPos, endPos, SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED, path, &pathLen);
	//return 0;

//}