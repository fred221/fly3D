/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class org_gof_server_support_PathFinding */

#ifndef _Included_org_gof_server_support_PathFinding
#define _Included_org_gof_server_support_PathFinding
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    loadNavData
 * Signature: (IILjava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_loadNavData
  (JNIEnv *, jclass, jint, jint, jstring);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    loadVoxelData
 * Signature: (IILjava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_loadVoxelData
  (JNIEnv *, jclass, jint, jint, jstring);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    loadOurRecast
 * Signature: (Ljava/lang/String;Ljava/lang/String;JI)Z
 */
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_loadOurRecast
  (JNIEnv *, jobject, jstring, jstring, jlong, jint);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    findPath
 * Signature: (J[F[FI)[F
 */
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_findPath
  (JNIEnv *, jobject, jlong, jfloatArray, jfloatArray, jint);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    isPosInBlock
 * Signature: (J[F)Z
 */
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_isPosInBlock
  (JNIEnv *, jobject, jlong, jfloatArray);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    raycast
 * Signature: (J[F[FI)[F
 */
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_raycast
  (JNIEnv *, jobject, jlong, jfloatArray, jfloatArray, jint);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    randomPosition
 * Signature: (J[FFFI)[F
 */
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_randomPosition
  (JNIEnv *, jobject, jlong, jfloatArray, jfloat, jint);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    posHeight
 * Signature: (J[F)[F
 */
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_posHeight
  (JNIEnv *, jobject, jlong, jfloatArray);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    destory
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_destory
  (JNIEnv *, jobject, jlong);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    findPath3D
 * Signature: (J[F[F)[F
 */
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_findPath3D
  (JNIEnv *, jobject, jlong, jfloatArray, jfloatArray);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    isPosInBlock3D
 * Signature: (J[F)Z
 */
JNIEXPORT jboolean JNICALL Java_org_gof_server_support_PathFinding_isPosInBlock3D
  (JNIEnv *, jobject, jlong, jfloatArray);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    findNearestByHightPoly
 * Signature: (J[F)[F
 */
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_findNearestByHightPoly
  (JNIEnv *, jobject, jlong, jfloatArray);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    correctClientPos
 * Signature: (J[F[F)[F
 */
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_correctClientPos
  (JNIEnv *, jobject, jlong, jfloatArray, jfloatArray);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    rayfly
 * Signature: (J[F[F)[F
 */
JNIEXPORT jfloatArray JNICALL Java_org_gof_server_support_PathFinding_rayfly
  (JNIEnv *, jobject, jlong, jfloatArray, jfloatArray);

/*
 * Class:     org_gof_server_support_PathFinding
 * Method:    test
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_org_gof_server_support_PathFinding_test
  (JNIEnv *, jobject, jint, jint);

#ifdef __cplusplus
}
#endif
#endif