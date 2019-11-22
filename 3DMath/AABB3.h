#pragma once

#ifndef __AABB3_H_INCLUDED__
#define __AABB3_H_INCLUDED__

#ifndef __VECTOR3_H_INCLUDED
#include "Vector3.h"
#endif // #ifndef __VECTOR3_H_INCLUDED

class Matrix4x3;

// 名称：AABB3D
// 创建者：cary
// 描述：3D中的轴对齐矩形边界框（AABB）
class AABB3
{
public:
	Vector3 min;
	Vector3 max;
	Vector3 size() const { return max - min; }
	float xSize() { return max.x - min.x; }
	float ySize() { return max.y - min.y; }
	float zSize() { return max.z - min.z; }
	Vector3 center() const { return (min + max) * .5f; }
	//提取八个定点中的一个
	Vector3 corner(int i)const;
	//矩形边界框操作
	//“清空”矩形边界框
	void empty();
	//向矩形边界框中添加点
	void add(const Vector3& p);
	//向矩形边界框中添加AABB
	void add(const AABB3 &box);
	//变换矩形边界框,计算新的AABB
	void setToTransFormedBox(const AABB3& box, const Matrix4x3& m);
	//包含/相交性测试
	//返回 true，如果矩形边界为空
	bool isEmpty() const;
	//返回true，如果矩形包含该点
	bool contains(const Vector3& p) const;
	//返回矩阵边界框上的最近点
	Vector3 closestPointTo(const Vector3& p)const;
	//返回true，如果和球相交
	bool intersectsSphere(const Vector3& center, float radius)const;
	//和参数射线的相交性测试，如果不相则返回值大于1
	//rayOrg 射线起点 
	//rayDelta 射线长度和方向
	//returnNoamal可选的相交点
	float rayIntersect(const Vector3& rayOrg, const Vector3 rayDelta, Vector3* returnNoamal = 0)const;
	//判断矩形边界框在矩形的那一面
	//静止AABB与平面的相交性检测
	//返回值：
	// 小于0  矩形边界框完全在平面的背面
	// 大于0  矩形边界框完全在平面的正面
	// 0  矩形边界框和平面相交
	int classifyPlane(const Vector3& n, float d)const;
	//和平面的动态相交性测试
	//n为平面的法向量（标准化向量）
	//planeD 为平面方程 p*n=d 中的D值
	//dir AABB移动的方向
	//
	//假设平面是静止的
	//返回交点的参数值——相交时AABB移动的距离，如果未相交则返回一个大数
	//
	//只探测和平面正面的相交
	// 返回值 如果 大于1，则未能及时到达平面，这时需要调用者进行检查
	float intersectPlane(const Vector3& n, float planeD, const Vector3& dir)const;

};

//检测AABB的相交性，如果返回true，还可以返回相交部分的AABB
bool intersectAABBs(const AABB3& box1, const AABB3& box2, AABB3* boxIntersect = 0);

//返回运动AABB和静止AABB相交时的参数点，如果不相交则返回值大于1
float intersectMovingAABB(const AABB3& stationaryBox, const AABB3& movingBox, const Vector3& d);

#endif // #ifndef __AABB3_H_INCLUDED__