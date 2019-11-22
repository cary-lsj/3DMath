#include "AABB3.h"
#include <assert.h>
#include <stdlib.h>
#include "AABB3.h"
#include "Matrix4x3.h"
#include <Windows.h>
#include <iostream>

using namespace std;

// 名称：AABB3D
// 创建者：cary
// 描述：3D中的轴对齐矩形边界框（AABB）

//提取八个定点中的一个
// 限定在 [000,111]
// 第0位选择 min.x vs. max.x
// 第1位选择 min.y vs. max.y
// 第2位选择 min.z vs. max.z
Vector3 AABB3::corner(int i)const {
	assert(i >= 0);
	assert(i <= 7);
	return Vector3(
		(i & 1) ? max.x : min.x,
		(i & 2) ? max.y : min.y,
		(i & 4) ? max.z : min.z
	);
}

//“清空”矩形边界框
//将值赋为极大值/极小值已清空矩形边界框
void AABB3::empty() {
	const float kBigNumber = 1e37f;
	min.x = min.y = min.z = +kBigNumber;
	max.x = max.y = max.z = -kBigNumber;
}

//向矩形边界框中添加点
void AABB3::add(const Vector3& p) {
	if (p.x < min.x)min.x = p.x;
	if (p.y < min.y)min.y = p.y;
	if (p.z < min.z)min.z = p.y;
	if (p.x > max.x)max.x = p.x;
	if (p.y > max.y)max.y = p.y;
	if (p.z > max.z)max.z = p.y;
}

//向矩形边界框中添加AABB
void AABB3::add(const AABB3& box) {

	if (box.min.x < min.x)min.x = box.min.x;
	if (box.min.y < min.y)min.y = box.min.y;
	if (box.min.z < min.z)min.z = box.min.y;
	if (box.max.x > max.x)max.x = box.max.x;
	if (box.max.y > max.y)max.y = box.max.y;
	if (box.max.z > max.z)max.z = box.max.y;
}

//变换矩形边界框,计算新的AABB
//会得到一个一样或者大得多的AABB
void AABB3::setToTransFormedBox(const AABB3& box, const Matrix4x3& m)
{
	if (box.isEmpty()) {
		empty();
		return;
	}
	//平移
	min = max = getTranslation(m);
	//x
	if (m.m11 > 0.0f) {
		min.x += m.m11 * box.min.x;
		max.x += m.m11 * box.max.x;
	}
	else {
		min.x += m.m11 * box.max.x;
		max.x += m.m11 * box.min.x;
	}
	if (m.m21 > 0.0f) {
		min.x += m.m21 * box.min.y;
		max.x += m.m21 * box.max.y;
	}
	else {
		min.x += m.m21 * box.max.y;
		max.x += m.m21 * box.min.y;
	}
	if (m.m31 > 0.0f) {
		min.x += m.m31 * box.min.z;
		max.x += m.m31 * box.max.z;
	}
	else {
		min.x += m.m31 * box.max.z;
		max.x += m.m31 * box.min.z;
	}

	//y
	if (m.m12 > 0.0f) {
		min.y += m.m12 * box.min.x;
		max.y += m.m12 * box.max.x;
	}
	else {
		min.y += m.m12 * box.max.x;
		max.y += m.m12 * box.min.x;
	}
	if (m.m22 > 0.0f) {
		min.y += m.m22 * box.min.y;
		max.y += m.m22 * box.max.y;
	}
	else {
		min.y += m.m22 * box.max.y;
		max.y += m.m22 * box.min.y;
	}
	if (m.m32 > 0.0f) {
		min.y += m.m32 * box.min.z;
		max.y += m.m32 * box.max.z;
	}
	else {
		min.y += m.m32 * box.max.z;
		max.y += m.m32 * box.min.z;
	}

	//z
	if (m.m13 > 0.0f) {
		min.z += m.m13 * box.min.x;
		max.z += m.m13 * box.max.x;
	}
	else {
		min.z += m.m13 * box.max.x;
		max.z += m.m13 * box.min.x;
	}
	if (m.m23 > 0.0f) {
		min.z += m.m23 * box.min.y;
		max.z += m.m23 * box.max.y;
	}
	else {
		min.z += m.m23 * box.max.y;
		max.z += m.m23 * box.min.y;
	}
	if (m.m33 > 0.0f) {
		min.z += m.m33 * box.min.z;
		max.z += m.m33 * box.max.z;
	}
	else {
		min.z += m.m33 * box.max.z;
		max.z += m.m33 * box.min.z;
	}
}

//返回 true，如果矩形边界为空
bool AABB3::isEmpty() const
{
	return (min.x > max.x) || (min.y > max.y) || (min.z > max.z);
}

//返回true，如果矩形包含该点
bool AABB3::contains(const Vector3& p) const
{
	return (p.x >= min.x) && (p.x <= max.x)
		&& (p.y >= min.y) && (p.y <= max.y)
		&& (p.z >= min.z) && (p.z <= max.z);
}

//返回矩阵边界框上的最近点
Vector3 AABB3::closestPointTo(const Vector3& p) const
{
	//在每一维上将p“推向”矩形边界框
	Vector3 r;
	//x
	if (p.x < min.x) {
		r.x = min.x;
	}
	else if (p.x > max.x) {
		r.x = max.x;
	}
	else {
		r.x = p.x;
	}

	//y
	if (p.y < min.y) {
		r.y = min.y;
	}
	else if (p.y > max.y) {
		r.y = max.y;
	}
	else {
		r.y = p.y;
	}

	//z
	if (p.z < min.z) {
		r.z = min.z;
	}
	else if (p.z > max.z) {
		r.z = max.z;
	}
	else {
		r.z = p.z;
	}
	return r;
}

//返回true，如果和球相交
bool AABB3::intersectsSphere(const Vector3& center, float radius) const
{
	//找到矩形边界框上离球心最近的点
	Vector3 closestPoint = closestPointTo(center);
	//检查最近点和球心的距离是否小于半径
	return distanceSquared(center, closestPoint) < radius * radius;
}

//和参数射线的相交性测试，如果不相则返回值大于1
	//rayOrg 射线起点 
	//rayDelta 射线长度和方向
	//returnNoamal可选的相交点
float AABB3::rayIntersect(const Vector3& rayOrg, const Vector3 rayDelta, Vector3* returnNoamal) const
{
	//如果未相交则返回这个大数
	const float kNoIntersection = 1e30f;
	//检查点在矩形边界框内的情况，并计算到每个面的距离
	bool inside = true;
	float xt, xn;
	if (rayOrg.x < min.x) {
		xt = min.x - rayOrg.x;
		if (xt > rayDelta.x) {
			return kNoIntersection;
		}
		xt /= rayDelta.x;
		inside = false;
		xn = -1.0f;
	}
	else if (rayDelta.x > max.x) {
		xt = max.x - rayDelta.x;
		if (xt < rayDelta.x) {
			return kNoIntersection;
		}
		xt /= rayDelta.x;
		inside = false;
		xn = -1.0f;
	}
	else {
		xt = -1.0f;
	}

	float yt, yn;
	if (rayOrg.y < min.y) {
		yt = min.y - rayOrg.y;
		if (yt > rayDelta.y) {
			return kNoIntersection;
		}
		yt /= rayDelta.y;
		inside = false;
		yn = -1.0f;
	}
	else if (rayDelta.y > max.y) {
		yt = max.y - rayDelta.y;
		if (yt < rayDelta.y) {
			return kNoIntersection;
		}
		yt /= rayDelta.y;
		inside = false;
		yn = -1.0f;
	}
	else {
		yt = -1.0f;
	}

	float zt, zn;
	if (rayOrg.z < min.z) {
		zt = min.z - rayOrg.z;
		if (zt > rayDelta.z) {
			return kNoIntersection;
		}
		zt /= rayDelta.z;
		inside = false;
		zn = -1.0f;
	}
	else if (rayDelta.z > max.z) {
		zt = max.z - rayDelta.z;
		if (zt < rayDelta.z) {
			return kNoIntersection;
		}
		zt /= rayDelta.z;
		inside = false;
		zn = -1.0f;
	}
	else {
		zt = -1.0f;
	}

	if (inside) {
		if (returnNoamal != NULL) {
			*returnNoamal = -rayDelta;
			returnNoamal->normalize();
		}
		return 0.0f;
	}

	//选择最远的平面——发生相交的地方
	int which = 0;
	float t = xt;
	if (yt > t) {
		which = 1;
		t = yt;
	}
	if (zt > t) {
		which = 2;
		t = zt;
	}

	switch (which)
	{
	case 0://和yz平面相交
	{
		float y = rayOrg.y + rayDelta.y * t;
		if (y<min.y || y>max.y) {
			return kNoIntersection;
		}

		float z = rayOrg.z + rayDelta.z * t;
		if (z<min.z || z>max.z) {
			return kNoIntersection;
		}
		if (returnNoamal != NULL) {
			returnNoamal->x = xn;
			returnNoamal->y = 0.0f;
			returnNoamal->z = 0.0f;
		}
	}
	break;
	case 1://和xz平面相交
	{
		float x = rayOrg.x + rayDelta.x * t;
		if (x<min.x || x>max.x) {
			return kNoIntersection;
		}

		float z = rayOrg.z + rayDelta.z * t;
		if (z<min.z || z>max.z) {
			return kNoIntersection;
		}
		if (returnNoamal != NULL) {
			returnNoamal->x = 0.0f;
			returnNoamal->y = yn;
			returnNoamal->z = 0.0f;
		}
	}
	break;
	case 2://和xy平面相交
	{
		float x = rayOrg.x + rayDelta.x * t;
		if (x<min.x || x>max.x) {
			return kNoIntersection;
		}

		float y = rayOrg.y + rayDelta.y * t;
		if (y<min.y || y>max.y) {
			return kNoIntersection;
		}

		if (returnNoamal != NULL) {
			returnNoamal->x = 0.0f;
			returnNoamal->y = 0.0f;
			returnNoamal->z = zn;
		}
	}
	break;
	default:
		break;
	}

	return t;
}

//判断矩形边界框在矩形的那一面
//静止AABB与平面的相交性检测
//返回值：
// 小于0  矩形边界框完全在平面的背面
// 大于0  矩形边界框完全在平面的正面
// 0  矩形边界框和平面相交
int AABB3::classifyPlane(const Vector3& n, float d) const
{
	//计算最小和最大距离
	float minD, maxD;
	if (n.x > 0.0f) {
		minD = n.x * min.x;
		maxD = n.x * max.x;
	}
	else {
		minD = n.x * max.x;
		maxD = n.x * min.x;
	}

	if (n.y > 0.0f) {
		minD += n.y * min.y;
		maxD += n.y * max.y;
	}
	else {
		minD += n.y * max.y;
		maxD += n.y * min.y;
	}

	if (n.z > 0.0f) {
		minD += n.z * min.z;
		maxD += n.z * max.z;
	}
	else {
		minD += n.z * max.z;
		maxD += n.z * min.z;
	}

	if (minD >= d) {
		//前面
		return +1;
	}

	if (maxD <= d) {
		//背面
		return -1;
	}

	//横跨平面
	return 0;
}
//和平面的动态相交性测试
//n为平面的法向量（标准化向量）
//planeD 为平面方程 p*n=d 中的D值
//dir AABB移动的方向
//
//假设平面是静止的
//返回交点的参数值——相交时AABB移动的距离，如果未相交则返回一个大数
//
//只探测和平面正面的相交
float AABB3::intersectPlane(const Vector3& n, float planeD, const Vector3& dir) const
{
	//检测向量是否正则化
	assert(fabs(n * n - 1.0f) < .01);
	assert(fabs(dir * dir - 1.0f) < .01);
	//如果未相交则返回这个大数
	const float kNoIntersection = 1e30f;
	//计算夹角，确保是在平面的正面移动
	float dot = n * dir;
	if (dot >= 0.0f) {
		return kNoIntersection;
	}
	//检查法向量，计算最小和最大值，minD是“跑在最前面的”顶点的D值
	float minD, maxD;
	if (n.x > 0.0f) {
		minD = n.x * min.x;
		maxD = n.x * max.x;
	}
	else {
		minD = n.x * max.x;
		maxD = n.x * min.x;
	}
	
	if (n.y > 0.0f) {
		minD += n.y * min.y;
		maxD += n.y * max.y;
	}
	else {
		minD += n.y * max.y;
		maxD += n.y * min.y;
	}

	if (n.z > 0.0f) {
		minD += n.z * min.z;
		maxD += n.z * max.z;
	}
	else {
		minD += n.z * max.z;
		maxD += n.z * min.z;
	}

	//检测是否已经全部在平面的一面
	if (maxD <= planeD) {
		return kNoIntersection;
	}
	//将最前面定点带入标准射线方程
	float t = (planeD - minD) / dot;
	if (t < 0.0f) {
		//穿过他了
		return 0.0f;
	}
	//如果 大于1，则未能及时到达平面，这时需要调用者进行检查
	return t;
}

//检测AABB的相交性，如果返回true，还可以返回相交部分的AABB
bool intersectAABBs(const AABB3& box1, const AABB3& box2, AABB3* boxIntersect)
{
	//判断是否有重叠
	if (box1.min.x > box2.max.x)return false;
	if (box1.min.y > box2.max.y)return false;
	if (box1.min.z > box2.max.z)return false;
	if (box1.max.x < box2.min.x)return false;
	if (box1.max.y < box2.min.y)return false;
	if (box1.max.z < box2.min.z)return false;
	//有重叠，计算重叠部分的AABB，
	if (boxIntersect != NULL) {
		boxIntersect->min.x = max(box1.min.x, box2.min.x);
		boxIntersect->min.y = max(box1.min.y, box2.min.y);
		boxIntersect->min.z = max(box1.min.z, box2.min.z);
		boxIntersect->max.x = min(box1.max.x, box2.max.x);
		boxIntersect->max.y = min(box1.max.y, box2.max.y);
		boxIntersect->max.z = min(box1.max.z, box2.max.z);
	}

	return true;
}

//返回运动AABB和静止AABB相交时的参数点，如果不相交则返回值大于1
float intersectMovingAABB(const AABB3& stationaryBox, const AABB3& movingBox, const Vector3& d)
{
	//如果未相交则返回这个大数
	const float kNoIntersection = 1e30f;
	//初始化时间区间，以包含需要考虑的全部时间段
	float tEnter = 0.0f;
	float tLeave = 1.0f;
	//计算每一维的重叠部分，再将这个重叠部分和前面的重叠部分相交
	//如果有一维上重叠部分为零则返回(不会相交)
	//每一维上都必须当心零重叠
	//
	//
	//x
	if (d.x == 0.0f) {
		//X轴上重叠部分为空
		if ((stationaryBox.min.x > movingBox.max.x) || (stationaryBox.max.x < movingBox.min.x)) {
			return kNoIntersection;
		}
	}
	else
	{
		float oneOverD = 1.0f / d.x;
		//计算开始接触和分离的时间
		float xEnter = (stationaryBox.min.x - movingBox.max.x) * oneOverD;
		float xLeave = (stationaryBox.max.x - movingBox.min.x) * oneOverD;
		//检查排序
		if (xEnter > xLeave) {
			swap(xEnter, xLeave);
		}
		//更新区间
		if (xEnter > tEnter) {
			tEnter = xEnter;
		}
		if (xLeave < tLeave) {
			tLeave = xLeave;
		}
		//是否导致空间重叠
		if (tEnter > tLeave) {
			return kNoIntersection;
		}
	}

	//y
	if (d.y == 0.0f) {
		//y轴上重叠部分为空
		if ((stationaryBox.min.y > movingBox.max.y) || (stationaryBox.max.y < movingBox.min.y)) {
			return kNoIntersection;
		}
	}
	else
	{
		float oneOverD = 1.0f / d.y;
		//计算开始接触和分离的时间
		float yEnter = (stationaryBox.min.y - movingBox.max.y) * oneOverD;
		float yLeave = (stationaryBox.max.y - movingBox.min.y) * oneOverD;
		//检查排序
		if (yEnter > yLeave) {
			swap(yEnter, yLeave);
		}
		//更新区间
		if (yEnter > tEnter) {
			tEnter = yEnter;
		}
		if (yLeave < tLeave) {
			tLeave = yLeave;
		}
		//是否导致空间重叠
		if (tEnter > tLeave) {
			return kNoIntersection;
		}
	}

	//z
	if (d.z == 0.0f) {
		//z轴上重叠部分为空
		if ((stationaryBox.min.z > movingBox.max.z) || (stationaryBox.max.z < movingBox.min.z)) {
			return kNoIntersection;
		}
	}
	else
	{
		float oneOverD = 1.0f / d.z;
		//计算开始接触和分离的时间
		float zEnter = (stationaryBox.min.z - movingBox.max.z) * oneOverD;
		float zLeave = (stationaryBox.max.z - movingBox.min.z) * oneOverD;
		//检查排序
		if (zEnter > zLeave) {
			swap(zEnter, zLeave);
		}
		//更新区间
		if (zEnter > tEnter) {
			tEnter = zEnter;
		}
		if (zLeave < tLeave) {
			tLeave = zLeave;
		}
		//是否导致空间重叠
		if (tEnter > tLeave) {
			return kNoIntersection;
		}
	}
	return tEnter;
}
