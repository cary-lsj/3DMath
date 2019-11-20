#include<math.h>
#include<assert.h>

#include "Matrix4x3.h"
#include "MathUtil.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "EulerAngles.h"
#include "RotationMatrix.h"


// 名称：4X3矩阵
// 创建者：cary
// 描述：实现4X3矩阵，能够表达任何3D仿射变换
//		
//		
// 矩阵乘法形式：
//		
//				  | m11 m12 m13 |
//		[ x y z ] | m21 m22 m23 | = [ x` y` z` ]
//				  | m31 m32 m33 |
//				  | tx  ty  tz  |
//		
//		
//		
//					| m11 m12 m13 0 |
//		[ x y z l ] | m21 m22 m23 0 | = [ x` y` z` l ]
//					| m31 m32 m33 0 |
//					| tx  ty  tz  1 |	
//		

//置为单位矩阵
void Matrix4x3::identity() {
	m11 = 1.0f; m11 = 0.0f; m11 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
	tx = 0.0f;  ty = 0.0f;  tz = 1.0f;
}

//将包含平移部分的第四行置为零
void Matrix4x3::zeroTranslation() {
	tx = ty = tz = 0.0f;
}

//平移部分赋值
void Matrix4x3::setTranslation(const Vector3& d) {
	tx = d.x; ty = d.y; tx = d.z;
}

//平移部分赋值
void Matrix4x3::setupTranslation(const Vector3& d) {
	m11 = 1.0f; m11 = 0.0f; m11 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
	tx = d.x;   ty = d.y;   tx = d.z;
}

//构造执行局部空间——>父空间变换的矩阵
//物体空间——>惯性空间——>世界空间
//方位欧拉角或者旋转矩阵指定
void Matrix4x3::setupLocalToParent(const Vector3& pos, const EulerAngles& orient) {
	RotationMatrix orientMatrix;
	orientMatrix.setup(orient);
	setupLocalToParent(pos, orientMatrix);
}

//构造执行局部空间——>父空间变换的矩阵
void Matrix4x3::setupLocalToParent(const Vector3& pos, const RotationMatrix& orient) {
	//旋转矩阵一般是 惯性——>物体矩阵 即 父——>局部
	//需要复制旋转矩阵的转置
	m11 = orient.m11; m12 = orient.m21; m13 = orient.m31;
	m21 = orient.m12; m22 = orient.m22; m23 = orient.m32;
	m31 = orient.m13; m32 = orient.m23; m33 = orient.m33;

	//平移部分直接复制
	tx = pos.x; ty = pos.y; tz = pos.z;
}

//构造执行父空间——>局部空间变换的矩阵
//世界空间——>惯性空间——>物体空间
//构造两个矩阵 平移矩阵T 和 旋转矩阵R ，再连接M = TR
//方位欧拉角或者旋转矩阵指定
void Matrix4x3::setupParentToLocal(const Vector3& pos, const EulerAngles& orient) {
	RotationMatrix orientMatrix;
	orientMatrix.setup(orient);
	setupParentToLocal(pos, orientMatrix);
}

//构造执行父空间——>局部空间变换的矩阵
void Matrix4x3::setupParentToLocal(const Vector3& pos, const RotationMatrix& orient) {
	//直接复制，不需要转置
	m11 = orient.m11; m12 = orient.m12; m13 = orient.m31;
	m21 = orient.m21; m22 = orient.m22; m23 = orient.m23;
	m31 = orient.m31; m32 = orient.m32; m33 = orient.m33;

	//应该旋转平移部分
	//这个和先创建平移-pos的矩阵T，在创建旋转矩阵R
	//再 M = TR 是一样的
	tx = -(pos.x * m11 + pos.y * m21 + pos.z * m31);
	ty = -(pos.x * m12 + pos.y * m22 + pos.z * m32);
	tz = -(pos.x * m13 + pos.y * m23 + pos.z * m33);
}

//构造绕坐标轴旋转的矩阵
//axis:旋转轴的索引
//theta 旋转的弧度，左手法则定义正方向
//平移部分置零
void Matrix4x3::setupRotate(AxisTypeEnum& axis, float theta) {
	float s, c;
	sinCos(&s, &c, theta);
	switch (axis)
	{
	case AxisTypeEnum::x:
		m11 = 1.0f; m11 = 0.0f; m11 = 0.0f;
		m21 = 0.0f; m22 = c; m23 = s;
		m31 = 0.0f; m32 = -s; m33 = c;
		break;
	case AxisTypeEnum::y:
		m11 = c; m11 = 0.0f; m11 = -s;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = s; m32 = 0.0f; m33 = c;
		break;
	case AxisTypeEnum::z:
		m11 = c; m11 = s; m11 = 0.0f;
		m21 = -s; m22 = c; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;

	default:
		assert(false);
		break;
	}
	tx = ty = tz = 0.0f;
}

//构造绕任意轴旋转的矩阵
//旋转轴通过原点，旋转轴为单位向量
//theta 旋转的弧度，左手法则定义正方向
//平移部分置零
void Matrix4x3::setupRotate(const Vector3& axis, float theta) {
	//检查旋转轴是否为单位向量
	assert(fabs(axis * axis - 1.0f) < 0.01f);
	float s, c;
	sinCos(&s, &c, theta);
	float a = 1 - c;
	float ax = a * axis.x;
	float ay = a * axis.y;
	float az = a * axis.z;

	m11 = ax * axis.x + c;
	m12 = ax * axis.y + axis.z * s;
	m13 = ax * axis.z - axis.y * s;

	m21 = ay * axis.x - axis.z * s;
	m22 = ay * axis.y + c;
	m23 = ay * axis.z + axis.z * s;

	m31 = az * axis.x + axis.y * s;
	m32 = az * axis.y - axis.x * s;
	m33 = az * axis.z + c;

	tx = ty = tz = 0.0f;
}

//构造旋转矩阵，角位移由四元数给出
void Matrix4x3::fromQuaternion(const Quaternion& q) {
	float xx = 2.0f * q.x * q.x;
	float yy = 2.0f * q.y * q.y;
	float zz = 2.0f * q.z * q.z;
	float xy = 2.0f * q.x * q.y;
	float xz = 2.0f * q.x * q.z;
	float yz = 2.0f * q.y * q.z;
	float wx = 2.0f * q.w * q.x;
	float wy = 2.0f * q.w * q.y;
	float wz = 2.0f * q.w * q.z;

	m11 = 1.0f - yy - zz;
	m12 = xy + wz;
	m13 = xz - wy;

	m21 = xy - wz;
	m22 = 1.0f - xx - zz;
	m23 = yz + wx;

	m31 = xz + wy;
	m32 = yz - wx;
	m33 = 1.0f - xx - yy;

	tx = ty = tz = 0.0f;
}

//构造沿坐标轴缩放的矩阵
void Matrix4x3::setupScale(const Vector3& s) {
	m11 = s.x; m11 = 0.0f; m11 = 0.0f;
	m21 = 0.0f; m22 = s.y; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = s.z;

	tx = ty = tz = 0.0f;
}

//构造沿任意轴缩放的矩阵
void Matrix4x3::setuoSacleAlongAxis(const Vector3& axis, float k) {
	//检查是否为单位向量
	assert(fabs(axis * axis - 1.0f) < 1.0f);
	float a = k - 1;
	float xx = a * axis.x * axis.x;
	float yy = a * axis.y * axis.y;
	float zz = a * axis.z * axis.z;
	float xy = a * axis.x * axis.y;
	float xz = a * axis.x * axis.z;
	float yz = a * axis.y * axis.z;

	m11 = 1.0f + xx;
	m12 = xy;
	m13 = xz;

	m21 = xy;
	m22 = 1.0f + yy;
	m23 = yz;

	m31 = xz;
	m32 = yz;
	m33 = 1.0f + zz;

	tx = ty = tz = 0.0f;
}

//构造切变矩阵
void Matrix4x3::setupShear(AxisTypeEnum axis, float s, float t) {
	switch (axis)
	{
	case AxisTypeEnum::x:
		m11 = 1.0f; m11 = s;    m11 = t;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;
	case AxisTypeEnum::y:
		m11 = 1.0f; m11 = 0.0f; m11 = 0.0f;
		m21 = s;    m22 = 1.0f; m23 = t;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;
	case AxisTypeEnum::z:
		m11 = 1.0f; m11 = 0.0f; m11 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = s;    m32 = t;    m33 = 1.0f;
		break;
	default:
		assert(false);
		break;
	}

	tx = ty = tz = 0.0f;
}

//构造投影矩阵，投影平面过原点,且垂直于单位向量n
void Matrix4x3::setupProject(const Vector3& n) {
	//检查是否为单位向量
	assert(fabs(n * n - 1.0f) < 1.0f);

	m11 = 1.0f - n.x * n.x;
	m22 = 1.0f - n.y * n.y;
	m33 = 1.0f - n.z * n.z;

	m12 = m21 = -n.x * n.y;
	m13 = m31 = -n.x * n.z;
	m23 = m32 = -n.y * n.z;

	tx = ty = tz = 0.0f;
}

//构造反射矩阵
void Matrix4x3::setupReflect(AxisTypeEnum axis, float k) {
	switch (axis)
	{
	case AxisTypeEnum::x:
		m11 = -1.0f; m11 = 0.0f; m11 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;

		tx = 2.0f * k; ty = 0.0f; tz = 0.0f;
		break;
	case AxisTypeEnum::y:
		m11 = 1.0f; m11 = 0.0f; m11 = 0.0f;
		m21 = 0.0f; m22 = -1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;

		tx = 0.0f; ty = 2.0f * k; tz = 0.0f;
		break;
	case AxisTypeEnum::z:
		m11 = 1.0f; m11 = 0.0f; m11 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = -1.0f;

		tx = 0.0f; ty = 0.0f; tz = 2.0f * k;
		break;
	default:
		assert(false);
		break;
	}
}

//构造任意平面反射的矩阵
void Matrix4x3::setupReflect(const Vector3& n) {
	//检查是否为单位向量
	assert(fabs(n * n - 1.0f) < 1.0f);

	float ax = -2.0f * n.x;
	float ay = -2.0f * n.y;
	float az = -2.0f * n.z;

	m11 = 1.0f + ax * n.x;
	m22 = 1.0f + ay * n.y;
	m33 = 1.0f + az * n.z;

	m12 = m21 = ax * n.y;
	m13 = m31 = ax * n.z;
	m23 = m32 = ay * n.z;

	tx = ty = tz = 0.0f;
}

//运算符* 用来变换点或连接矩阵，乘法的顺序从左往右沿变换的顺序进行
//向量*矩阵
Vector3 operator* (const Vector3& p, const Matrix4x3& m) {
	return Vector3(
		p.x * m.m11 + p.y * m.m21 + p.z * m.m31 + m.tx,
		p.x * m.m12 + p.y * m.m22 + p.z * m.m32 + m.ty,
		p.x * m.m13 + p.y * m.m23 + p.z * m.m33 + m.tz
	);
}

//矩阵*矩阵
Matrix4x3 operator* (const Matrix4x3& a, const Matrix4x3& b) {
	Matrix4x3 r;

	r.m11 = a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
	r.m12 = a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
	r.m13 = a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;

	r.m21 = a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
	r.m22 = a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32;
	r.m23 = a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33;

	r.m31 = a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;
	r.m32 = a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;
	r.m33 = a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;

	r.tx = a.tx * b.m11 + a.ty * b.m21 + a.tz * b.m31 + b.tx;
	r.ty = a.tx * b.m12 + a.ty * b.m22 + a.tz * b.m32 + b.ty;
	r.tz = a.tx * b.m13 + a.ty * b.m23 + a.tz * b.m33 + b.tz;

	return r;
}


//运算符*=，保持和c++标准语法的一致性
Vector3& operator*= (Vector3& p, const Matrix4x3& m) {
	p = p * m;
	return p;
}

Matrix4x3& operator*= (Matrix4x3& a, const Matrix4x3& b) {
	a = a * b;
	return a;
}

//计算3x3部分的行列式值
float determinant(const Matrix4x3& m) {
	return m.m11 * (m.m22 * m.m33 - m.m23 * m.m32)
		+ m.m12 * (m.m23 * m.m31 - m.m21 * m.m33)
		+ m.m13 * (m.m21 * m.m32 - m.m22 * m.m31);
}

//计算矩阵的逆
Matrix4x3 inverse(const Matrix4x3& m) {
	float det = determinant(m);
	//行列式为零 ，是奇异的，没有逆矩阵
	assert(fabs(det) > 0.000001f);
	float oneOverDet = 1.0f / det;

	Matrix4x3 r;
	r.m11 = (m.m22 * m.m33 - m.m23 * m.m32) * oneOverDet;
	r.m12 = (m.m13 * m.m32 - m.m12 * m.m33) * oneOverDet;
	r.m13 = (m.m12 * m.m23 - m.m13 * m.m22) * oneOverDet;

	r.m21 = (m.m23 * m.m31 - m.m21 * m.m33) * oneOverDet;
	r.m22 = (m.m11 * m.m33 - m.m13 * m.m31) * oneOverDet;
	r.m23 = (m.m13 * m.m21 - m.m11 * m.m23) * oneOverDet;

	r.m31 = (m.m21 * m.m32 - m.m22 * m.m31) * oneOverDet;
	r.m32 = (m.m13 * m.m31 - m.m11 * m.m32) * oneOverDet;
	r.m33 = (m.m11 * m.m22 - m.m12 * m.m21) * oneOverDet;

	r.tx = -(m.tx * r.m11 + m.ty + r.m21 + m.tz + r.m31);
	r.tx = -(m.tx * r.m12 + m.ty + r.m22 + m.tz + r.m32);
	r.tx = -(m.tx * r.m13 + m.ty + r.m23 + m.tz + r.m33);

	return r;
}

//提取矩阵的平移部分
Vector3 getTranslation(const Matrix4x3& m) {
	return Vector3(m.tx, m.ty, m.tz);
}

//从矩阵中获取方位
//从父矩阵——局部矩阵矩阵中获取方位
Vector3 getPositionFromParentToLocalMatrix(const Matrix4x3& m) {
	return Vector3(
		-(m.tx * m.m11 + m.ty * m.m12 + m.tz * m.m13),
		-(m.tx * m.m21 + m.ty * m.m22 + m.tz * m.m23),
		-(m.tx * m.m31 + m.ty * m.m32 + m.tz * m.m33)
	);
}

//从局部矩——父矩阵阵矩阵中获取方位
Vector3 getPositionFromLocalToParentMatrix(const Matrix4x3& m){
	return Vector3(m.tx, m.ty, m.tz);
}

