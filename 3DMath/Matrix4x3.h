#pragma once
#ifndef __MATRIX4X3_H_INDECUDED__
#define __MATRIX4X3_H_INDECUDED__

class Vector3;
class EulerAngles;
class Quaternion;
class RotationMatrix;

enum AxisTypeEnum
{
	x,
	y,
	z
};

// 名称：4X3矩阵
// 创建者：cary
// 描述：实现4X3矩阵，能够表达任何3D仿射变换
//		

class Matrix4x3
{
public:
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
	float tx, ty, tz;

	//置为单位矩阵
	void identity();

	//直接访问平移部分
	//将包含平移部分的第四行置为零
	void zeroTranslation();
	//平移部分赋值
	void setTranslation(const Vector3& d);
	void setupTranslation(const Vector3& d);

	//构造执行父空间<——>局部空间变换的矩阵
	void setupLocalToParent(const Vector3& pos, const EulerAngles& orient);
	void setupLocalToParent(const Vector3& pos, const RotationMatrix& orient);
	void setupParentToLocal(const Vector3& pos, const EulerAngles& orient);
	void setupParentToLocal(const Vector3& pos, const RotationMatrix& orient);

	//构造绕坐标轴旋转的矩阵
	//axis:旋转轴的索引
	//theta 旋转的弧度，左手法则定义正方向
	//平移部分置零
	void setupRotate(AxisTypeEnum& axis, float theta);

	//构造绕任意轴旋转的矩阵
	//旋转轴通过原点，旋转轴为单位向量
	//theta 旋转的弧度，左手法则定义正方向
	//平移部分置零
	//
	void setupRotate(const Vector3& axis, float theta);

	//构造旋转矩阵，角位移由四元数给出
	void fromQuaternion(const Quaternion& q);

	//构造沿坐标轴缩放的矩阵
	void setupScale(const Vector3& s);

	//构造沿任意轴缩放的矩阵
	void setuoSacleAlongAxis(const Vector3& axis, float k);

	//构造切变矩阵
	void setupShear(AxisTypeEnum axis, float s, float t);

	//构造投影矩阵，投影平面过原点,且垂直于单位向量n
	void setupProject(const Vector3& n);

	//构造反射矩阵
	void setupReflect(AxisTypeEnum axis, float k = 0.0f);

	//构造任意平面反射的矩阵
	void setupReflect(const Vector3& n);
};

//运算符* 用来变换点或连接矩阵，乘法的顺序从左往右沿变换的顺序进行
Vector3 operator* (const Vector3& p, const Matrix4x3& m);
Matrix4x3 operator* (const Matrix4x3& a, const Matrix4x3& b);


//运算符*=，保持和c++标准语法的一致性
Vector3& operator*= (Vector3& p, const Matrix4x3& m);
Matrix4x3& operator*= (Matrix4x3& a, const Matrix4x3& b);

//计算3x3部分的行列式值
float determinant(const Matrix4x3& m);

//计算矩阵的逆
Matrix4x3 inverse(const Matrix4x3& m);

//提取矩阵的平移部分
Vector3 getTranslation(const Matrix4x3& m);

//从矩阵中获取方位
//从父矩阵——局部矩阵矩阵中获取方位
Vector3 getPositionFromParentToLocalMatrix(const Matrix4x3& m);
//从局部矩——父矩阵阵矩阵中获取方位
Vector3 getPositionFromLocalToParentMatrix(const Matrix4x3& m);

#endif // #ifdef __MATRIX4X3_H_INDECUDED__
