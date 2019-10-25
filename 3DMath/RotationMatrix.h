#pragma once
#ifndef _ROTATIONMATRIX_N_INCLUDED
#define _ROTATIONMATRIX_N_INCLUDED

class EulerAngles;
class Quaternion;
class Vector3;

// 名称：旋转矩阵
// 创建者：cary
// 描述：实现在一个简单的3X3矩阵，仅用作旋转
//		

class RotationMatrix
{
public:
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;

	//置为单位矩阵
	void identity();

	//根据指定方向构造矩阵
	//用欧拉角构造矩阵
	void setup(const EulerAngles& orientation);

	//根据四元数构造矩阵，该四元数参数代表指定方向的变换
	//根据惯性——物体旋转四元数构造矩阵
	void formIntertialToObjectQuaternion(const Quaternion& q);
	//根据物体——惯性旋转四元数构造矩阵
	void formObjectToIntertialQuaternion(const Quaternion& q);

	//执行旋转
	//惯性——物体矩阵旋转
	Vector3 intertialToObject(const Vector3& v)const;
	//物体——惯性矩阵旋转
	Vector3 objectToIntertial(const Vector3& v)const;

};

#endif // #ifndef _ROTATIONMATRIX_N_INCLUDED