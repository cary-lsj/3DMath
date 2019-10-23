#pragma once
#include "Vector3.h"
#include "EulerAngles.h"
#ifndef __QUATERNION_H_INCLUDEED__
#define __QUATERNION_H_INCLUDEED__


// 名称：四元数
// 创建者：cary
// 描述：实现在3D中表示角位移的四元数
//		

class Quaternion
{
public:
	float x, y, z, w;

	//置为单元四元数
	void identity(){
		w = x = y = z = 0.0f;
	}

	void setQuaternionAboutX(float theta);
	void setQuaternionAboutY(float theta);
	void setQutaernionAboutZ(float theta);
	void setQutaernionAboutAxis(const Vector3& axis, float theta);

	//从欧拉角计算物体——惯性的四元数
	void setToRotationObjectToInertial(const EulerAngles& orientation);
	//从欧拉角计算惯性——物体的四元数
	void setToRotationInertialToObject(const EulerAngles& orientation);

	//重载* 实现叉乘
	Quaternion operator *(const Quaternion& a) const;
	//重载*= 实现叉乘并赋值
	Quaternion& operator *=(const Quaternion& a);

	//正则化
	void normalize();

	//旋转角
	float getRotationAngle() const;
	//轴
	Vector3 getRotaionAxis() const;
};

//全局“单位”四元数
extern const Quaternion kQuaternionIdentity;

//四元数点乘
extern float dotProduct(const Quaternion& a, const Quaternion& b);

//球面线性插值
extern Quaternion slerp(const Quaternion& q0, const Quaternion& q1, float t);

//四元数共轭
extern Quaternion conjugate(const Quaternion& q);

//四元数幂
extern Quaternion pow(const Quaternion& q, float exponent);

#endif // #ifndef __QUATERNION_H_INCLUDEED__