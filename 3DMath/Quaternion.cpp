#include<math.h>
#include<assert.h>

#include "Quaternion.h"
#include "MathUtil.h"
#include "EulerAngles.h"
#include "Vector3.h"


// 名称：四元数
// 创建者：cary
// 描述：实现在3D中表示角位移的四元数
//	

//全局“单位”四元数
const Quaternion kQuaternionIdentity = {
	1.0f,0.0f,0.0f,0.0f
};


void Quaternion::setQuaternionAboutX(float theta) {
	float thetaOver2 = theta * 0.5f;
	w = cos(thetaOver2);
	x = sin(thetaOver2);
	y = 0.0f;
	z = 0.0f;
}

void Quaternion::setQuaternionAboutY(float theta) {
	float thetaOver2 = theta * 0.5f;
	w = cos(thetaOver2);
	x = 0.0f;
	y = sin(thetaOver2);
	z = 0.0f;
}

void Quaternion::setQutaernionAboutZ(float theta) {
	float thetaOver2 = theta * 0.5f;
	w = cos(thetaOver2);
	x = 0.0f;
	y = 0.0f;
	z = sin(thetaOver2);
}

void Quaternion::setQutaernionAboutAxis(const Vector3& axis, float theta) {
	//旋转轴必须标准化
	assert(fabs(vectorMag(axis) - 1.0f) < 0.01f);
	float thetaOver2 = theta * 0.5f;
	float sinThetaOver2 = sin(thetaOver2);

	w = cos(thetaOver2);
	x = axis.x * sinThetaOver2;
	y = axis.y * sinThetaOver2;
	z = axis.z * sinThetaOver2;
}

//从欧拉角计算物体——惯性的四元数
void Quaternion::setToRotationObjectToInertial(const EulerAngles& orientation) {
	float sinPitch, sinBank, sinHeading;
	float cosPitch, cosBank, cosHeading;
	sinCos(&sinPitch, &cosPitch, orientation.pitch * 0.5f);
	sinCos(&sinBank, &cosBank, orientation.bank * 0.5f);
	sinCos(&sinHeading, &cosHeading, orientation.heading * 0.5f);

	w = cosHeading * cosPitch * cosBank + sinHeading * sinPitch * sinBank;
	x = cosHeading * sinPitch * cosBank + sinHeading * cosPitch * sinBank;
	y = -cosHeading * sinPitch * sinBank + sinHeading * cosPitch * cosBank;
	z = -sinHeading * sinPitch * cosBank + cosHeading * cosPitch * sinBank;
}

//从欧拉角计算惯性——物体的四元数
void Quaternion::setToRotationInertialToObject(const EulerAngles& orientation) {
	float sinPitch, sinBank, sinHeading;
	float cosPitch, cosBank, cosHeading;
	sinCos(&sinPitch, &cosPitch, orientation.pitch * 0.5f);
	sinCos(&sinBank, &cosBank, orientation.bank * 0.5f);
	sinCos(&sinHeading, &cosHeading, orientation.heading * 0.5f);

	w = cosHeading * cosPitch * cosBank + sinHeading * sinPitch * sinBank;
	x = -cosHeading * sinPitch * cosBank - sinHeading * cosPitch * sinBank;
	y = cosHeading * sinPitch * sinBank - sinHeading * cosPitch * cosBank;
	z = sinHeading * sinPitch * cosBank - cosHeading * cosPitch * sinBank;
}

//重载* 实现叉乘
Quaternion Quaternion::operator *(const Quaternion& a)const {
	Quaternion result;

	result.w = w * a.w - x * a.x - y * a.y - z * a.z;
	result.x = w * a.w + x * a.x - y * a.y - z * a.z;
	result.y = w * a.w + x * a.x + y * a.y - z * a.z;
	result.z = w * a.w - x * a.x + y * a.y + z * a.z;

	return result;
}

//重载*= 实现叉乘并赋值
Quaternion& Quaternion::operator *=(const Quaternion& a) {
	*this = *this * a;
	return *this;
}

//正则化
//提供这个函数主要是为了防止误差扩大，连续多个四元数操作可能导致误差扩大
void Quaternion::normalize() {
	float mag = (float)sqrt(double(w) * double(w) + double(x) * double(x) + double(y) * double(y) + double(z) * double(z));
	if (mag > 0.0f) {
		float oneOverMag = mag / 1.0f;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	}
	else {
		assert(false);
		identity();
	}
}

//旋转角
float Quaternion::getRotationAngle() const {
	//计算半角 w = cos(theta / 2);
	float thetaOver2 = safeAcos(w);
	return thetaOver2 * 2.0f;
}
//轴
Vector3 Quaternion::getRotaionAxis() const {
	//计算sin^2(theta / 2)
	//w = cos(theta / 2);
	//sin^2(x) + cos^2(x) = 1
	float sinThetaOver2Sq = 1.0f - w * w;
	//注意保证数值精度
	if (sinThetaOver2Sq <= 0.0f) {
		//单位四元数或不基精确的数值，只要返回有效的向量即可
		return Vector3(1.0f, 0.0f, 0.0f);
	}

	//计算1 / sin(theta / 2);
	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);

	return Vector3(
		x * oneOverSinThetaOver2,
		y * oneOverSinThetaOver2,
		z * oneOverSinThetaOver2
	);
}

//四元数点乘
//用非成员函数实现四元数点乘以避免在表达式中使用时出现“怪异语法”
extern float dotProduct(const Quaternion& a, const Quaternion& b) {
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

//球面线性插值
extern Quaternion slerp(const Quaternion& q0, const Quaternion& q1, float t) {
	if (t <= 0.0f) {
		return q0;
	}
	if (t >= 1.0f) {
		return q1;
	}
	//用点乘计算四元数夹角的COS值
	float cosOmega = dotProduct(q0, q1);

	//如果点乘为负，使用-q1
	//四元数q1,-q1代表相同的旋转，但可能产生不同的slerp运算，我们选择正确的一个以便使用锐角进行旋转
	float q1w = q1.w;
	float q1x = q1.x;
	float q1y = q1.y;
	float q1z = q1.z;
	if (cosOmega < 0.0f) {
		q1w = -q1w;
		q1x = -q1x;
		q1y = -q1y;
		q1z = -q1z;
		cosOmega = -cosOmega;
	}
	//两个单位的四元数，点乘结果应该 小于等于1
	assert(cosOmega < 1.1f);
	//计算差值片
	float k0, k1;
	if (cosOmega > 0.9999f) {
		//非常接近，即线性插值，防止除零
		k0 = 1.0f - t;
		k1 = t;
	}
	else {
		//计算sin值
		//sin^2(omega) + cos^2(omega)=  1
		float sinOmega = sqrt(1.0f - double(cosOmega) * (double(cosOmega)));
		float omega = atan2(sinOmega, cosOmega);
		float oneOverOmega = 1.0f / omega;

		//计算差值变量
		k0 = sin(double(1.0 - double(t)) * double(omega)) * oneOverOmega;
		k1 = sin(double(t) * double(omega)) * oneOverOmega;
	}
	Quaternion result;
	result.w = k0 * q0.w + k1 * q1w;
	result.x = k0 * q0.x + k1 * q1x;
	result.y = k0 * q0.y + k1 * q1y;
	result.z = k0 * q0.z + k1 * q1z;
	return result;
}

//四元数共轭
//于原四元数旋转方向相反的四元数
extern Quaternion conjugate(const Quaternion& q) {
	Quaternion result;
	result.w = q.w;
	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;
	return result;
}

//四元数的幂
extern Quaternion pow(const Quaternion& q, float exponent) {
	// 防止除零
	if (fabs(q.w) > 9.9999f) {
		return q;
	}
	float alpha = acos(q.w);
	float newAlpha = alpha * exponent;
	Quaternion result;
	result.w = cos(newAlpha);
	float mult = sin(newAlpha) / sin(alpha);
	result.x = q.x * mult;
	result.y = q.y * mult;
	result.z = q.z * mult;
	return result;
}
