#include <math.h>
#include "EulerAngles.h"
#include "Quaternion.h"
#include "MathUtil.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"

// 名称：欧拉角
// 创建者：cary
// 描述：欧拉角基本运算和性质的封装
//		该类用于表述 heading-pitch-bank欧拉角系统

const EulerAngles kEulerAnglesIdentity(0.0f, 0.0f, 0.0f);

//变换成“限制集”欧拉角
void EulerAngles::canonize() {
	pitch = warpPi(pitch);
	warpPitchPiOver2();
	//检查是否在万向锁中
	if (fabs(pitch) > kPiOver2 - 1e-4) {
		heading += bank;
		bank = 0.0f;
	}
	else {
		bank = warpPi(bank);
	}

	heading = warpPi(heading);
}

//将pitch变换到[-pi/2-pi/2]之间
void EulerAngles::warpPitchPiOver2() {
	if (pitch < -kPiOver2) {
		pitch = -kPi - pitch;
		heading += kPi;
		bank += kPi;
	}
	else if (pitch > kPiOver2)
	{
		pitch = kPi - pitch;
		heading += kPi;
		bank += kPi;
	}
}

//从物体——惯性四元数到欧拉角
void EulerAngles::fromObjectToIntertialQuaternion(const Quaternion& q) {
	//计算sin(pitch)
	//m23 = -sin(pitch) = 2yz - 2wx
	float sinPicth = -2.0f * (q.y * q.z - q.w * q.x);
	//检查是否在万向锁中
	if (fabs(sinPicth) > 0.9999f) {
		//向正上方或者正下方看
		pitch = kPiOver2 * sinPicth;
		//heading = atan2(-xz + wy, 1/2 - y^2 - z^2);
		heading = atan2(-double(q.x) * double(q.z) + double(q.w) * double(q.y), 0.5f - double(q.y) * double(q.y) - double(q.z) * double(q.z));
		bank = 0.0f;
	}
	else {
		pitch = asin(sinPicth);
		//heading = atan2(xz + wy, 1/2 - x^2 - y^2);
		heading = atan2(double(q.x) * double(q.z) + double(q.w) * double(q.y), 0.5f - double(q.x) * double(q.x) - double(q.y) * double(q.y));
		//bank = atan2(xy + wz, 1/2 - x^2 - z^2);
		bank = atan2(double(q.x) * double(q.y) + double(q.w) * double(q.z), 0.5f - double(q.x) * double(q.x) - double(q.z) * double(q.z));
	}
}

//从惯性——物体四元数到欧拉角
void EulerAngles::fromIntertialToObjectQuaternion(const Quaternion& q) {
	//计算sin(pitch)
		//m23 = -sin(pitch) = 2yz + 2wx
	float sinPicth = -2.0f * (q.y * q.z + q.w * q.x);;
	//检查是否在万向锁中
	if (fabs(sinPicth) > 0.9999f) {
		//向正上方或者正下方看
		pitch = kPiOver2 * sinPicth;
		//heading = atan2(-xz - wy, 1/2 - y^2 - z^2);
		heading = atan2(-double(q.x) * double(q.z) - double(q.w) * double(q.y), 0.5f - double(q.y) * double(q.y) - double(q.z) * double(q.z));
		bank = 0.0f;
	}
	else {
		pitch = asin(sinPicth);
		//heading = atan2(xz - wy, 1/2 - x^2 - y^2);
		heading = atan2(double(q.x) * double(q.z) - double(q.w) * double(q.y), 0.5f - double(q.x) * double(q.x) - double(q.y) * double(q.y));
		//bank = atan2(xy - wz, 1/2 - x^2 - z^2);
		bank = atan2(double(q.x) * double(q.y) - double(q.w) * double(q.z), 0.5f - double(q.x) * double(q.x) - double(q.z) * double(q.z));
	}
}

//输入矩阵假设为物体——世界转换矩阵
void EulerAngles::formObjectToWorldMatrix(const Matrix4x3& m) {
	float sinPitch = -m.m32;

	//检测是否在万向锁中
	if (fabs(sinPitch) > 9.9999f) {
		pitch = kPiOver2 * sinPitch;
		heading = atan2(-m.m23, m.m11);
		bank = 0.0f;
	}
	else {
		pitch = asin(sinPitch);
		heading = atan2(m.m31, m.m33);
		bank = atan2(m.m21, m.m22);
	}
}

//输入矩阵假设为世界——物体转换矩阵
void EulerAngles::formWorldToObjectMatrix(const Matrix4x3& m) {
	float sinPitch = -m.m23;

	//检测是否在万向锁中
	if (fabs(sinPitch) > 9.9999f) {
		pitch = kPiOver2 * sinPitch;
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		pitch = asin(sinPitch);
		heading = atan2(m.m31, m.m33);
		bank = atan2(m.m21, m.m22);
	}
}

//从旋转矩阵转换到欧拉角
void EulerAngles::fromRotationMatrix(const RotationMatrix& m) {
	float sinPitch = -m.m23;

	//检测是否在万向锁中
	if (fabs(sinPitch) > 9.9999f) {
		pitch = kPiOver2 * sinPitch;
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		pitch = asin(sinPitch);
		heading = atan2(m.m31, m.m33);
		bank = atan2(m.m21, m.m22);
	}
}
