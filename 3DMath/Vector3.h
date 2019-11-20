#pragma once
#include <math.h>

// 名称：3D向量
// 创建者：cary
// 描述：对3D向量基本运算的封装

class Vector3
{
public:
	float x, y, z;

	Vector3() {}

	Vector3(const Vector3& a) :x(a.x), y(a.y), z(a.z) {}

	Vector3(float nx, float ny, float nz) :x(nx), y(ny), z(nz) {}

	Vector3 operator = (const Vector3& a) {
		x = a.x;
		y = a.y;
		z = a.z;
		return *this;
	}

	bool operator == (const Vector3& a) {
		return x == a.x && y == a.y && z == a.z;
	}

	bool operator != (const Vector3& a) {
		return x != a.x || y != a.y || z != a.z;
	}

	//归零
	void zero() {
		x = y = z = 0.0f;
	}

	Vector3 operator - () const {
		return Vector3(-x, -y, -z);
	}

	Vector3 operator + (const Vector3& a) const {
		return Vector3(x + a.x, y + a.y, z + a.z);
	}

	Vector3 operator - (const Vector3& a) const {
		return Vector3(x - a.x, y - a.y, z - a.z);
	}

	Vector3 operator * (const float a) const {
		return Vector3(x * a, y *a, z * a);
	}

	Vector3 operator / (const float a) const {
		float reciprocal = 1.0f / a;
		return Vector3(x * reciprocal, y * reciprocal, z * reciprocal);
	}

	Vector3 operator += (const Vector3& a) {
		x += a.x;
		y += a.y;
		z += a.z;
		return *this;
	}

	Vector3 operator -= (const Vector3& a) {
		x -= a.x;
		y -= a.y;
		z -= a.z;
		return *this;
	}

	Vector3 operator *= (const float a) {
		x *= a;
		y *= a;
		z *= a;
		return *this;
	}

	Vector3 operator /= (const float a) {
		float reciprocal = 1.0f / a;
		x *= reciprocal;
		y *= reciprocal;
		z *= reciprocal;
		return *this;
	}

	//向量标准化
	void normalize() {
		float sumOfSquare = x * x + y * y + z * z;
		if (sumOfSquare > 0.0f) {
			float reciprocal = 1.0f / sqrt(sumOfSquare);
			x *= reciprocal;
			y *= reciprocal;
			z *= reciprocal;
		}
	}

	float operator * (const Vector3 &a) const {
		return x * a.x + y * a.y + z * a.z;
	}
};

//求向量的膜
inline float vectorMag(const Vector3& a) {
	return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

//计算两向量的叉乘
inline Vector3 crossProduct(const Vector3 &a, const Vector3& b) {
	return Vector3(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y - b.x
	);
}

//实现标量左乘
inline Vector3 operator *(float k,const Vector3 &v) {
	return Vector3(k * v.x, k * v.y,k*v.z);
}

//计算两点间的距离
inline float distance(const Vector3& a, const Vector3& b) {
	double dx = double(a.x )- double( b.x);
	double dy = double(a.y) - double(b.y);
	double dz = double(a.z) - double(b.z);
	return sqrt(dx * dx + dy * dy + dz * dz);

}

//提供一个全局零向量
extern const Vector3 kZeroVector;