#include<math.h>
#include "MathUtil.h"
#include "Vector3.h"

// 名称：数学工具类
// 创建者：cary
// 描述：封装一些常见的数学公式

const Vector3 kZeroVector(0.0f, 0.0f, 0.0f);

// 通过加适当的2pi倍数将角度限制在[-pi,pi]
float warpPi(float theta) {
	theta += kPi;
	theta -= floor(theta * k1Over2Pi) * k2Pi;
	theta -= kPi;
	return theta;
}

// "安全" 反三角函数
// 和acose(x)相同，但如果x超出范围将返回最为接近的有效值
// 返回值在[0,pi]，和c语言中的标准acos()的函数相同
extern float safeAcos(float x) {
	if (x <= -1.0f) {
		return kPi;
	}
	if (x >= 1.0f) {
		return 0.0f;
	}
	//使用标准C函数
	return acos(x);
}
