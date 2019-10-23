#pragma once

#ifndef __MATHUTIL_H_INCLUDED__
#define __MATHUTIL_H_INCLUDED__

#include <math.h>

// 名称：数学工具类
// 创建者：cary
// 描述：封装一些常见的数学公式

// 定义和pi有关的常量

const float kPi = 3.1415926f;
const float k2Pi = kPi * 2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;
// 通过加适当的2pi倍数将角度限制在[-pi,pi]
extern float warpPi(float theta);
// "安全" 反三角函数
// 和acose(x)相同，但如果x超出范围将返回最为接近的有效值
// 返回值在[0,pi]，和c语言中的标准acos()的函数相同
extern float safeAcos(float x);
// 计算角度的sin和cose值
//在某些平台上，如果需要这两个值，同时计算比分开计算快
inline void sinCos(float* returnSin, float* returnCos, float theta) {
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}

#endif //#ifndef __MATHUTIL_H_INCLUDED__
