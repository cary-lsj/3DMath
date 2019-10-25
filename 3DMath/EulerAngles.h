#pragma once

// 名称：欧拉角
// 创建者：cary
// 描述：欧拉角基本运算和性质的封装
//		该类用于表述 heading-pitch-bank欧拉角系统

#ifndef __UELERANGLES_H_INCLUDED
#define __UELERANGLES_H_INCLUDED


class EulerAngles {
public:
	float heading;
	float pitch;
	float bank;

	EulerAngles() {}
	EulerAngles(float h, float p, float b) :heading(h), pitch(p), bank(b) {};

	//置零
	void identity(){
		heading = pitch = bank = 0.0f;
	}

	//变换成“限制集”欧拉角
	void canonize();

	//从四元数转换到欧拉角
	//从物体——惯性四元数到欧拉角
	void fromObjectToIntertialQuaternion(const Quaternion& q);
	//从惯性——物体四元数到欧拉角
	void fromIntertialToObjectQuaternion(const Quaternion& q);

	//从矩阵转换到欧拉角
	//平移部分被省略，并且假设矩阵是正交的
	//输入矩阵假设为物体——世界转换矩阵
	void formObjectToWorldMatrix(const Matrix4x3& m);
	//输入矩阵假设为世界——物体转换矩阵
	void formWorldToObjectMatrix(const Matrix4x3& m);

	//从旋转矩阵转换到欧拉角
	void fromRotationMatrix(const RotationMatrix& m);

private:
	//将pitch变换到[-pi/2-pi/2]之间
	void warpPitchPiOver2();
};

//全局的“单位”欧拉角
extern const EulerAngles kEulerAnglesIdentity;

#endif //#ifndef __UELERANGLES_H_INCLUDED
