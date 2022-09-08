#include "stdafx.h"
#include "quaternion.h"
#include <math.h>

CQuaternion::CQuaternion()
{
	x = 0;
	y = 0;
	z = 0;
	w = 0;
}
CQuaternion::CQuaternion(double a,double b,double c,double d)
{
	x = a;
	y = b;
	z = c;
	w = d;
}
//������Ԫ�����
CQuaternion CQuaternion::operator +(const CQuaternion &b)
{
	CQuaternion result;
	result.x = x + b.x;
	result.y = y + b.y;
	result.z = z + b.z;
	result.w = w + b.w;
	return result;
}
//������Ԫ�����
CQuaternion CQuaternion::operator -(const CQuaternion &b)
{
	CQuaternion result;
	result.x = x -  b.x;
	result.y = y -  b.y;
	result.z = z -  b.z;
	result.w = w -  b.w;
	return result;
}
//��Ԫ��������
CQuaternion CQuaternion::operator *(const double &coff)
{
	CQuaternion result; 
	result.x = x * coff;
	result.y = y * coff;
	result.z = z * coff;
	result.w = w * coff;
	return result;
}
//��Ԫ��������
CQuaternion CQuaternion::operator /(const double &coff)
{
	CQuaternion result;
	result.x = x / coff;
	result.y = y / coff;
	result.z = z / coff;
	result.w = w / coff;
	return result;
}
//������Ԫ�����
CQuaternion CQuaternion::operator *(const CQuaternion &b)
{
	CQuaternion result;
	result.x = w*b.x + x*b.w + y*b.z - z*b.y;
	result.y = w*b.y - x*b.z + y*b.w + z*b.x;
	result.z = w*b.z + x*b.y - y*b.x + z*b.w;
	result.w = w*b.w - x*b.x - y*b.y - z*b.z;
	return result;
}

//�ƣ�a��b��c������תtheta�Ƕȵ���Ԫ��(�Ѿ���λ��)
void CQuaternion::R2Q(const double &theta,const double &a,const double &b,const double &c)
{
	double sin_2 = sin(theta*PI/360.0);
	double cos_2 = cos(theta*PI/360.0);
	double len = sqrt(a*a + b*b + c*c);
	x = a /len * sin_2;
	y = b /len * sin_2;    
	z = c /len * sin_2;
	w = cos_2;
}
//��һ����Ԫ��ת��Ϊһ��3*3����ת�����Լ��޸ľ��������չΪ4*4
void CQuaternion::Q2R(Matrix4d& q_r)
{
	q_r(0,0) = 1 - 2*y*y - 2*z*z;
	q_r(0,1) = 2*x*y - 2*z*w;
	q_r(0,2) = 2*y*w + 2*x*z;
	q_r(0,3) = 0;

	q_r(1,0) = 2*x*y + 2*z*w;
	q_r(1,1) = 1- 2*x*x - 2*z*z;
	q_r(1,2) = -2*x*w + 2*y*z;
	q_r(1,3) = 0;

	q_r(2,0) = -2*y*w + 2*x*z;
	q_r(2,1) = 2*x*w + 2*y*z;
	q_r(2,2) = 1-2*x*x - 2*y*y;
	q_r(2,3) = 0;

	q_r(3,0) = 0;
	q_r(3,1) = 0;
	q_r(3,2) = 0;
	q_r(3,3) = 1;
}

//����Ԫ����ģ
double CQuaternion::GetNorm()
{
	double result;
	result = sqrt(x*x + y*y + z*z + w*w);
	return result;
}

//����Ԫ����λ��
void CQuaternion::Normalize()
{
	if (!(x || y || z || w))
		return;
	double norm = GetNorm();

	x = x/norm;
	y = y/norm;
	z = z/norm;
	w = w/norm;
}
//��Ԫ������
void CQuaternion::GetInverse()
{
	double normex = GetNorm() * GetNorm();
	x = (-x)/normex;
	y = (-y)/normex;
	z = (-z)/normex;
	w = w;
}

//��ת����4*4��ת��Ϊ��Ԫ��
void CQuaternion::RotationMatrix2Qua(const Matrix4d& rotation)
{
	/*
	x = (rotation(0,0) + rotation(1,1) + rotation(2,2) + 1.0) / 4.0;
	y = (rotation(0,0) - rotation(1,1) - rotation(2,2) + 1.0) / 4.0;
	z = (-rotation(0,0) + rotation(1,1) - rotation(2,2) + 1.0) / 4.0;
	w = (-rotation(0,0) - rotation(1,1) + rotation(2,2) + 1.0) / 4.0;
	x = sqrt(max(x, 0.0));
	y = sqrt(max(y, 0.0));
	z = sqrt(max(z, 0.0));
	w = sqrt(max(w, 0.0));

	if (x >= y && x >= z && x >= w)
	{
		x *= 1.0;
		y *= Sign(rotation(2,1) - rotation(1,2));
		z *= Sign(rotation(0,2) - rotation(2,0));
		w *= Sign(rotation(1,0) - rotation(0,1));
	}
	else if (y >= x && y >= z && y >= w)
	{
		x *= Sign(rotation(2,1) - rotation(1,2));
		y *= 1.0;
		z *= Sign(rotation(1,0) + rotation(0,1));
		w *= Sign(rotation(0,2) + rotation(2,0));
	}
	else if (z >= x && z >= y && z >= w)
	{
		x *= Sign(rotation(0,2) - rotation(2,0));
		y *= Sign(rotation(1,0) + rotation(0,1));
		z *= 1.0;
		w *=Sign(rotation(2,1) + rotation(1,2));
	}
	else if (w >= x && w >= y && w >= z)
	{
		x *= Sign(rotation(1,0) - rotation(0,1));
		y *= Sign(rotation(0,2) + rotation(2,0));
		z *= Sign(rotation(2,1) + rotation(1,2));
		w *= 1.0;
	}

	Normalize();
	*/


	double e_1_2 = rotation(1,2);
	double e_2_0 = rotation(2,0);
	double angle = 0;
	if(e_1_2 != 0)
	{
		angle = asin(-e_1_2)/PI  * 180; 
		R2Q(angle,1,0,0);
	}
	else if(e_2_0 != 0)
	{
		angle = asin(-e_2_0)/PI * 180;
		R2Q(angle,0,1,0);
	}
}

//��Ԫ�����������Բ�ֵ
CQuaternion&  CQuaternion::Slerp(CQuaternion& qua1, CQuaternion& qua2, double t)
{
	qua1.Normalize();
	qua2.Normalize();

	float a0 ,a1;

	double cosValue = qua1.DotProduct(qua2);
	if(cosValue < 0)
	{
		/*
		w = -w;
		x = -x;
		y = -y;
		z = -z;
		*/

		qua2.w = -qua2.w;
		qua2.x = -qua2.x;
		qua2.y = -qua2.y;
		qua2.z = -qua2.z;

		cosValue = -cosValue;
	}

	if(cosValue > 0.999999)
	{
		a0 = (float)t;
		a1 = (float)(1 - t);
	}

	else
	{
		double sinValue = sqrt(1 - cosValue*cosValue);
		double ang = asin(sinValue);
		double oneOverSin = 1.0 / sinValue;
		a0 = (float)(sin((1.0-t)*ang) * oneOverSin);
		a1 = (float)(sin(t * ang) * oneOverSin);
	}

	w = a0 * qua1.w + a1 * qua2.w;
	x = a0 * qua1.x + a1 * qua2.x;
	y = a0 * qua1.y + a1 * qua2.y;
	z = a0 * qua1.z + a1 * qua2.z;

	return (*this);
}

//��Ԫ���ĵ��
double CQuaternion::DotProduct(const CQuaternion&  q)
{
	double result = 0;
	result += (x * q.x);
	result += (y * q.y);
	result += (z * q.z);
	result += (w * q.w);

	return result;
}