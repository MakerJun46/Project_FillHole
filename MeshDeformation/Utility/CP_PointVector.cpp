// CP_PointVector.cpp ʵ����Point2D��Point3D��Vector2D��Vector3D
#include "stdafx.h"
#include "CP_PointVector.h"

#include <math.h>

// ////////////////////////////////////////////////////////////////////////////
// ʵ����Point2D��ʼ
Point2D::Point2D(double newx, double newy):m_x(newx), m_y(newy)
{
} // ��Point2D���캯������
// ʵ����Point2D����
// ////////////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////
// ʵ����Point3D��ʼ
void Point3D::operator=(Point3D point)
{
	m_x = point.m_x;
	m_y = point.m_y;
	m_z = point.m_z;
}

void Point3D::operator/=(int num)
{
	m_x /= (double)num;
	m_y /= (double)num;
	m_z /= (double)num;
}
Point3D& Point3D::operator += (const Point3D& p)
{
	m_x += p.m_x;
	m_y += p.m_y;
	m_z += p.m_z;

	return *this;
}

Point3D Point3D::operator+(Point3D point)
{
	Point3D p; 
	p.m_x = m_x + point.m_x;
	p.m_y = m_y + point.m_y;
	p.m_z = m_z + point.m_z;

	return p;
}

Point3D& Point3D::operator-=(Point3D point)
{
	m_x -= point.m_x;
	m_y -= point.m_y;
	m_z -= point.m_z;

	return *this;
}

Point3D& Point3D::operator*=(double num)
{
	m_x *= num;
	m_y *= num;
	m_z *= num;

	return *this;
}

Point3D Point3D::operator*(double num)
{
	Point3D ret;
	ret.m_x = m_x * num;
	ret.m_y = m_y * num;
	ret.m_z = m_z * num;

	return ret;
}

Point3D::Point3D(double newx, double newy, double newz):m_x(newx), m_y(newy), m_z(newz)
{
} // ��Point3D���캯������
// ʵ����Point3D����
// ////////////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////
// ʵ����Point3D��ʼ
Vector2D::Vector2D (double newx, double newy):m_x(newx), m_y(newy)
{
} // ��Vector2D���캯������

Vector2D& Vector2D::operator += (const Vector2D& v)
{ 
	m_x += v.m_x;
	m_y += v.m_y; 
	return *this; 
} //��Ա����operator +=����

Vector2D& Vector2D::operator -= (const Vector2D& v)
{ 
	m_x -= v.m_x;
	m_y -= v.m_y; 
	return *this; 
} //��Ա����operator -=����

Vector2D& Vector2D::operator *= (double num)
{
	m_x *= num;
	m_y *= num; 
	return *this; 
} //��Ա����operator *=����

Vector2D& Vector2D::operator /= (double num)
{
	m_x /= num;  // ע������û�д������Ϊ0������
	m_y /= num; 
	return *this;
} //��Ա����operator /=����

double Vector2D::operator ^(const Vector2D& v)
{
	return( m_x* v.m_y- m_y* v.m_x );
} //��Ա����operator ^����

Vector2D Vector2D::operator - () const
{ 
	return Vector2D (-m_x, -m_y); 
} //��Ա����operator -����

double Vector2D::mf_getLength( )  const                             
{ 
	return sqrt(m_x*m_x + m_y*m_y); 
} //��Ա����mf_getLength����

Vector2D Vector2D::mf_getPerpendicularVector() const
{
	return Vector2D(-m_y, m_x);
} //��Ա����mf_getPerpendicularVector����

void Vector2D::mf_normalize( )
{
	double a = mf_getLength( );
	(*this) /= a; // ע��: ����û�д�������Ϊ0�����
} //��Ա����mf_normalize����

void Vector2D::mf_setValue(double newx, double newy)
{
	m_x=newx;
	m_y=newy;
} //��Ա����mf_setValue����

// ʵ����Vector2D����
// ////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////
// ʵ����Vector3D��ʼ
Vector3D::Vector3D (double newx, double newy, double newz):m_x(newx), m_y(newy), m_z(newz)
{
} // ��Vector3D���캯������

Vector3D::Vector3D (const Point3D start, const Point3D end)
{
	m_x = end.m_x - start.m_x;
	m_y = end.m_y - start.m_y;
	m_z = end.m_z - start.m_z;
}

Vector3D& Vector3D::operator += (const Vector3D& v)
{ 
	m_x += v.m_x;
	m_y += v.m_y;
	m_z += v.m_z;  
	return *this; 
} //��Ա����operator +=����

Vector3D& Vector3D::operator -= (const Vector3D& v)
{
	m_x -= v.m_x;
	m_y -= v.m_y;
	m_z -= v.m_z; 
	return *this; 
} //��Ա����operator -=����

Vector3D& Vector3D::operator *= (double num)
{ 
	m_x *= num;
	m_y *= num;
	m_z *= num; 
	return *this; 
} //��Ա����operator *=����

Vector3D& Vector3D::operator /= (double num)
{
	num = 1.0/num;
	m_x *= num;
	m_y *= num;
	m_z *= num;
	return *this;
} //��Ա����operator /=����

Vector3D& Vector3D::operator ^= (const Vector3D& v)
{ 
	double a =   m_y * v.m_z - m_z * v.m_y;
	double b = - m_x * v.m_z + m_z * v.m_x;
	double c =   m_x * v.m_y - m_y * v.m_x;

	m_x = a;
	m_y = b;
	m_z = c;
	return *this;
} //��Ա����operator ^=����

bool Vector3D::operator == (const Vector3D& v)
{
	return (abs(m_x - v.m_x) < 1e-8) && (abs(m_y - v.m_y) < 1e-8) && (abs(m_z - v.m_z) < 1e-8);
}

Vector3D Vector3D::operator - ( ) const
{ 
	return Vector3D (-m_x, -m_y, -m_z); 
} //��Ա����operator -����

double Vector3D::mf_getLength( )  const                             
{ 
	return sqrt(m_x*m_x + m_y*m_y + m_z*m_z); 
} //��Ա����mf_getLength����

Vector3D Vector3D::mf_getPerpendicularVector( ) const
{
	Vector3D vecReturn;
	if(fabs(m_y)<fabs(m_z))
	{
		vecReturn.m_x=m_z;
		vecReturn.m_y=0.0;
		vecReturn.m_z=-m_x;
	}
	else
	{
		vecReturn.m_x=-m_y;
		vecReturn.m_y=m_x;
		vecReturn.m_z=0.0;
	}
	return vecReturn;
} //��Ա����mf_getPerpendicularVector����

void Vector3D::mf_normalize( )
{
	double a = mf_getLength( );
	(*this) /= a; // ע��: ����û�д������Ϊ0�����
} //��Ա����mf_normalize����

void Vector3D::mf_setValue(double newx, double newy, double newz)
{
	m_x=newx;
	m_y=newy;
	m_z=newz;
} //��Ա����mf_setValue����

Face::Face(int m0,int m1, int m2)
{
	v0 = m0;
	v1 = m1;
	v2 = m2;
}

// ʵ����Vector3D����
// ////////////////////////////////////////////////////////////////////////////

Point2D operator + (const Point2D& p, const Vector2D& v)
{
	return Point2D (p.m_x + v.m_x, p.m_y + v.m_y); 
} //����operator +����

Point2D operator - (const Point2D& pt, const Vector2D& v)
{
	return Point2D (pt.m_x - v.m_x, pt.m_y - v.m_y); 
} //����operator -����

Vector2D operator - (const Point2D& p, const Point2D& q)
{
	return Vector2D (p.m_x - q.m_x, p.m_y - q.m_y); 
} //����operator -����

Point3D operator + (const Point3D& pt, const Vector3D& v)
{
	return Point3D (pt.m_x + v.m_x, pt.m_y + v.m_y, pt.m_z + v.m_z); 
} //����operator +����

Point3D operator - (const Point3D& pt, const Vector3D& v)
{
	return Point3D (pt.m_x - v.m_x, pt.m_y - v.m_y, pt.m_z - v.m_z); 
} //����operator -����

Vector3D operator - (const Point3D& p, const Point3D& q)
{
	return Vector3D (p.m_x - q.m_x, p.m_y - q.m_y, p.m_z - q.m_z); 
} //����operator -����

Vector2D operator + (const Vector2D& u, const Vector2D& v)
{
	return Vector2D (u.m_x + v.m_x, u.m_y + v.m_y); 
} //����operator +����

Vector2D operator - (const Vector2D& u, const Vector2D& v)
{
	return Vector2D (u.m_x - v.m_x, u.m_y - v.m_y);
} //����operator -����

// ���
double  operator * (const Vector2D& u, const Vector2D& v)
{
	return u.m_x * v.m_x + u.m_y * v.m_y;
} //����operator *����

Vector2D operator * (const Vector2D& v, double num)
{
	return Vector2D (v.m_x * num, v.m_y * num);
} //����operator *����

Vector2D operator / (const Vector2D& v, double num)
{
	return Vector2D (v.m_x / num, v.m_y / num); // ע��: ����û�д������Ϊ0�����
} //����operator /����

Vector3D operator + (const Vector3D& u, const Vector3D& v)
{
	return Vector3D(u.m_x + v.m_x, u.m_y + v.m_y, u.m_z + v.m_z);
} //����operator +����

Vector3D operator - (const Vector3D& u, const Vector3D& v)
{
	return Vector3D (u.m_x - v.m_x, u.m_y - v.m_y, u.m_z - v.m_z);
} //����operator -����

// ���
double operator * (const Vector3D& u, const Vector3D& v)
{
	return (u.m_x * v.m_x+u.m_y * v.m_y+ u.m_z * v.m_z);
} //����operator *����

// ���
Vector3D operator ^ (const Vector3D& u, const Vector3D& v)
{
	return Vector3D((u.m_y*v.m_z)-(u.m_z*v.m_y), 
		(-u.m_x*v.m_z)+(u.m_z*v.m_x),
		(u.m_x*v.m_y)-(u.m_y*v.m_x));
} //����operator ^����

Vector3D operator * (const Vector3D& v, double num)
{
	return Vector3D (v.m_x * num, v.m_y * num, v.m_z * num);
} //����operator *����

Vector3D operator / (const Vector3D& v, double num)
{
	num = 1.0/num; // ע��: ����û�д������Ϊ0�����
	return Vector3D (v.m_x * num, v.m_y * num, v.m_z * num);
} //����operator /����

double  GetDistance(const Point3D p1, const Point3D p2)
{
	Vector3D p1_2 = p1 - p2;
	return p1_2.mf_getLength();
}

Vector3D& NormalizeVector3D(Vector3D& n)
{
	double len = sqrt(n.m_x * n.m_x + n.m_y * n.m_y + n.m_z * n.m_z);
	ASSERT(len != 0);

	n.m_x = n.m_x / len;
	n.m_y = n.m_y / len;
	n.m_z = n.m_z / len;
	return n;
}

double SinValue(const Vector3D& a, const Vector3D& b, const Vector3D& c)
{
	double lab = (b - a).mf_getLength();
	double lac = (c - a).mf_getLength();
	return ((b-a)^(c-a)).mf_getLength()/(lab*lac);
}

double CosValue(const Vector3D& a, const Vector3D& b, const Vector3D& c)
{
	double lab = (b - a).mf_getLength();
	double lac = (c - a).mf_getLength();
	double lbc = (b - c).mf_getLength();
	double lab2 = lab*lab;
	double lac2 = lac*lac;
	double lbc2 = lbc*lbc;
	return (lab2+lac2-lbc2)/(2.0*lab*lac);
}

double CotValue(const Vector3D& a, const Vector3D& b, const Vector3D& c)
{
	double cosx = CosValue(a,b,c);
	double sinx = MaxValue(SinValue(a,b,c),1e-8);
	double cotx = cosx/sinx;
	return cotx;
}

double MaxValue(double value1, double value2)
{
	if(value1 > value2)
		return value1;
	else
		return value2;
}