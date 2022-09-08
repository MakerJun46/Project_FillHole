// CP_PointVector.h: ������Point2D��Point3D��Vector2D��Vector3D
#ifndef CP_POINTVECTOR_H
#define CP_POINTVECTOR_H

#define PI2         6.28318530717958647692
#define PI          3.14159265358979323846
#define PI_2        1.57079632679489661923 

class Point2D
{
public:
	double    m_x, m_y;
public:
	//���캯��
	Point2D (double newx=0.0, double newy=0.0);
};

class Point3D
{
public:
	double    m_x, m_y, m_z;
public:
	//���캯��
	Point3D (double newx=0.0, double newy=0.0, double newz=0.0);
	void operator=(Point3D point);	
	Point3D& operator*=(double num);	
	Point3D operator*(double num);	
	Point3D operator+(Point3D point);	
	void operator/=(int num);
	Point3D& operator += (const Point3D& p);
	Point3D& operator-=(Point3D point);	
};

class Vector2D
{
public:
	double    m_x, m_y;

public:
	Vector2D (double newx=0.0, double newy=0.0);

	// ��ֵ����
	Vector2D& operator += (const Vector2D& v);
	Vector2D& operator -= (const Vector2D& v);
	Vector2D& operator *= (double num);
	Vector2D& operator /= (double num);
	double operator ^(const Vector2D& v);

	//��Ŀ��
	Vector2D operator - ( ) const;

	double mf_getLength( ) const; // ȡ����
	Vector2D mf_getPerpendicularVector( ) const; //�õ�һ����ֱ������

	void mf_normalize( ); // ��λ��
	void mf_setValue(double newx=0.0, double newy=0.0);
};

class Vector3D
{
public:
	double    m_x, m_y, m_z;

public:
	Vector3D (double newx=0.0, double newy=0.0, double newz=0.0);
	Vector3D (const Point3D start, const Point3D end);

	//��ֵ����
	Vector3D& operator += (const Vector3D& v);
	Vector3D& operator -= (const Vector3D& v);
	Vector3D& operator *= (double num);
	Vector3D& operator /= (double num);
	Vector3D& operator ^= (const Vector3D& v);
	bool operator == (const Vector3D& v);

	//��Ŀ��
	Vector3D operator - () const;

	double mf_getLength ( ) const; // ȡ����
	Vector3D mf_getPerpendicularVector( ) const; //�õ�һ����ֱ������

	void mf_normalize( ); // ��λ��
	void mf_setValue(double newx=0.0, double newy=0.0,double newz=0.0);
};

//face���涥�������
class Face
{
public:
	int v0;
	int v1;
	int v2;
public:
	Face(int m0,int m1, int m2);
};

extern Point2D operator + (const Point2D& pt, const Vector2D& v);
extern Point2D operator - (const Point2D& pt, const Vector2D& v);
extern Vector2D operator - (const Point2D& p, const Point2D& q);

extern Point3D operator + (const Point3D& pt, const Vector3D& v);
extern Point3D operator - (const Point3D& pt, const Vector3D& v);
extern Vector3D operator - (const Point3D& p, const Point3D& q);

extern Vector2D operator + (const Vector2D& u, const Vector2D& v); 
extern Vector2D operator - (const Vector2D& u, const Vector2D& v); 
extern double  operator * (const Vector2D& u, const Vector2D& v); // ���
extern Vector2D operator * (const Vector2D& v, double num);
extern Vector2D operator / (const Vector2D& v, double num); 

extern Vector3D operator + (const Vector3D& u, const Vector3D& v);
extern Vector3D operator - (const Vector3D& u, const Vector3D& v);
extern double operator * (const Vector3D& u, const Vector3D& v); // ���
extern Vector3D operator ^ (const Vector3D& u, const Vector3D& v); // ���
extern Vector3D operator * (const Vector3D& v, double num);
extern Vector3D operator / (const Vector3D& v, double num);
extern double  GetDistance(const Point3D p1, const Point3D p2);
extern Vector3D& NormalizeVector3D(Vector3D& n);

double SinValue(const Vector3D& a, const Vector3D& b, const Vector3D& c);
double CosValue(const Vector3D& a, const Vector3D& b, const Vector3D& c);
double CotValue(const Vector3D& a, const Vector3D& b, const Vector3D& c);
double MaxValue(double value1, double value2);

#endif 