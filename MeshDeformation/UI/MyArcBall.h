
#pragma  once

#include "ArcBall.h"
#include "Eigen/Dense"
#include "../Utility/predefine.h"

#define MID_BTN_DOWN		1
#define RIGHT_BTN_DOWN		2
#define M_ZOOM_OUT			3
#define M_ZOOM_IN			4
#define MID_BTN_UP			5
#define RIGHT_BTN_UP		6


class MyArcBall
{
public:

	//��ת����===============================
	Matrix4fT   RotaMat;
	Matrix3fT   LastRot;
	Matrix3fT   ThisRot;
	ArcBallT*   ArcBall;
	Point2fT    MousePt;
	bool        isMClicked;
	bool        isRotaDragging;	

	//ƽ�Ʋ���===============================
	double      translateFactor;
	Matrix4fT	TranslateMat;
	Point2fT    lastPoint;
	bool        isTranslateDragging;	
	bool		isRClicked;

	//���Ų���===============================
	Matrix4fT	ScaleMat;

	int transType;

public:
	void initialize();
	void SetBounds(float w, float h);
	Matrix4fT GetTransformMat();
	void  Rotation();
	void  Translate();
	void  Scale(int zoomx);
	void  MouseEvent(int keyStyle);
	void  Convert2EigenMat(Matrix4fT& mat, Eigen::Matrix4d& EigehMat);
	Eigen::Matrix4d  GetCurrenRotaMat();
	Eigen::Matrix4d  GetCurrenTranslateMat();
}; 