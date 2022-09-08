// ��� MFC ʾ��Դ������ʾ���ʹ�� MFC Microsoft Office Fluent �û����� 
// (��Fluent UI��)����ʾ�������ο���
// ���Բ��䡶Microsoft ������ο����� 
// MFC C++ ������渽����ص����ĵ���
// ���ơ�ʹ�û�ַ� Fluent UI ����������ǵ����ṩ�ġ�
// ��Ҫ�˽��й� Fluent UI ��ɼƻ�����ϸ��Ϣ�������  
// http://msdn.microsoft.com/officeui��
//
// ��Ȩ����(C) Microsoft Corporation
// ��������Ȩ����

// MeshDeformationView.h : CMeshDeformationView ��Ľӿ�
//

#pragma once
#include "../Utility/baseHeadFile.h"
#include "UIControl.h"
#include "../MeshDeformationDoc.h"
#include "glut.h"
#include "MyArcBall.h"


class CMeshDeformationView : public CView
{
protected: // �������л�����
	CMeshDeformationView();
	DECLARE_DYNCREATE(CMeshDeformationView)

// ����
public:
	CMeshDeformationDoc* GetDocument() const;

// ����
private:
	//����ı任
	int m_transState;
	CPoint m_prePoint;
	bool m_isRotation;
	MyArcBall modelTransObj;


public:
	MyArcBall handleTransObj;
	//��ʾ����
	UIControl m_uiControl;
	int m_viewWidth;
	int m_viewHeight;
	GLint    m_viewport[4]; 
	GLdouble m_modelview[16]; 
	GLdouble m_projection[16]; 
	double m_transFactor;

	//ʰȡ
	bool m_startPick;
	CPoint m_lButtonDown;
	CPoint m_lButtonUp;
	CPoint m_rButtonDown;
	CPoint m_rButtonUp;
	CPoint m_mButtonDown;
	bool m_lbuttonPushed;
	CPoint m_movePoint;
	CClientDC *m_pDC;

	//hand�ı任
	double m_hand_scale;
	Vector2D m_hand_angle;
	Vector3D m_hand_translate;

// ��д
public:
	virtual void OnDraw(CDC* pDC);  // ��д�Ի��Ƹ���ͼ
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// ʵ��
public:
	virtual ~CMeshDeformationView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:
	HGLRC  m_hRC;
// ���ɵ���Ϣӳ�亯��
protected:
	afx_msg void OnFilePrintPreview();
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()

public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);


	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnMButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);


	afx_msg void OnWireFrame(){theApp.m_triMesh.SetMeshDisMode(BOUNDARY_MESH); Invalidate();}
	afx_msg void OnNoWireFrame(){theApp.m_triMesh.SetMeshDisMode(SMOOTH_MESH); Invalidate();}
	afx_msg void OnPointcloud(){ theApp.m_triMesh.SetMeshDisMode(POINT_ONLY_MESH); Invalidate();}
	afx_msg void OnSelectSingleVertex(){ theApp.m_pickMesh.SetPickStyle(true);}
	afx_msg void OnSelectFrame(){ theApp.m_pickMesh.SetPickStyle(false);}
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg int  GetTransState(){return m_transState;}


public:
	void updateView();    //when new mesh open
	void initTransParameter(); 
	void ShowLight(void); //lighting
	Point3D ProjectScreen2World(CPoint point);
	afx_msg void OnMeshHide();

	
};

#ifndef _DEBUG  // MeshDeformationView.cpp �еĵ��԰汾
inline CMeshDeformationDoc* CMeshDeformationView::GetDocument() const
   { return reinterpret_cast<CMeshDeformationDoc*>(m_pDocument); }
#endif 