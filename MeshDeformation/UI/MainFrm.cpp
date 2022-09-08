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

// MainFrm.cpp : CMainFrame ���ʵ��
//

#include "stdafx.h"
#include "MainFrm.h"
#include "../MeshDeformation.h"
#include "../resource.h"
#include "../Utility/predefine.h"
#include "../MeshDeform/PoissonDeform/PoissonDeformation.h"
#include "../Utility/MyAfxFunction.h"
#include <iostream>
using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CMainFrame

IMPLEMENT_DYNCREATE(CMainFrame, CFrameWndEx)

	BEGIN_MESSAGE_MAP(CMainFrame, CFrameWndEx)
		ON_WM_CREATE()
		ON_COMMAND_RANGE(ID_VIEW_APPLOOK_WIN_2000, ID_VIEW_APPLOOK_WINDOWS_7, &CMainFrame::OnApplicationLook)
		ON_UPDATE_COMMAND_UI_RANGE(ID_VIEW_APPLOOK_WIN_2000, ID_VIEW_APPLOOK_WINDOWS_7, &CMainFrame::OnUpdateApplicationLook)
		ON_COMMAND(ID_FILE_PRINT, &CMainFrame::OnFilePrint)
		ON_COMMAND(ID_FILE_PRINT_DIRECT, &CMainFrame::OnFilePrint)
		ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CMainFrame::OnFilePrintPreview)
		ON_UPDATE_COMMAND_UI(ID_FILE_PRINT_PREVIEW, &CMainFrame::OnUpdateFilePrintPreview)
		ON_COMMAND(ID_POISSON, &CMainFrame::OnPoissonDeformation)
		
	END_MESSAGE_MAP()

	// CMainFrame ����/����

	CMainFrame::CMainFrame():m_deformation(theApp.m_deformation)
	//	,mesh(theApp.m_triMesh.m_mesh)
		,mesh(*theApp.m_triMesh.GetMesh())
	{
		// TODO: �ڴ���ӳ�Ա��ʼ������
		theApp.m_nAppLook = theApp.GetInt(_T("ApplicationLook"), ID_VIEW_APPLOOK_VS_2008);
	}

	CMainFrame::~CMainFrame()
	{
	}

	int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
	{
		if (CFrameWndEx::OnCreate(lpCreateStruct) == -1)
			return -1;

		BOOL bNameValid;
		// ���ڳ־�ֵ�����Ӿ�����������ʽ
		OnApplicationLook(theApp.m_nAppLook);

		m_wndRibbonBar.Create(this);
		m_wndRibbonBar.LoadFromResource(IDR_RIBBON);

		if (!m_wndStatusBar.Create(this))
		{
			TRACE0("δ�ܴ���״̬��\n");
			return -1;      // δ�ܴ���
		}

		CString strTitlePane1;
		CString strTitlePane2;
		bNameValid = strTitlePane1.LoadString(IDS_STATUS_PANE1);
		ASSERT(bNameValid);
		bNameValid = strTitlePane2.LoadString(IDS_STATUS_PANE2);
		ASSERT(bNameValid);
		m_wndStatusBar.AddElement(new CMFCRibbonStatusBarPane(ID_STATUSBAR_PANE1, strTitlePane1, TRUE), strTitlePane1);
		m_wndStatusBar.AddExtendedElement(new CMFCRibbonStatusBarPane(ID_STATUSBAR_PANE2, strTitlePane2, TRUE), strTitlePane2);

		if (!m_pane.Create(_T("���ο������"), this, CRect(0, 0, 200, 200), TRUE, 1001, WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | CBRS_LEFT | CBRS_FLOAT_MULTI))
		{
			TRACE0("Failed to create LeftPane\n");
			return -1;
		}

		m_pane.EnableDocking(CBRS_ALIGN_ANY);
		EnableDocking(CBRS_ALIGN_ANY);
		DockPane(&m_pane);
		m_pane.ShowPane(FALSE,FALSE,TRUE);

		// ���� Visual Studio 2005 ��ʽͣ��������Ϊ
		CDockingManager::SetDockingMode(DT_SMART);
		// ���� Visual Studio 2005 ��ʽͣ�������Զ�������Ϊ
		EnableAutoHidePanes(CBRS_ALIGN_ANY);

		//MFC�Զ����书��
		EnableLoadDockState(FALSE);
		return 0;
	}

	BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
	{
		if( !CFrameWndEx::PreCreateWindow(cs) )
			return FALSE;
		// TODO: �ڴ˴�ͨ���޸�
		//  CREATESTRUCT cs ���޸Ĵ��������ʽ

		return TRUE;
	}

	// CMainFrame ���

#ifdef _DEBUG
	void CMainFrame::AssertValid() const
	{
		CFrameWndEx::AssertValid();
	}

	void CMainFrame::Dump(CDumpContext& dc) const
	{
		CFrameWndEx::Dump(dc);
	}
#endif //_DEBUG


	// CMainFrame ��Ϣ�������

	void CMainFrame::OnApplicationLook(UINT id)
	{
		CWaitCursor wait;

		theApp.m_nAppLook = id;

		switch (theApp.m_nAppLook)
		{
		case ID_VIEW_APPLOOK_WIN_2000:
			CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManager));
			m_wndRibbonBar.SetWindows7Look(FALSE);
			break;

		case ID_VIEW_APPLOOK_OFF_XP:
			CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerOfficeXP));
			m_wndRibbonBar.SetWindows7Look(FALSE);
			break;

		case ID_VIEW_APPLOOK_WIN_XP:
			CMFCVisualManagerWindows::m_b3DTabsXPTheme = TRUE;
			CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerWindows));
			m_wndRibbonBar.SetWindows7Look(FALSE);
			break;

		case ID_VIEW_APPLOOK_OFF_2003:
			CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerOffice2003));
			CDockingManager::SetDockingMode(DT_SMART);
			m_wndRibbonBar.SetWindows7Look(FALSE);
			break;

		case ID_VIEW_APPLOOK_VS_2005:
			CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerVS2005));
			CDockingManager::SetDockingMode(DT_SMART);
			m_wndRibbonBar.SetWindows7Look(FALSE);
			break;

		case ID_VIEW_APPLOOK_VS_2008:
			CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerVS2008));
			CDockingManager::SetDockingMode(DT_SMART);
			m_wndRibbonBar.SetWindows7Look(FALSE);
			break;

		case ID_VIEW_APPLOOK_WINDOWS_7:
			CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerWindows7));
			CDockingManager::SetDockingMode(DT_SMART);
			m_wndRibbonBar.SetWindows7Look(TRUE);
			break;

		default:
			switch (theApp.m_nAppLook)
			{
			case ID_VIEW_APPLOOK_OFF_2007_BLUE:
				CMFCVisualManagerOffice2007::SetStyle(CMFCVisualManagerOffice2007::Office2007_LunaBlue);
				break;

			case ID_VIEW_APPLOOK_OFF_2007_BLACK:
				CMFCVisualManagerOffice2007::SetStyle(CMFCVisualManagerOffice2007::Office2007_ObsidianBlack);
				break;

			case ID_VIEW_APPLOOK_OFF_2007_SILVER:
				CMFCVisualManagerOffice2007::SetStyle(CMFCVisualManagerOffice2007::Office2007_Silver);
				break;

			case ID_VIEW_APPLOOK_OFF_2007_AQUA:
				CMFCVisualManagerOffice2007::SetStyle(CMFCVisualManagerOffice2007::Office2007_Aqua);
				break;
			}

			CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerOffice2007));
			CDockingManager::SetDockingMode(DT_SMART);
			m_wndRibbonBar.SetWindows7Look(FALSE);
		}

		RedrawWindow(NULL, NULL, RDW_ALLCHILDREN | RDW_INVALIDATE | RDW_UPDATENOW | RDW_FRAME | RDW_ERASE);

		theApp.WriteInt(_T("ApplicationLook"), theApp.m_nAppLook);
	}

	void CMainFrame::OnUpdateApplicationLook(CCmdUI* pCmdUI)
	{
		pCmdUI->SetRadio(theApp.m_nAppLook == pCmdUI->m_nID);
	}

	void CMainFrame::OnFilePrint()
	{
		if (IsPrintPreview())
		{
			PostMessage(WM_COMMAND, AFX_ID_PREVIEW_PRINT);
		}
	}

	void CMainFrame::OnFilePrintPreview()
	{
		if (IsPrintPreview())
		{
			PostMessage(WM_COMMAND, AFX_ID_PREVIEW_CLOSE);  // ǿ�ƹرա���ӡԤ����ģʽ
		}
	}

	void CMainFrame::OnUpdateFilePrintPreview(CCmdUI* pCmdUI)
	{
		pCmdUI->SetCheck(IsPrintPreview());
	}

	CMeshDeformationDoc*  CMainFrame::GetDocument()
	{
		POSITION posDT = theApp.GetFirstDocTemplatePosition();
		CDocTemplate *pDocTemplate = theApp.GetNextDocTemplate( posDT );
		POSITION posDoc = pDocTemplate->GetFirstDocPosition();
		CMeshDeformationDoc *pDocument = (CMeshDeformationDoc*) pDocTemplate->GetNextDoc( posDoc );
		return pDocument;
	}

	//���ɱ���
	void CMainFrame::OnPoissonDeformation()
	{
		if(!mesh.vertices_empty())
		{
			m_pane.ShowPane(TRUE,FALSE,TRUE);
			m_pane.LoadDialog(IDD_POISSON);
			m_pane.OnSize(0,230,800);

			if(m_deformation) delete m_deformation;
			m_deformation = new PoissonDeformation(&mesh);
		}
		else
			MessageBox("no mesh model!");
	}
	 