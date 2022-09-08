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

// MeshDeformationDoc.cpp : CMeshDeformationDoc ���ʵ��
//

#include "stdafx.h"
// SHARED_HANDLERS ������ʵ��Ԥ��������ͼ������ɸѡ�������
// ATL ��Ŀ�н��ж��壬�����������Ŀ�����ĵ����롣
#ifndef SHARED_HANDLERS
#include "MeshDeformation.h"
#endif

#include "MeshDeformationDoc.h"
#include "UI/MeshDeformationView.h"
#include "Utility/MyAfxFunction.h"
#include "fillingHole/PoissonDeform.h"
#include <propkey.h>

#include "fillingHole/FillHole.h"
#include "fillingHole/MeshSpike.h"
#include "fillingHole/MeshDegeneratedTriangles.h"
#include "fillingHole/MeshIsland.h"
#include "fillingHole/PoissonDeform.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CMeshDeformationDoc

IMPLEMENT_DYNCREATE(CMeshDeformationDoc, CDocument)

BEGIN_MESSAGE_MAP(CMeshDeformationDoc, CDocument)
	ON_COMMAND(ID_FILE_OPEN, &CMeshDeformationDoc::OnDocMeshOpen)
	ON_COMMAND(ID_FILE_OPEN_ALL, &CMeshDeformationDoc::OnDecMeshOpenAll)
	ON_COMMAND(ID_FILE_SAVE, &CMeshDeformationDoc::OnDocMeshSave)
	ON_COMMAND(ID_FILL_ALL_HOLE, &CMeshDeformationDoc::OnMeshFillAllHole)
	ON_COMMAND(ID_FILL_WITHOUT_0TH_HOLE, &CMeshDeformationDoc::OnMeshFillWithout0thHole)
	ON_COMMAND(ID_FILL_HOLE_SELECT, &CMeshDeformationDoc::OnMeshFillHoleSelect)
	ON_COMMAND(ID_MESH_SPIKE, &CMeshDeformationDoc::OnMeshSpike)
	ON_COMMAND(ID_MESH_DEGENERATED_TRIANGLES, &CMeshDeformationDoc::OnMeshDegeneratedTriangles)
	ON_COMMAND(ID_MESH_ISLAND, &CMeshDeformationDoc::OnMeshIsland)
	ON_COMMAND(ID_HOLE_DEFORMATION, &CMeshDeformationDoc::OnMeshDeformation)
END_MESSAGE_MAP()

FillHole fh(theApp.m_triMesh.m_mesh);

// CMeshDeformationDoc ����/����

CMeshDeformationDoc::CMeshDeformationDoc()
{
	// TODO: �ڴ����һ���Թ������

}

CMeshDeformationDoc::~CMeshDeformationDoc()
{
}

BOOL CMeshDeformationDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: �ڴ�������³�ʼ������
	// (SDI �ĵ������ø��ĵ�)

	return TRUE;
}




// CMeshDeformationDoc ���л�

void CMeshDeformationDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: �ڴ���Ӵ洢����
	}
	else
	{
		// TODO: �ڴ���Ӽ��ش���
	}
}

#ifdef SHARED_HANDLERS

void CMeshDeformationDoc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT)GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

void CMeshDeformationDoc::InitializeSearchContent()
{
	CString strSearchContent;
	SetSearchContent(strSearchContent);
}

void CMeshDeformationDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl* pChunk = NULL;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != NULL)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CMeshDeformationDoc ���

#ifdef _DEBUG
void CMeshDeformationDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CMeshDeformationDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG



void CMeshDeformationDoc::OnDocMeshOpen()
{
	theApp.m_triMesh.LoadMesh();
	theApp.resetDeformation();

	fh.set(theApp.m_triMesh.m_mesh);

	fh.findAllHoles();

	fh.showMeshHoleInfo();

	afxGetMainView()->updateView();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());
}

void CMeshDeformationDoc::OnDocMeshSave()
{
	CString m_filePath;
	CFileDialog file(false);
	file.m_ofn.lpstrFilter = _T("mesh file(*.obj)\0*.obj\0Off File(*.off)\0*.off\0All File(*.*)\0*.*\0\0");
	file.m_ofn.lpstrTitle = _T("SAVE");
	if (file.DoModal() == IDOK)
	{
		m_filePath = file.GetPathName();

		string path = m_filePath;

		cout << path;

		OpenMesh::IO::write_mesh(theApp.m_triMesh.m_mesh, path + ".obj");
	}
}

void CMeshDeformationDoc::OnDecMeshOpenAll()
{
	theApp.m_triMesh.LoadMeshAll();
	theApp.resetDeformation();

	//afxGetMainView()->updateView();
	afxGetMainView()->Invalidate();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());
}

void CMeshDeformationDoc::OnMeshFillAllHole()
{
	fh.fillAllHoles();

	theApp.m_triMesh.m_mesh = fh.getMesh();
	theApp.resetDeformation();

	//fh.save("test_fillallhole");

	//afxGetMainView()->updateView();
	afxGetMainView()->Invalidate();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());
	afxGetMainMesh()->Draw();

	theApp.m_triMesh.UpdateDisList();
}

void CMeshDeformationDoc::OnMeshFillWithout0thHole()
{
	fh.fillAllHoles(false);

	theApp.m_triMesh.m_mesh = fh.getMesh();
	theApp.resetDeformation();

	//fh.save("test_fillwithout0thhole");

	//afxGetMainView()->updateView();
	afxGetMainView()->Invalidate();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());
	afxGetMainMesh()->Draw();

	theApp.m_triMesh.UpdateDisList();
}

void CMeshDeformationDoc::OnMeshFillHoleSelect()
{
	int input;

	cout << "Select Hole idx : ";

	cin >> input;

	fh.fillHole(input);

	theApp.m_triMesh.m_mesh = fh.getMesh();
	theApp.resetDeformation();

	//fh.save("test_fillselect_" + to_string(input) + "Hole");

	//afxGetMainView()->updateView();
	afxGetMainView()->Invalidate();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());

	theApp.m_triMesh.UpdateDisList();
}

void CMeshDeformationDoc::OnMeshSpike()
{
	MeshSpike ms(theApp.m_triMesh.m_mesh, 35);

	ms.printInfo();

	if (ms.meshHasSpike())
	{
		int input = 0;
		int option;

		while (input != -1)
		{
			cout << "===== Select Spike Options =====\n";
			cout << "-1 : Cancel\n";
			cout << "0 : Show Info\n";
			cout << "1 : Delete All Spike\n";
			cout << "2 : Delete Spike (index Select)\n";
	
			cin >> input;

			switch (input)
			{
			case -1 :
				theApp.m_triMesh.m_mesh = ms.returnMesh();
				break;
			case 0 :
				ms.printInfo();
				break;
			case 1:

				cout << "Select Delete Spike Option\n";
				cout << "1 : Simple Delete (make Hole)\n";
				cout << "2 : Smoothing N-th Hole\n";
				cout << "3 : Simple Smoothing\n";

				cin >> option;

				ms.deleteAllSpikes(option);
				break;
			case 2:
				int index;

				cout << "input target index : ";
				cin >> index;

				cout << "input delete option(1 : Simple Delete, 2 : Smoothing N-th Hole, 3 : Simple Smoothing) : ";
				cin >> option;

				ms.deleteSpike(index, option);
				break;
			default:
				break;
			}
		}
	}
	//afxGetMainView()->updateView();
	afxGetMainView()->Invalidate();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());
	theApp.m_triMesh.UpdateDisList();
}

void CMeshDeformationDoc::OnMeshDegeneratedTriangles()
{
	MeshDegeneratedTriangles dt(theApp.m_triMesh.m_mesh, 10, 160, 1);

	dt.printDTInfo();

	if (dt.isMeshHasDT())
	{
		int input = 0;

		while (input != -1)
		{
			cout << "===== Select Degenerated Triangles Options =====\n";
			cout << "-1 : Cancel\n";
			cout << "0 : Show Info\n";
			cout << "1 : Merge All DT\n";
			cout << "2 : Merge DT (index Select)\n";

			cin >> input;

			switch (input)
			{
			case -1:
				theApp.m_triMesh.m_mesh = dt.returnMesh();
				break;
			case 0:
				dt.printDTInfo();
				break;
			case 1:
				dt.mergeAllDT();
				break;
			case 2:
				int index;

				cout << "input target index : ";
				cin >> index;

				dt.mergeDT(index);
				break;
			default:
				break;
			}
		}
	}
	//afxGetMainView()->updateView();
	afxGetMainView()->Invalidate();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());
	theApp.m_triMesh.UpdateDisList();

}

void CMeshDeformationDoc::OnMeshIsland()
{
	MeshIsland mi(theApp.m_triMesh.m_mesh);

	mi.printAllInfo();

	if (mi.isMeshHasIsland())
	{
		int input = 0;

		while (input != -1)
		{
			cout << "===== Select Island Options =====\n";
			cout << "-1 : Cancel\n";
			cout << "0 : Show Info\n";
			cout << "1 : Delete All Island\n";
			cout << "2 : Delete Island (index Select)\n";

			cin >> input;

			switch (input)
			{
			case -1:
				theApp.m_triMesh.m_mesh = mi.returnMesh();
				break;
			case 0:
				mi.printAllInfo();
				break;
			case 1:
				mi.eraseAllIsland();
				break;
			case 2:
				int index;

				cout << "input target index : ";
				cin >> index;

				mi.eraseIsland(index);
				break;
			default:
				break;
			}
		}
	}	

	//afxGetMainView()->updateView();
	afxGetMainView()->Invalidate();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());
	theApp.m_triMesh.UpdateDisList();
}

void CMeshDeformationDoc::OnMeshDeformation()
{
	PoissonDeform PD(theApp.m_triMesh.m_mesh, fh.m_Holes);

	PD.deform();

	theApp.m_triMesh.m_mesh = PD.getMesh();
	theApp.resetDeformation();

	//fh.save("test_MeshDeformation_Hole");

	//afxGetMainView()->updateView();
	afxGetMainView()->Invalidate();
	afxGetPickMesh()->SetDesMesh(theApp.m_triMesh.GetMesh());

	theApp.m_triMesh.UpdateDisList();
}