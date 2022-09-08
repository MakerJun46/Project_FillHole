#pragma once
#include "PoissonDefDlg.h"
#include "../MeshDeform/Deformation.h"
#include "../MeshDeform/PoissonDeform/PoissonDeformation.h"
// PossionDefDlg �Ի���

class PoissonDefDlg : public CDialog
{
	DECLARE_DYNAMIC(PoissonDefDlg)

public:
	PoissonDefDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~PoissonDefDlg();

// �Ի�������
	enum { IDD = IDD_POISSON };

public:
	PoissonDeformation* m_poisson_def;
	Deformation*& m_deformation;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnImportBC();
	afx_msg void OnSetFixedRegion();
	afx_msg void OnComputeLaplacianMatrix();
	afx_msg void OnPoissonDeformation();
	afx_msg void OnReset();
};
 