// 这段 MFC 示例源代码演示如何使用 MFC Microsoft Office Fluent 用户界面 
// (“Fluent UI”)。该示例仅供参考，
// 用以补充《Microsoft 基础类参考》和 
// MFC C++ 库软件随附的相关电子文档。
// 复制、使用或分发 Fluent UI 的许可条款是单独提供的。
// 若要了解有关 Fluent UI 许可计划的详细信息，请访问  
// http://msdn.microsoft.com/officeui。
//
// 版权所有(C) Microsoft Corporation
// 保留所有权利。

// MeshDeformationDoc.h : CMeshDeformationDoc 类的接口
//


#pragma once
#include "Utility/baseHeadFile.h"
#include "fillingHole/FillHole.h"

class CMeshDeformationDoc : public CDocument
{
protected:
	CMeshDeformationDoc();
	DECLARE_DYNCREATE(CMeshDeformationDoc)

public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// 实现
public:
	virtual ~CMeshDeformationDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// 用于为搜索处理程序设置搜索内容的 Helper 函数
	void SetSearchContent(const CString& value);
#endif // SHARED_HANDLERS
public:
	afx_msg void OnDocMeshOpen();
	afx_msg void OnDecMeshOpenAll();
	afx_msg void OnDocMeshSave();
	afx_msg void OnMeshFillAllHole();
	afx_msg void OnMeshFillWithout0thHole();
	afx_msg void OnMeshFillHoleSelect();
	afx_msg void OnMeshSpike();
	afx_msg void OnMeshDegeneratedTriangles();
	afx_msg void OnMeshIsland();
	afx_msg void OnMeshDeformation();
};
 