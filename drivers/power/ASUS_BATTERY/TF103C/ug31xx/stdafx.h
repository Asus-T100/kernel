// stdafx.h : Include the standard system Include files
// or include files without frequently modified

#pragma once

#ifndef BUILD_UG31XX_LIB

#ifndef	uG31xx_OS_ANDROID

#ifndef CONFIG_ASUS_UG31XX_BATTERY

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // �q Windows ���Y�ư����`�ϥΪ�����
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // ���T�w�q������ CString �غc�禡

#include <afxwin.h>         // MFC �֤߻P�зǤ���
#include <afxext.h>         // MFC �X�R�\��

#ifndef _AFX_NO_OLE_SUPPORT
#include <afxole.h>         // MFC OLE ���O
#include <afxodlgs.h>       // MFC OLE ��ܤ�����O
#include <afxdisp.h>        // MFC Automation ���O
#endif // _AFX_NO_OLE_SUPPORT

#ifndef _AFX_NO_DB_SUPPORT
#include <afxdb.h>                      // MFC ODBC ��Ʈw���O
#endif // _AFX_NO_DB_SUPPORT

#ifndef _AFX_NO_DAO_SUPPORT
#include <afxdao.h>                     // MFC DAO ��Ʈw���O
#endif // _AFX_NO_DAO_SUPPORT

#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC �䴩�� Internet Explorer 4 �q�α��
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>                     // MFC �䴩�� Windows �q�α��
#endif // _AFX_NO_AFXCMN_SUPPORT

#endif  ///< end of CONFIG_ASUS_UG31XX_BATTERY

#endif	///< end of uG31xx_OS_ANDROID

#endif  ///< end of BUILD_UG31XX_LIB

