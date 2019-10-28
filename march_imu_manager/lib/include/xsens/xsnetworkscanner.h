/*	WARNING: COPYRIGHT (C) 2016 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#ifndef XSNETWORKSCANNER_H
#define XSNETWORKSCANNER_H

#include "xdaconfig.h"

//lint -esym(534, RegCloseKey)

struct XsNetworkScanner;

#ifdef __cplusplus
extern "C" {
#endif

XDA_DLL_API void XsNetworkScanner_construct(struct XsNetworkScanner* thisPtr);
XDA_DLL_API void XsNetworkScanner_destruct(struct XsNetworkScanner* thisPtr);

#ifdef __cplusplus
} // extern "C"
	
/*!	\struct XsNetworkScanner
	\brief This class will take care of instantiating/deleting the NetworkScanner and will act as reference Counter
*/
struct XsNetworkScanner {
	
/*! \brief Constructor of the class
*/
	inline explicit XsNetworkScanner()
	{
		XsNetworkScanner_construct(this);
	}
	
/*! \brief Destructor
*/
	inline ~XsNetworkScanner()
	{
		XsNetworkScanner_destruct(this);
	}
};

#endif

typedef struct XsNetworkScanner XsNetworkScanner;

#endif // file guard
