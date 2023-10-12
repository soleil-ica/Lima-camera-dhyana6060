/************************************************************************

*  Copyright (C) Xintu Photonics Co.,Ltd. 2012-2021. All rights reserved.

*  @file      TUFits.h

*  @brief     TUFits export functions header file

*  @version	  1.0.0.1

*  @author    FDY

*  @date      2018-07-09

************************************************************************/

#ifdef TUFITS_LIB_EXPORTS
#define TUFITS_LIB_API extern "C" __declspec(dllexport)
#else
#define TUFITS_LIB_API extern "C" __declspec(dllimport)
#endif

// Define the struct of fits header
typedef struct _tagFITS_HEADER
{
	//  The based information
	USHORT usWidth;         // [in] The image width
	USHORT usHeight;        // [in] The image height

	UCHAR  ucChannels;      // [in] The image data channels
	UCHAR  ucElemBytes;     // [in] The image data bytes per element

	//  The data
	PUCHAR pImgData;        // [in] Pointer to the image data

	DOUBLE dblExposure;     // [in] The exposure time
	DOUBLE dbTemperature;   // [in] The temperature

	PCHAR   pstrGain;       // [in] The gain mode
	PCHAR   pstrCamName;    // [in] The camera name
	PCHAR   pstrPath;       // [in] The path of save file 
}FITS_HEADER, *PFITS_HEADER;

TUFITS_LIB_API	bool TUFITS_SaveFile(FITS_HEADER fitsHeader);      /* Create FITS File */