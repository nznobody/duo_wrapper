//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This library is part of Dense3D SDK that allows the use of Dense3D devices in your own applications
//
// For updates and file downloads go to: http://duo3d.com/
//
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _DENSE3DMT_H
#define _DENSE3DMT_H

#include <stdint.h>
#include "DUOLib.h"

#ifdef WIN32
    #ifdef DENSE3DMT_DLL
        #define API_FUNCTION(type)	__declspec(dllexport) type __cdecl
    #else
        #define API_FUNCTION(type)	__declspec(dllimport) type __cdecl
    #endif
#else
    #define API_FUNCTION(type)	 __attribute__((visibility("default"))) type
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Dense3D C API
extern "C" {

// Dense3D instance
typedef void *Dense3DMTInstance;

typedef struct  
{
    float x;
    float y;
    float z;
}Dense3DDepth, *PDense3DDepth;

// Dense3D Frame
typedef struct
{
    PDUOFrame duoFrame;                 // pointer to DUOFrame
    float *disparityData;               // Dense3D disparity data
    PDense3DDepth depthData;            // Dense3D depth data
}Dense3DFrame, *PDense3DFrame;

// Dense3D parameters
enum Dense3DParameter
{
    DENSE3D_LICENSE,                    // Set only: (char*) license key
    DENSE3D_IMAGE_INFO,                 // Set/Get: (uint32_t, uint32_t, double) width, height, fps
    DENSE3D_PARAMS,                     // Set/Get: (Dense3DParams) dense3DParameters
};

typedef struct
{
    uint32_t scale;                     // [0, 3] - [No Scale, Scale X, Scale Y, Scale X&Y]
    uint32_t mode;                      // [0, 3] - [BM, SGBM, BM_HQ, SGBM_HQ]
    uint32_t preFilterCap;              // [1, 63]
    uint32_t numDisparities;            // [2, 16]
    uint32_t sadWindowSize;             // [2, 10]
    uint32_t uniqenessRatio;            // [1, 100]
    uint32_t speckleWindowSize;         // [0, 256]
    uint32_t speckleRange;              // [0, 32]
}Dense3DParams, *PDense3DParams;

// Dense3D error codes
enum Dense3DErrorCode
{
    DENSE3D_NO_ERROR,
    DENSE3D_ERROR_INVALID_DUO3D_INSTANCE,
    DENSE3D_ERROR_COULD_NOT_START_DUO,
    DENSE3D_INVALID_DENSE3D_INSTANCE,
    DENSE3D_ERROR_CREATING_DENSE3D_INSTANCE,
    DENSE3D_INVALID_LICENSE,
    DENSE3D_INVALID_PARAMETER,
    DENSE3D_INVALD_IMAGE_POINTER,
    DENSE3D_INVALD_DEPTH_DATA_POINTER,
    DENSE3D_INVALID_IMAGE_SIZE,
    DENSE3D_INVALD_PLY_FILE_NAME,
    DENSE3D_ERROR_EXPORTING_PLY_FILE
};

// Dense3D error code
API_FUNCTION(Dense3DErrorCode) Dense3DGetErrorCode();

// Dense3D library version
API_FUNCTION(char*) Dense3DGetLibVersion();

// Dense3D initialization
API_FUNCTION(bool) Dense3DOpen(Dense3DMTInstance *dense3D, DUOInstance duo);
API_FUNCTION(bool) Dense3DClose(Dense3DMTInstance dense3D);

// Dense3D frame callback function
// NOTE: This function is called in the context of the Dense3D thread.
//		 To prevent any dropped frames, this function must return as soon as possible.
typedef void (CALLBACK *Dense3DFrameCallback)(const PDense3DFrame pFrameData, void *pUserData);

// Dense3D capture control
API_FUNCTION(bool) Dense3DStart(Dense3DMTInstance dense3D, Dense3DFrameCallback frameCallback, void *pUserData);
API_FUNCTION(bool) Dense3DStop(Dense3DMTInstance dense3D);

// Dense3D data access
API_FUNCTION(bool) Dense3DSavePLY(Dense3DMTInstance dense3D, char *plyFile, const PDense3DFrame pFrameData);

// Dense3D parameters control
// Do not call these functions directly
// Use below defined macros
API_FUNCTION(bool) __Dense3DParamSet__(Dense3DMTInstance dense3D, Dense3DParameter param, ...);
API_FUNCTION(bool) __Dense3DParamGet__(Dense3DMTInstance dense3D, Dense3DParameter param, ...);

// Get Dense3D parameters
#define GetDense3DImageInfo(dense3D, w, h)          __Dense3DParamGet__(dense3D, DENSE3D_IMAGE_INFO, (uint32_t*)w, (uint32_t*)h, (double*)fps)
#define GetDense3Params(dense3D, val)               __Dense3DParamGet__(dense3D, DENSE3D_PARAMS, (Dense3DParams&)val)

// Set Dense3D parameters
#define SetDense3DLicense(dense3D, val)             __Dense3DParamSet__(dense3D, DENSE3D_LICENSE, (char*)val)
#define SetDense3DImageInfo(dense3D, w, h, fps)     __Dense3DParamSet__(dense3D, DENSE3D_IMAGE_INFO, (uint32_t)w, (uint32_t)h, (double)fps)
#define SetDense3Params(dense3D, val)               __Dense3DParamSet__(dense3D, DENSE3D_PARAMS, (Dense3DParams&)val)

} // extern "C"

#endif // _DENSE3DMT_H
