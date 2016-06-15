//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This library is part of Dense3D SDK that allows the use of Dense3D devices in your own applications
//
// For updates and file downloads go to: http://duo3d.com/
//
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _DENSE3D_H
#define _DENSE3D_H

#include <stdint.h>

#ifdef WIN32
    #ifdef DENSE3D_DLL
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
typedef void *Dense3DInstance;

typedef struct  
{
    float x;
    float y;
    float z;
}Dense3DDepth, *PDense3DDepth;

// Dense3D parameters
enum Dense3DParameter
{
    DENSE3D_LICENSE,                    // Set only: (char*) license key
    DENSE3D_IMAGE_SIZE,                 // Set/Get: (uint32_t, uint32_t) width, height
    DENSE3D_CALIBRATION,                // Set only: (void*) stereo
    DENSE3D_SCALE,                      // Set/Get: (uint32_t) [0, 3] - [No Scale, Scale X, Scale Y, Scale X&Y]
    DENSE3D_MODE,                       // Set/Get: (uint32_t) [0, 3] - [BM, SGBM, BM_HQ, SGBM_HQ]
    DENSE3D_PREFILTER_CAP,              // Set/Get: (uint32_t) [1, 63]
    DENSE3D_NUM_DISPARITIES,            // Set/Get: (uint32_t) [2, 16]
    DENSE3D_SAD_WINDOW_SIZE,            // Set/Get: (uint32_t) [2, 10]
    DENSE3D_UNIQUENESS_RATIO,           // Set/Get: (uint32_t) [1, 100]
    DENSE3D_SPECKLE_WINDOW_SIZE,        // Set/Get: (uint32_t) [0, 256]
    DENSE3D_SPECKLE_RANGE,              // Set/Get: (uint32_t) [0, 32]
};

// Dense3D error codes
enum Dense3DErrorCode
{
    DENSE3D_NO_ERROR,
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
API_FUNCTION(bool) Dense3DOpen(Dense3DInstance *dense3D);
API_FUNCTION(bool) Dense3DClose(Dense3DInstance dense3D);

// Dense3D processing
API_FUNCTION(bool) Dense3DGetDepth(Dense3DInstance dense3D, uint8_t *leftImage, uint8_t *rightImage,
                                   float *disparityData, PDense3DDepth depthData);
// Dense3D data access
API_FUNCTION(bool) Dense3DSavePLY(Dense3DInstance dense3D, char *plyFile);

// Dense3D parameters control
// Do not call these functions directly
// Use below defined macros
API_FUNCTION(bool) __Dense3DParamSet__(Dense3DInstance dense3D, Dense3DParameter param, ...);
API_FUNCTION(bool) __Dense3DParamGet__(Dense3DInstance dense3D, Dense3DParameter param, ...);

// Get Dense3D parameters
#define GetDense3DImageSize(dense3D, w, h)          __Dense3DParamGet__(dense3D, DENSE3D_IMAGE_SIZE, (uint32_t*)w, (uint32_t*)h)
#define GetDense3DScale(dense3D, val)               __Dense3DParamGet__(dense3D, DENSE3D_SCALE, (uint32_t*)val)
#define GetDense3DMode(dense3D, val)                __Dense3DParamGet__(dense3D, DENSE3D_MODE, (uint32_t*)val)
#define GetDense3DPreFilterCap(dense3D, val)        __Dense3DParamGet__(dense3D, DENSE3D_PREFILTER_CAP, (uint32_t*)val)
#define GetDense3DNumDisparities(dense3D, val)      __Dense3DParamGet__(dense3D, DENSE3D_NUM_DISPARITIES, (uint32_t*)val)
#define GetDense3DSADWindowSize(dense3D, val)       __Dense3DParamGet__(dense3D, DENSE3D_SAD_WINDOW_SIZE, (uint32_t*)val)
#define GetDense3DUniqunessRatio(dense3D, val)      __Dense3DParamGet__(dense3D, DENSE3D_UNIQUNESS_RATIO, (uint32_t*)val)
#define GetDense3DSpeckleWindowSize(dense3D, val)   __Dense3DParamGet__(dense3D, DENSE3D_SPECKLE_WINDOW_SIZE, (uint32_t*)val)
#define GetDense3DSpeckleRange(dense3D, val)        __Dense3DParamGet__(dense3D, DENSE3D_SPECKLE_RANGE, (uint32_t*)val)

// Set Dense3D parameters
#define SetDense3DLicense(dense3D, val)             __Dense3DParamSet__(dense3D, DENSE3D_LICENSE, (char*)val)
#define SetDense3DImageSize(dense3D, w, h)          __Dense3DParamSet__(dense3D, DENSE3D_IMAGE_SIZE, (uint32_t)w, (uint32_t)h)
#define SetDense3DCalibration(dense3D, stereo)      __Dense3DParamSet__(dense3D, DENSE3D_CALIBRATION, (void*)stereo)
#define SetDense3DScale(dense3D, val)               __Dense3DParamSet__(dense3D, DENSE3D_SCALE, (uint32_t)val)
#define SetDense3DMode(dense3D, val)                __Dense3DParamSet__(dense3D, DENSE3D_MODE, (uint32_t)val)
#define SetDense3DPreFilterCap(dense3D, val)        __Dense3DParamSet__(dense3D, DENSE3D_PREFILTER_CAP, (uint32_t)val)
#define SetDense3DNumDisparities(dense3D, val)      __Dense3DParamSet__(dense3D, DENSE3D_NUM_DISPARITIES, (uint32_t)val)
#define SetDense3DSADWindowSize(dense3D, val)       __Dense3DParamSet__(dense3D, DENSE3D_SAD_WINDOW_SIZE, (uint32_t)val)
#define SetDense3DUniquenessRatio(dense3D, val)     __Dense3DParamSet__(dense3D, DENSE3D_UNIQUENESS_RATIO, (uint32_t)val)
#define SetDense3DSpeckleWindowSize(dense3D, val)   __Dense3DParamSet__(dense3D, DENSE3D_SPECKLE_WINDOW_SIZE, (uint32_t)val)
#define SetDense3DSpeckleRange(dense3D, val)        __Dense3DParamSet__(dense3D, DENSE3D_SPECKLE_RANGE, (uint32_t)val)

} // extern "C"

#endif // _DENSE3D_H
