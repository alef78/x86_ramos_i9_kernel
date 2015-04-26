/*COPYRIGHT**
 * -------------------------------------------------------------------------
 *               INTEL CORPORATION PROPRIETARY INFORMATION
 *  This software is supplied under the terms of the accompanying license
 *  agreement or nondisclosure agreement with Intel Corporation and may not
 *  be copied or disclosed except in accordance with the terms of that
 *  agreement.
 *        Copyright (c) 2010-2012 Intel Corporation.  All Rights Reserved.
 * -------------------------------------------------------------------------
**COPYRIGHT*/
/*
 *  File  : lwpmudrv_version.h
 */

#ifndef _LWPMUDRV_VERSION_H_
#define _LWPMUDRV_VERSION_H_

// SEP VERSIONING - Make sure to update basic001 DQ to match if version numbers here are changed, otherwise the test will fail.

#define _STRINGIFY(x)     #x
#define STRINGIFY(x)      _STRINGIFY(x)
#define _STRINGIFY_W(x)   L#x
#define STRINGIFY_W(x)    _STRINGIFY_W(x)

#define SEP_MAJOR_VERSION 3
#define SEP_MINOR_VERSION 10
#define SEP_API_VERSION   3

#define EMON_MAJOR_VERSION          9
#define EMON_MINOR_VERSION          0
#define EMON_PRODUCT_RELEASE_STRING 0

#ifdef EMON_INTERNAL
#define EMON_PRODUCT_TYPE "Private"
#else
#define EMON_PRODUCT_TYPE "Public"
#endif

#ifndef PRODUCT_BUILDER
#define PRODUCT_BUILDER "(unknown)"
#define SEP_PRODUCT_TYPE "(public)"
#else
#define SEP_PRODUCT_TYPE "(private)"
#endif

#define SEP_NAME          "sep"
#define SEP_NAME_W        L"sep"

#define SEP_PRODUCT_NAME  "Sampling Enabling Product"
#define EMON_PRODUCT_NAME "EMON"

#define PRODUCT_VERSION_DATE    __DATE__ " at " __TIME__

#define PRODUCT_COPYRIGHT   "Copyright (C) 1993-2013 Intel Corporation. All rights reserved."
#define PRODUCT_DISCLAIMER  "Warning: This computer program is protected under U.S. and international\ncopyright laws, and may only be used or copied in accordance with the terms\nof the license agreement.  Except as permitted by such license, no part\nof this computer program may be reproduced, stored in a retrieval system,\nor transmitted in any form or by any means without the express written consent\nof Intel Corporation."

#define SEP_MSG_PREFIX    SEP_NAME""STRINGIFY(SEP_MAJOR_VERSION)"_"STRINGIFY(SEP_MINOR_VERSION)":"
#define SEP_VERSION_STR   STRINGIFY(SEP_MAJOR_VERSION)"."STRINGIFY(SEP_MINOR_VERSION)"."STRINGIFY(SEP_API_VERSION)

#if defined(DRV_OS_WINDOWS)

#define SEP_DRIVER_NAME   SEP_NAME"drv"STRINGIFY(SEP_MAJOR_VERSION)"_"STRINGIFY(SEP_MINOR_VERSION)
#define SEP_DRIVER_NAME_W SEP_NAME_W L"drv" STRINGIFY_W(SEP_MAJOR_VERSION) L"_" STRINGIFY_W(SEP_MINOR_VERSION)
#define SEP_DEVICE_NAME   SEP_DRIVER_NAME

#endif

#if defined(DRV_OS_LINUX) || defined(DRV_OS_SOLARIS) || defined(DRV_OS_ANDROID) || defined(DRV_OS_FREEBSD)

#define SEP_DRIVER_NAME   SEP_NAME""STRINGIFY(SEP_MAJOR_VERSION)"_"STRINGIFY(SEP_MINOR_VERSION)
#define SEP_SAMPLES_NAME  SEP_DRIVER_NAME"_s"
#define SEP_DEVICE_NAME   "/dev/"SEP_DRIVER_NAME

#endif

#if defined(DRV_OS_MAC)

#define SEP_DRIVER_NAME   SEP_NAME""STRINGIFY(SEP_MAJOR_VERSION)"_"STRINGIFY(SEP_MINOR_VERSION)
#define SEP_SAMPLES_NAME  SEP_DRIVER_NAME"_s"
#define SEP_DEVICE_NAME   SEP_DRIVER_NAME

#endif

#if defined(EMON_INTERNAL)
#define SEP_DRIVER_MODE " (EMON INTERNAL)"
#elif defined(EMON)
#define SEP_DRIVER_MODE " (EMON)"
#else
#define SEP_DRIVER_MODE ""
#endif

#endif

