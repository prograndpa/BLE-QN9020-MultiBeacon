/**
 ****************************************************************************************
 *
 * @file compiler.h
 *
 * @brief Definitions of compiler specific directives.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 6863 $
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 *
 * @file compiler.h
 *
 * @brief NXP revised and cut.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _COMPILER_H_
#define _COMPILER_H_

/**
 * @addtogroup MAIN
 * @{
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include <assert.h>
#include "intc.h"
#include "usr_config.h"

/// define the force inlining attribute for this compiler
#undef __INLINE
#define __INLINE  __STATIC_INLINE

/*
 * ASSERTION CHECK
 ****************************************************************************************
 */

#ifdef CFG_DBG_PRINT

extern void assert_err(const char *condition, const char * file, int line);
extern void assert_param(int param0, int param1, const char * file, int line);
extern void assert_warn(const char *condition, const char * file, int line);
    
/// Assertions showing a critical error that could require a full system reset
#define ASSERT_ERR(cond) ((cond) || ((assert_err(#cond, __FILE__, __LINE__)),0))

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1) ((cond) || ((assert_param(param0, param1, __FILE__, __LINE__)),0))

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond) ((cond) || ((assert_warn(#cond, __FILE__, __LINE__)),0))

#else

#define NDEBUG
#define ASSERT_ERR(cond)
#define ASSERT_INFO(cond, param0, param1)
#define ASSERT_WARN(cond)

#endif

/// @} MAIN

#endif // _COMPILER_H_
