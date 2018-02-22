/*
 * CONFIDENTIAL
 *
 * Copyright (c) 2013 Yamaha Corporation
 */
#ifndef __YAS53X_H__
#define __YAS53X_H__

#define YAS_VERSION	"1.0.2"

#if defined(__KERNEL__)
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#ifndef NULL
#define NULL ((void *)(0))
#endif

#define YAS_NO_ERROR			(0)
#define YAS_ERROR_DEVICE_COMMUNICATION	(-1)
#define YAS_ERROR_I2C			YAS_ERROR_DEVICE_COMMUNICATION
#define YAS_ERROR_POWER			(-2)
#define YAS_ERROR_TEST_ORDER		(-3)
#define YAS_ERROR_NOT_INITIALIZED	YAS_ERROR_TEST_ORDER
#define YAS_ERROR_INTERRUPT		(-4)
#define YAS_ERROR_BUSY			(-5)
#define YAS_ERROR_OVERFLOW		(-6)
#define YAS_ERROR_UNDERFLOW		(-7)
#define YAS_ERROR_DIRCALC		(-8)
#define YAS_ERROR_NOT_SUPPORTED		(-9)
#define YAS_ERROR_CALREG		(-10)
#define YAS_ERROR_CHIP_ID		(-11)
#define YAS_ERROR_ARG			(-128)

#define NELEMS(a) ((int)(sizeof(a)/sizeof(a[0])))

struct yas_vector {
	int32_t v[3];
};

struct yas_matrix {
	int32_t m[9];
};

struct yas_quaternion {
	int32_t q[4];
};

#endif
