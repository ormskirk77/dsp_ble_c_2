/*
 * File:           E:\ALL_INPUTS\0_1\0_1_IC_1_PARAM.h
 *
 * Created:        Thursday, September 19, 2019 3:54:22 PM
 * Description:    :IC 1 parameter RAM definitions.
 *
 * This software is distributed in the hope that it will be useful,
 * but is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This software may only be used to program products purchased from
 * Analog Devices for incorporation by you into audio products that
 * are intended for resale to audio product end users. This software
 * may not be distributed whole or in any part to third parties.
 *
 * Copyright ©2019 Analog Devices, Inc. All rights reserved.
 */
#ifndef __0_1_IC_1_PARAM_H__
#define __0_1_IC_1_PARAM_H__


/* Module NxM Mixer1 - NxM Mixer*/
#define MOD_NXMMIXER1_COUNT                            4
#define MOD_NXMMIXER1_DEVICE                           "IC1"
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10000_ADDR   0
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10000_FIXPT  0x00800000
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10000_VALUE  SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10000_TYPE   SIGMASTUDIOTYPE_FIXPOINT
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10001_ADDR   1
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10001_FIXPT  0x00800000
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10001_VALUE  SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10001_TYPE   SIGMASTUDIOTYPE_FIXPOINT
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10100_ADDR   2
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10100_FIXPT  0x00800000
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10100_VALUE  SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10100_TYPE   SIGMASTUDIOTYPE_FIXPOINT
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10101_ADDR   3
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10101_FIXPT  0x00800000
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10101_VALUE  SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_NXMMIXER1_ALG0_NXNMIXER1940ALG10101_TYPE   SIGMASTUDIOTYPE_FIXPOINT

/* Module Gen Filter1 - General (2nd order)*/
#define MOD_GENFILTER1_COUNT                           5
#define MOD_GENFILTER1_DEVICE                          "IC1"
#define MOD_GENFILTER1_ALG0_STAGE0_B0_ADDR             4
#define MOD_GENFILTER1_ALG0_STAGE0_B0_FIXPT            0x00800000
#define MOD_GENFILTER1_ALG0_STAGE0_B0_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_GENFILTER1_ALG0_STAGE0_B0_TYPE             SIGMASTUDIOTYPE_FIXPOINT
#define MOD_GENFILTER1_ALG0_STAGE0_B1_ADDR             5
#define MOD_GENFILTER1_ALG0_STAGE0_B1_FIXPT            0x00000000
#define MOD_GENFILTER1_ALG0_STAGE0_B1_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0)
#define MOD_GENFILTER1_ALG0_STAGE0_B1_TYPE             SIGMASTUDIOTYPE_FIXPOINT
#define MOD_GENFILTER1_ALG0_STAGE0_B2_ADDR             6
#define MOD_GENFILTER1_ALG0_STAGE0_B2_FIXPT            0x00000000
#define MOD_GENFILTER1_ALG0_STAGE0_B2_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0)
#define MOD_GENFILTER1_ALG0_STAGE0_B2_TYPE             SIGMASTUDIOTYPE_FIXPOINT
#define MOD_GENFILTER1_ALG0_STAGE0_A1_ADDR             7
#define MOD_GENFILTER1_ALG0_STAGE0_A1_FIXPT            0x00000000
#define MOD_GENFILTER1_ALG0_STAGE0_A1_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0)
#define MOD_GENFILTER1_ALG0_STAGE0_A1_TYPE             SIGMASTUDIOTYPE_FIXPOINT
#define MOD_GENFILTER1_ALG0_STAGE0_A2_ADDR             8
#define MOD_GENFILTER1_ALG0_STAGE0_A2_FIXPT            0x00000000
#define MOD_GENFILTER1_ALG0_STAGE0_A2_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0)
#define MOD_GENFILTER1_ALG0_STAGE0_A2_TYPE             SIGMASTUDIOTYPE_FIXPOINT

/* Module SW vol 1 - Single SW slew vol (adjustable)*/
#define MOD_SWVOL1_COUNT                               2
#define MOD_SWVOL1_DEVICE                              "IC1"
#define MOD_SWVOL1_ALG0_TARGET_ADDR                    9
#define MOD_SWVOL1_ALG0_TARGET_FIXPT                   0x00800000
#define MOD_SWVOL1_ALG0_TARGET_VALUE                   SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_SWVOL1_ALG0_TARGET_TYPE                    SIGMASTUDIOTYPE_FIXPOINT
#define MOD_SWVOL1_ALG0_STEP_ADDR                      10
#define MOD_SWVOL1_ALG0_STEP_FIXPT                     0x00000800
#define MOD_SWVOL1_ALG0_STEP_VALUE                     SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.000244140625)
#define MOD_SWVOL1_ALG0_STEP_TYPE                      SIGMASTUDIOTYPE_FIXPOINT

#endif
