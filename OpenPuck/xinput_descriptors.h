/*
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: Copyright (c) 2021 Jason Skuby (mytechtoybox.com)
 *
 * Configuration body adapted from GP2040-CE's XInputDescriptors.h.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#pragma once
#include <stdint.h>

static const uint8_t XINPUT_CONFIG_BODY[] = {
	9,    4,    0,	  0,	2,    0xFF, 0x5D, 0x01, 0,    17,   0x21,
	0,    1,    1,	  0x25, 0x81, 0x14, 0,	  0,	0,    0,    0x13,
	0x02, 0x08, 0,	  0,	7,    5,    0x81, 3,	0x20, 0,    1,
	7,    5,    0x02, 3,	0x20, 0,    8,

	9,    4,    1,	  0,	4,    0xFF, 0x5D, 0x03, 0,    27,   0x21,
	0,    1,    1,	  1,	0x83, 0x40, 1,	  0x04, 0x20, 0x16, 0x85,
	0,    0,    0,	  0,	0,    0,    0x16, 0x06, 0,    0,    0,
	0,    0,    0,	  7,	5,    0x83, 3,	  0x20, 0,    2,    7,
	5,    0x04, 3,	  0x20, 0,    4,    7,	  5,	0x85, 3,    0x20,
	0,    0x40, 7,	  5,	0x06, 3,    0x20, 0,	0x10,

	9,    4,    2,	  0,	1,    0xFF, 0x5D, 0x02, 0,    9,    0x21,
	0,    1,    1,	  0x22, 0x86, 3,    0,	  7,	5,    0x86, 3,
	0x20, 0,    0x10,

	9,    4,    3,	  0,	0,    0xFF, 0xFD, 0x13, 4,    6,    0x41,
	0,    1,    1,	  3,
};

static_assert(sizeof XINPUT_CONFIG_BODY == 144,
	      "Xbox 360 configuration body must be 144 bytes");
static const uint16_t XINPUT_SECURITY_STRING_OFFSET = 137;
static const char XINPUT_SECURITY_STRING[] =
	"Xbox Security Method 3, Version 1.00, \xc2\xa9 2005 Microsoft "
	"Corporation. All rights reserved.";