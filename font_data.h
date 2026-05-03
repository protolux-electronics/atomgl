// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: Linux kernel developers et al.
// See also lib/fonts/font_8x16.c
//
// Style Exception (AVMCCS-N001/N005/N008): the identifiers FONTDATAMAX
// and fontdata are kept verbatim from the Linux kernel font_8x16 source
// to simplify comparison with upstream. Do not rename them.

#ifndef _FONT_DATA_H_
#define _FONT_DATA_H_

#define FONTDATAMAX 4096
#define CHAR_WIDTH 8

extern const unsigned char fontdata[FONTDATAMAX];

#endif
