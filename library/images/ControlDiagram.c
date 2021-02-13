/* Copyright (c) 2021, Texas Instruments Incorporated
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

*  Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

*  Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

*  Neither the name of Texas Instruments Incorporated nor the names of
   its contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/


#include <ti/grlib/grlib.h>
#include <stdint.h>

static const uint8_t pixel_ControlDiagram4BPP_UNCOMP[] =
{
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xb2, 0x55, 0xcd, 0xcd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xed, 0xc5, 0x52, 0x2b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x25, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdc, 0x22, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xb2, 0x5d, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xe2, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x2e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd2, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x2d, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0x52, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x02, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x2b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xb0, 0x5d, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xe2, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0x25, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x2b, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xb0, 0x5f, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xe1, 0x11, 0x11, 0x11, 0x1d, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xe2, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xb5, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd1, 0x11, 0x11, 0x11, 0x1e, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x44, 0xa4, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x2b, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0x5d, 0xed, 0xef, 0xed, 0xef, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xe1, 0x11, 0x11, 0x11, 0x1d, 0xed, 0xef, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0x41, 0xf4, 0x4d, 0xed, 0xed, 0xef, 0xed, 0xed, 0xe2, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xb2, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd1, 0x11, 0x11, 0x11, 0x1e, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x14, 0x4e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x2b, 0xbb, 0xbb, 
0xbb, 0xbb, 0xb5, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xf1, 0x11, 0x11, 0x11, 0x1d, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xe4, 0xa4, 0xff, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0x5b, 0xbb, 0xbb, 
0xbb, 0xbb, 0x2e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xbb, 0xbb, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd1, 0x11, 0x11, 0x11, 0x1e, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd4, 0x4e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd2, 0xbb, 0xbb, 
0xbb, 0xbb, 0xcd, 0xef, 0xed, 0xed, 0xed, 0xbb, 0xbb, 0xbb, 0xbb, 0xef, 0xed, 0xed, 0xed, 0xef, 0x11, 0x11, 0x11, 0xef, 0xed, 0xed, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xe4, 0x41, 0xf4, 0x4d, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xbb, 0xbb, 
0xbb, 0xb2, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdf, 0x44, 0xa4, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x2b, 0xbb, 
0xbb, 0xbd, 0xef, 0xed, 0xef, 0xfd, 0xeb, 0xb0, 0x19, 0xbb, 0xbb, 0xbd, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xcb, 0xbb, 
0xbb, 0x2e, 0xde, 0xde, 0xde, 0xde, 0xbb, 0xb4, 0xa4, 0xbb, 0xbb, 0xbb, 0xde, 0xde, 0xde, 0xde, 0xd4, 0xa9, 0xa4, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xab, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd2, 0xbb, 
0xbb, 0x5d, 0xed, 0xed, 0xed, 0xed, 0xbb, 0xbb, 0x19, 0xbb, 0xbb, 0xbb, 0xed, 0xef, 0xed, 0xed, 0xe9, 0xff, 0xed, 0xed, 0xed, 0xef, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xeb, 0xbb, 0xbb, 0xba, 0x9a, 0xbb, 0xbb, 0xbd, 0xef, 0xed, 0xed, 0xed, 0xef, 0xe2, 0xbb, 
0xbb, 0xde, 0xde, 0xde, 0xde, 0xde, 0xbb, 0xbb, 0x14, 0xbb, 0xbb, 0xbb, 0xde, 0xde, 0xde, 0xde, 0xd4, 0xa9, 0x4f, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xbb, 0xa9, 0xaa, 0xab, 0xbb, 0xbb, 0x0e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xbb, 
0xb2, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xbb, 0xbb, 0x19, 0xbb, 0xbb, 0xbb, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xf1, 0x44, 0xfd, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xb0, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0x2b, 
0xb2, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0x14, 0xbb, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x44, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xde, 0x2b, 
0xb2, 0xed, 0xed, 0xef, 0xed, 0xed, 0xeb, 0xb4, 0xaa, 0x91, 0xbb, 0xbf, 0xed, 0xed, 0xed, 0xef, 0xf4, 0xf1, 0x44, 0xef, 0xed, 0xed, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xed, 0xed, 0xed, 0xed, 0xed, 0xed, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x9b, 0xbb, 0xbb, 0xbb, 0xbb, 0xb0, 0xef, 0xed, 0xed, 0xed, 0x5b, 
0xbc, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xbb, 0xbb, 0xbb, 0xbb, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd4, 0xa9, 0x4f, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xde, 0xde, 0xde, 0xde, 0x5b, 
0xbd, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xbb, 0xbb, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xed, 0xed, 0xfd, 0xeb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbd, 0xef, 0xfd, 0xef, 0xcb, 
0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xdb, 
0xbd, 0xef, 0xed, 0xef, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xed, 0xed, 0xed, 0xed, 0xef, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xef, 0xed, 0xed, 0xed, 0xef, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x9b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xed, 0xef, 0xed, 0xeb, 
0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0x44, 0xde, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xde, 0xde, 0xde, 0xdb, 
0xbd, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xf4, 0x44, 0xfd, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xef, 0xfd, 0xef, 0xcb, 
0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0x44, 0x14, 0xde, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xba, 0xbb, 0xde, 0xde, 0xde, 0xdb, 
0xb5, 0xed, 0xed, 0xef, 0xed, 0xef, 0xed, 0xeb, 0xbb, 0xbd, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xed, 0xe4, 0x4f, 0xf9, 0xed, 0xba, 0x9b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x9b, 0xbb, 0xbb, 0xbb, 0xbb, 0xba, 0x9b, 0xed, 0xed, 0xed, 0xcb, 
0xb5, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xd4, 0xaa, 0xa9, 0xae, 0xa9, 0xaa, 0xa9, 0xaa, 0xa9, 0xaa, 0xa9, 0xaa, 0xa9, 0xaa, 0xa9, 0xaa, 0xa9, 0xa9, 0xde, 0xde, 0xde, 0x2b, 
0xb2, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xed, 0xf4, 0xfd, 0xba, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xba, 0xab, 0xef, 0xfd, 0xef, 0x2b, 
0xb2, 0xde, 0xde, 0xde, 0xde, 0xde, 0xbb, 0x04, 0xa9, 0x40, 0xbb, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0x14, 0xde, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xba, 0xbb, 0xde, 0xde, 0xde, 0x2b, 
0xbb, 0xed, 0xed, 0xed, 0xed, 0xeb, 0xbb, 0x04, 0x11, 0x41, 0xbb, 0xbd, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xef, 0xed, 0xed, 0xed, 0xef, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x9b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xed, 0xef, 0xed, 0xbb, 
0xbb, 0x2e, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xbb, 0x41, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xde, 0xde, 0xd5, 0xbb, 
0xbb, 0x2d, 0xef, 0xfd, 0xef, 0xfb, 0xbb, 0xbb, 0xb4, 0x40, 0xbb, 0xbd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xeb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbd, 0xef, 0xfd, 0xe2, 0xbb, 
0xbb, 0xbe, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0x44, 0x0b, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbe, 0xde, 0xde, 0x5b, 0xbb, 
0xbb, 0xb2, 0xed, 0xed, 0xed, 0xed, 0xbb, 0x14, 0x4b, 0xbb, 0xbb, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x9b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xef, 0xed, 0xed, 0x2b, 0xbb, 
0xbb, 0xbb, 0xde, 0xde, 0xde, 0xde, 0xbb, 0x1a, 0xa9, 0xa4, 0xbb, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x0b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xde, 0xde, 0xd5, 0xbb, 0xbb, 
0xbb, 0xbb, 0x5f, 0xed, 0xef, 0xfd, 0xeb, 0xbb, 0xbb, 0xbb, 0xbf, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xeb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xbf, 0xed, 0xef, 0xc2, 0xbb, 0xbb, 
0xbb, 0xbb, 0xb5, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xab, 0xbb, 0xbb, 0xbb, 0xbb, 0xde, 0xde, 0xde, 0x5b, 0xbb, 0xbb, 
0xbb, 0xbb, 0xb2, 0xef, 0xed, 0xed, 0xed, 0xed, 0xed, 0xed, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xe0, 0xbb, 0xbb, 0xbb, 0xaa, 0x9a, 0xab, 0xbb, 0xbb, 0xbf, 0xed, 0xed, 0xed, 0x2b, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0x2e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xdb, 0xbb, 0xbb, 0xb9, 0xaa, 0xbb, 0xbb, 0xbe, 0xde, 0xde, 0xde, 0xd2, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xb2, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xeb, 0xbb, 0xbb, 0xab, 0xbb, 0xbd, 0xef, 0xfd, 0xef, 0xfd, 0x5b, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0x2e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd5, 0x0b, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0x25, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0x5b, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xb2, 0x2e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xd5, 0x0b, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xb2, 0xef, 0xed, 0xef, 0xfd, 0xed, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0xef, 0xed, 0xef, 0xfd, 0x2b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x25, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x52, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xb0, 0x5d, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xed, 0xef, 0xed, 0xed, 0xc2, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xb2, 0x2e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x52, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x25, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0xef, 0xfd, 0x52, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0x02, 0x25, 0x5e, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0xde, 0x55, 0x22, 0x2b, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 
0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb
};

static const uint32_t palette_ControlDiagram4BPP_UNCOMP[]=
{
	0x1c1c1c, 	0x5a5a5a, 	0xa20a09, 	0xeb1110, 
	0xfbe7e6, 	0xca0e0d, 	0x000000, 	0xec1211, 
	0x010002, 	0xf5f5f5, 	0xffffff, 	0x010000, 
	0xe1110f, 	0xe11013, 	0xe70f10, 	0xed1312
};

const Graphics_Image  ControlDiagram4BPP_UNCOMP=
{
	IMAGE_FMT_4BPP_UNCOMP,
	128,
	60,
	16,
	palette_ControlDiagram4BPP_UNCOMP,
	pixel_ControlDiagram4BPP_UNCOMP,
};
