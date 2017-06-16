/***************************************************************************//**
* \file piksi.h
*
* \brief Standalone C Driver for the Swift Navigation Piksi GPS (header)
* \author Scott K Logan
* \author Caleb Jamison
* \date Febuary 23, 2014
*
* API for the standalone C driver
*
* \section license License (BSD-3)
* Copyright (c) 2013, Scott K Logan\n
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of Willow Garage, Inc. nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef _piksi_h
#define _piksi_h

#ifdef __cplusplus
extern "C" {
#endif

#include <libsbp/sbp.h>

#include <termios.h>

enum piksi_error
{
	PIKSI_SUCCESS = 0,
	PIKSI_ERROR_IO = -1,
	PIKSI_ERROR_NO_DEVICE = -4,
	PIKSI_ERROR_NO_MEM = -11,
	PIKSI_ERROR_OTHER = -13,
	PIKSI_ERROR_TIMEOUT = -14
};

int piksi_open( const char *port, const int baud_term_rate);
void piksi_close( const int8_t piksid );

u8 piksi_spin( const int8_t piksid );

u32 send_cmd( u8 *data, u32 num_bytes, void* context );
u32 read_data( u8 *data, u32 num_bytes, void* context );

static int baud2term( int baud )
{
    switch( baud )
    {
    case 1200:
        return B1200;
        break;
    case 2400:
        return B2400;
        break;
    case 4800:
        return B4800;
        break;
    case 9600:
        return B9600;
        break;
    case 19200:
        return B19200;
        break;
    case 38400:
        return B38400;
        break;
    case 57600:
        return B57600;
        break;
    case 115200:
        return B115200;
        break;
    case 230400:
        return B230400;
        break;
    case 460800:
        return B460800;
        break;
    case 921600:
        return B921600;
        break;
    case 1000000:
        return B1000000;
        break;
    default:
        return B0;
        break;
    }
}


#ifdef __cplusplus
}
#endif

#endif /* _piksi_h */
