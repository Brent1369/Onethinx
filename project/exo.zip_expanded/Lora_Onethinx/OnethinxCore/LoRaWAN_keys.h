/* ==========================================================
 *    ___             _   _     _			
 *   / _ \ _ __   ___| |_| |__ (_)_ __ __  __
 *  | | | | '_ \ / _ \ __| '_ \| | '_ \\ \/ /
 *  | |_| | | | |  __/ |_| | | | | | | |>  < 
 *   \___/|_| |_|\___|\__|_| |_|_|_| |_/_/\_\
 *
 * Copyright Onethinx, 2018
 * All Rights Reserved
 *
 * UNPUBLISHED, LICENSED SOFTWARE.
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Onethinx BV
 *
 * ==========================================================
*/

#ifndef LORAWAN_KEYS_H
#define LORAWAN_KEYS_H

#include "OnethinxCore01.h"

//a81965fffe0697b0
//39644542483274684d6a793169666934
LoRaWAN_keys_t TTN_OTAAkeys = {
	.KeyType 						= OTAA_10x_key,
	.PublicNetwork					= true,
	.OTAA_10x.DevEui				= { 0xa8, 0x19, 0x65, 0xff, 0xfe, 0x06, 0x97, 0xb0 }, //{ 0xa8, 0x19, 0x65, 0xff, 0xfe, 0x06, 0x97, 0xb0 }
	.OTAA_10x.AppEui				= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	.OTAA_10x.AppKey				= { 0x39, 0x64 ,0x45   ,0x42  ,0x48  ,0x32  ,0x74  ,0x68  ,0x4d  ,0x6a  ,0x79  ,0x31  ,0x69  ,0x66  ,0x69  ,0x34 }
};

#endif /* LORAWAN_KEYS_H */
/* [] END OF FILE */
