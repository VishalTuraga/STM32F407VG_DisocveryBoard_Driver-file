/*
 * 008RTCLCD.c
 *
 *  Created on: Oct 8, 2024
 *      Author: ASUS
 */


#include <stdio.h>
#include <stdint.h>
#include "ds1307.h"

int main()
{
	RTC_time_t TIME;
	TIME.seconds = 45;
	TIME.minutes = 32;
	TIME.hours = 2;

	RTC_set_current_time(&TIME);

	return 0;
}
