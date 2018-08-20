/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

#ifndef APP_EVENT_H_
#define APP_EVENT_H_

#include "ToCoNet_event.h"

typedef enum
{
	E_EVENT_APP_BASE = ToCoNet_EVENT_APP_BASE,
    E_PER_START,
    E_PER_STOP,
    E_PER_RESCAN_SLAVE,
    E_EVENT_TICK_A,
    E_EVENT_SCAN_FINISH,
    E_EVENT_SLAVE_CONF_FAIL,
    E_EVENT_SLEEP_REQUEST
} teEventApp;

// STATE MACHINE
typedef enum
{
	E_STATE_APP_BASE = ToCoNet_STATE_APP_BASE,
	E_STATE_PER_INIT,
	E_STATE_PER,
	E_STATE_PER_RESCAN,
	E_STATE_PER_RESCAN_PRE_WAIT,
	E_STATE_PER_FINISH,
	E_STATE_PRE_MEASURING
} teStateApp;

#endif /* EVENT_H_ */
