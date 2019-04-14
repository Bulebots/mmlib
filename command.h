#ifndef __COMMAND_H
#define __COMMAND_H

#include "mmlib/calibration.h"
#include "mmlib/common.h"
#include "mmlib/control.h"
#include "mmlib/logging.h"
#include "mmlib/move.h"
#include "mmlib/speed.h"

#include "config.h"
#include "serial.h"

void execute_command(void);

#endif /* __COMMAND_H */
