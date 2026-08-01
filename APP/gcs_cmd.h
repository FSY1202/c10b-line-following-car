#ifndef __GCS_CMD_H
#define __GCS_CMD_H

#include <stdint.h>

// Standalone speed-test controller. The car starts automatically after power-up;
// RDK task, start, and speed commands have no effect in this project.
void GCS_Cmd_Init(void);
void GCS_Cmd_Poll(void);
void GCS_Cmd_Update(void);
uint8_t GCS_Cmd_Is_Started(void);

#endif
