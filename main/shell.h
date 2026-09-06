/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stddef.h>

void shell_loop(void);
int shell_getline(char *line, size_t len);
