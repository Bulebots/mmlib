#ifndef __COMMON_H
#define __COMMON_H

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

int sign(float number);
bool starts_with(char *string, char *start_string);
float parse_float(char *string, unsigned string_size, int spaces_before);

#endif /* __COMMON_H */
