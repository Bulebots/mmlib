#include "common.h"

/**
 * @brief Return the sign of a number.
 *
 * @param number The number to get the sign from.
 *
 * @return The sign of the number.
 */
int sign(float number)
{
	return (int)(number > 0) - (int)(number < 0);
}

/**
 * @brief Check if the received buffer starts with the given string.
 *
 * @param[in] string String buffer.
 * @param[in] start_string Prefix to look for at the start of the string buffer.
 */
bool starts_with(char *string, char *start_string)
{
	return (bool)!strncmp(string, start_string, strlen(start_string));
}

/**
 * @brief Parse a float number in a given string.
 *
 * The parsing will start after a defined number of spaces that are expected
 * before the float in the string.
 *
 * @param[in] string String to parse the float from.
 * @param[in] string_size Size from the string to parse the float from.
 * @param[in] spaces_before Number of spaces expected before the float.
 */
float parse_float(char *string, unsigned string_size, int spaces_before)
{
	unsigned int i;
	char *pointer;

	pointer = string;
	for (i = 0; i < string_size; i++) {
		if (string[i] == ' ')
			spaces_before--;
		if (string[i] == '\0')
			return 0.;
		pointer++;
		if (spaces_before == 0)
			break;
	}
	return strtof(pointer, NULL);
}
