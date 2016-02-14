#ifndef STRINGUTILS_H_
#define STRINGUTILS_H_

#define UCHAR_MAX 255
#define USHORT_MAX 65535
#define MAX_FIELD_LENGTH 15

#include "ciaaPOSIX_stdio.h"

unsigned char readInteger(unsigned char * str, unsigned char * cantLeidoPtr, unsigned char * valorPtr);
unsigned char readFixedPoint(unsigned char * str, unsigned char exp, unsigned char * cantLeidoPtr, unsigned short * valorPtr);
unsigned char isNumber(unsigned char);

unsigned char * fixedPointToString(unsigned short, unsigned char, unsigned char, unsigned char *);
unsigned char * ushortToString(unsigned short, unsigned char, unsigned char *);
unsigned char * uintToString(unsigned int, unsigned char, unsigned char *);
unsigned char * hexIntToString(unsigned int, unsigned char, unsigned char *);

unsigned char strContainsChar(const unsigned char * str, unsigned char c);

size_t strlcpy(char *dest, const char *src, size_t size);

#endif /* STRINGUTILS_H_ */
