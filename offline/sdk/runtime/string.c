//=============================================================================
// memcpy, memmove, memset and memcmp for ROBO TX Controller programs.
//
// Programs don't link a C library (the firmware offers the usual functions in its hook table), but GCC may call
// these four on its own, e.g. to copy or clear a struct. Built with -fno-tree-loop-distribute-patterns so the
// loops don't turn into calls to themselves.
//=============================================================================

#include <stddef.h>

void * memcpy(void * destination, const void * source, size_t count)
{
    unsigned char * d = destination;
    const unsigned char * s = source;
    while (count--)
    {
        *d++ = *s++;
    }
    return destination;
}

void * memmove(void * destination, const void * source, size_t count)
{
    unsigned char * d = destination;
    const unsigned char * s = source;
    if (d < s)
    {
        while (count--)
        {
            *d++ = *s++;
        }
    }
    else
    {
        while (count--)
        {
            d[count] = s[count];
        }
    }
    return destination;
}

void * memset(void * destination, int value, size_t count)
{
    unsigned char * d = destination;
    while (count--)
    {
        *d++ = (unsigned char)value;
    }
    return destination;
}

int memcmp(const void * a, const void * b, size_t count)
{
    const unsigned char * x = a;
    const unsigned char * y = b;
    for (; count--; ++x, ++y)
    {
        if (*x != *y)
        {
            return *x - *y;
        }
    }
    return 0;
}
