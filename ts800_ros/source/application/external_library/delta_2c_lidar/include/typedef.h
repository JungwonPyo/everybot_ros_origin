
#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#include <stdint.h>

typedef int64_t             s64;
typedef uint64_t            u64;


#define HWREG(x)                            	 (*((volatile unsigned long *)(x)))
#define HWREGS(x)                            	 (*((volatile signed long *)(x)))
#define HSREG(x)								 (*((volatile unsigned short *)(x)))
#define HBREG(x)                            	 (*((volatile unsigned char *)(x)))

#define U8_MAX     								 ((uint8_t)255)
#define S8_MAX     								 ((int8_t)127)
#define S8_MIN     								 ((int8_t)-128)
#define U16_MAX    								 ((uint16_t)65535u)
#define S16_MAX    								 ((int16_t)32767)
#define S16_MIN    								 ((int16_t)-32768)
#define U32_MAX    								 ((uint32_t)4294967295uL)
#define S32_MAX    								 ((int32_t)2147483647)
#define S32_MIN    								 ((int32_t)-2147483648)

#endif //__TYPEDEF_H__
