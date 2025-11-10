#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H


typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */

//定义一个新的类型名 bool_t，它实际上就是 unsigned char（无符号 8 位整型）。
//这样你可以用 bool_t 来声明变量，表示布尔类型（通常用 0 表示假，非 0 表示真），但本质上它还是一个 unsigned char 类型。
//主要用于代码可读性和兼容性。
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;

typedef float fp32;
typedef double fp64;

#endif



