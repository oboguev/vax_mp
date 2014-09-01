/*
 * HW.C - Hello World program for ST-CC stress test
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <varargs.h>
#include <ssdef.h>
#include <stsdef.h>
#include <descrip.h>
#include <starlet.h>
#include <math.h>

int main(int argc, char** argv)
{
    printf("Hello, World!\n");
    return SS$_NORMAL;
}
