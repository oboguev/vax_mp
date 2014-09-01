/*
 * NUMTIM.C 
 *
 *    Perform SYS$NUMTIM test
 *
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
#include <lnmdef.h>
#include <math.h>

int main(int argc, char** argv)
{
    unsigned short t[7];
    int status;

    status = sys$numtim (t, NULL);
    if (! $VMS_STATUS_SUCCESS(status))
        exit(status);

    printf("%d-%d-%d  %02d:%02d:%02d.%02d\n", t[2], t[1], t[0], t[3], t[4], t[5], t[6]);

    return SS$_NORMAL;
}
