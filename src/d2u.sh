#!/bin/sh

#
# When modifying this file, remember to save it in Unix (LF) format,
# rather than DOS (CR-LF) format.
#
# If it is unable to run becuase it was converted to CR-LF format, execute
# on Linux:
#
#     dos2unix d2u.sh
#
# on OS X:
#
#     perl -p -e 's/(\r\n|\n|\r)/\n/g' d2u.sh >d2u.tmp
#     mv -f d2u.tmp d2u.sh
#

if [ "`which perl`" = "" ] && [ "`which dos2unix`" = "" ]
then
    echo "Error: Neither dos2unix nor perl installed"
    exit 1
fi

if [ "$1" = "" ]
then
    find . -name "*.cpp" -exec $0 {} \;
    find . -name "*.c" -exec $0 {} \;
    find . -name "*.h" -exec $0 {} \;
    find . -name "*.com" -exec $0 {} \;
    find . -name "*.mar" -exec $0 {} \;
    find . -name "*.txt" -exec $0 {} \;
    find . -name "*.asm" -exec $0 {} \;
    find . -name "*.ini" -exec $0 {} \;
    find . -name "*.msg" -exec $0 {} \;
    $0 ./Makefile
    $0 ./makefile2
    find . \( -name "*.sh" -and -not -name "d2u.sh" \) -exec $0 {} \;
    find . -name "*.sh" -exec chmod a+x {} \;
    find . -name "*.COM" -exec $0 {} \;
    find . -name "*.C" -exec $0 {} \;
    find . -name "*.H" -exec $0 {} \;
    find . -name "*.MSG" -exec $0 {} \;
    find . -name "*.MAR" -exec $0 {} \;
    find . -name "*.MSG" -exec $0 {} \;
    find . -name "*.TXT" -exec $0 {} \;
elif [ "`which dos2unix`" != "" ]
then
    chmod o+w $1
    dos2unix $1
else
    echo Converting $1
    perl -p -e 's/(\r\n|\n|\r)/\n/g' $1 >$1.d2u.tmp
    touch -r $1 $1.d2u.tmp
    mv -f $1.d2u.tmp $1
fi
