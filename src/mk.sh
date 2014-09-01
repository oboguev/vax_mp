#!/bin/sh

#
# Convenience driver for VAX MP makefile.
#
# Invoke as:
#
#    mk.sh {x86|x64} {dbg|rel} {net|shr|nonet} [tap] [vde] [all vax_mp clean depend rebuild]
#
# Multiple targets can be specified for the given configuration.
#

CONF_ARCH=""
CONF_BUILD=""
CONF_NET=""
CONF_TAP=""
CONF_VDE=""
CONF_TARGETS=""
badargs="false"

for arg in $*
do

    case "$arg" in
    'x86' | 'x64')
        if [ "$CONF_ARCH" = "" ] || [ "$CONF_ARCH" = "$arg" ]
        then
            CONF_ARCH="$arg"
        else
            echo "Error: Conflicting arguments: $CONF_ARCH and $arg"
            badargs="true"
        fi
        ;;
    'dbg' | 'rel')
        if [ "$CONF_BUILD" = "" ] || [ "$CONF_BUILD" = "$arg" ]
        then
            CONF_BUILD="$arg"
        else
            echo "Error: Conflicting arguments: $CONF_BUILD and $arg"
            badargs="true"
        fi
        ;;
    'net' | 'nonet' | 'shr')
        if [ "$CONF_NET" = "" ] || [ "$CONF_NET" = "$arg" ]
        then
            CONF_NET="$arg"
        else
            echo "Error: Conflicting arguments: $CONF_NET and $arg"
            badargs="true"
        fi
        ;;
    'tap')
        CONF_TAP="true"
        ;;
    'vde')
        CONF_VDE="true"
        ;;
    'all' | 'vax_mp' | 'clean' | 'depend' | 'rebuild')
        if [ "$CONF_TARGETS" = "" ]
        then
            CONF_TARGETS="$arg"
        else
            CONF_TARGETS="$CONF_TARGETS $arg"
        fi
        ;;
    *)
        echo "Error: Invalid argument: $arg"
        badargs="true"
        ;;
    esac

done

if [ "$CONF_ARCH" = "" ]
then
    echo "Error: Missing target architecture parameter"
    badargs="true"
fi

if [ "$CONF_BUILD" = "" ]
then
    echo "Error: Missing target build  parameter"
    badargs="true"
fi

if [ "$CONF_NET" = "" ]
then
    echo "Error: Missing target network parameter, specify one of NONET, NET or SHR"
    badargs="true"
fi

if [ "$badargs" = "true" ]
then
    echo "Usage: mk.sh {x86|x64} {dbg|rel} {net|shr|nonet} [tap] [vde]"
    echo "             [all vax_mp clean depend rebuild]"
    echo ""
    echo "       x86|x64        select target host processor architecture"
    echo "       dbg|rel        select release or debug build"
    echo "       net|shr|nonet  select network support: static or dynamic library, or none"
    echo "       tap            support TAP devices for host-VM networking (Linux, OS X)"
    echo "       vde            support VDE networking (Linux)"
    exit 1
fi

if [ "$CONF_NET" = "net" ]
then
    USE_NETWORK=1
    USE_SHARED=0
fi

if [ "$CONF_NET" = "shr" ]
then
    USE_NETWORK=0
    USE_SHARED=1
fi

if [ "$CONF_NET" = "nonet" ]
then
    USE_NETWORK=0
    USE_SHARED=0
fi

if [ "$CONF_TAP" = "true" ] && [ "$USE_NETWORK" = "0" ] && [ "$USE_SHARED" = "0" ]
then
    echo "Error: TAP requires NET or SHR options"
    exit 1
fi

if [ "$CONF_VDE" = "true" ] && [ "$USE_NETWORK" = "0" ] && [ "$USE_SHARED" = "0" ]
then
    echo "Error: VDE requires NET or SHR options"
    exit 1
fi

if [ "$CONF_TAP" = "true" ]
then
    USE_TAP_NETWORK=1
else
    USE_TAP_NETWORK=0
fi

if [ "$CONF_VDE" = "true" ]
then
    USE_VDE_NETWORK=1
else
    USE_VDE_NETWORK=0
fi

export OSTYPE

make CONFIG=$CONF_ARCH-$CONF_BUILD USE_NETWORK=$USE_NETWORK USE_SHARED=$USE_SHARED USE_TAP_NETWORK=$USE_TAP_NETWORK USE_VDE_NETWORK=$USE_VDE_NETWORK $CONF_TARGETS
