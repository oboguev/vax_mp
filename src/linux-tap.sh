#!/bin/sh

#
#  Create tap device owned by specified user and bridge it to the Ethernet interface
#  For usage invoke as ./linux-tap.sh help
#

if [ "`which /usr/sbin/brctl`" = "" ]
then
    echo "Error: brctl is not installed"
    exit 1
fi

if [ "`which /usr/sbin/tunctl`" = "" ]
then
    echo "Error: tunctl is not installed"
    exit 1
fi

if [ "`which gawk`" = "" ]
then
    echo "Error: gawk is not installed"
    exit 1
fi

VERB=$1

case "$VERB" in
'create')
    #
    ############################################################################################
    #
    BRDEV=$2
    ETHDEV=$3
    TAPDEV=$4
    USERID=$5
    #
    echo BRDEV=$BRDEV
    echo ETHDEV=$ETHDEV
    echo TAPDEV=$TAPDEV
    echo USERID=$USERID
    if [ "$TAPDEV" = "" ] || [ "$ETHDEV" = "" ] || [ "$BRDEV" = "" ] || [ "$USERID" = "" ]
    then
        echo "Error: missing argument"
        exit 1
    fi
    #
    ############################################################################################
    #
    HOSTIP=`/sbin/ifconfig $ETHDEV | grep "inet addr" | gawk -- '{ print $2 }' | gawk -F : -- '{ print $2 }'`
    HOSTNETMASK=`/sbin/ifconfig $ETHDEV | grep "inet addr" | gawk -- '{ print $4 }' | gawk -F : -- '{ print $2 }'`
    HOSTBCASTADDR=`/sbin/ifconfig $ETHDEV | grep "inet addr" | gawk -- '{ print $3 }' | gawk -F : -- '{ print $2 }'`
    HOSTDEFAULTGATEWAY=`/sbin/route -n | grep ^0.0.0.0 | gawk -- '{ print $2 }'`
    #
    ############################################################################################
    #
    echo HOSTIP=$HOSTIP
    echo HOSTNETMASK=$HOSTNETMASK
    echo HOSTBCASTADDR=$HOSTBCASTADDR
    echo HOSTDEFAULTGATEWAY=$HOSTDEFAULTGATEWAY
    if [ "$HOSTIP" = "" ] || [ "$HOSTNETMASK" = "" ] || [ "$HOSTBCASTADDR" = "" ] || [ "$HOSTDEFAULTGATEWAY" = "" ]
    then
        echo "Error: missing network configuration data"
        exit 1
    fi
    #
    ############################################################################################
    #
    /usr/sbin/tunctl -t $TAPDEV -u $USERID
    /sbin/ifconfig $TAPDEV up
    #
    ############################################################################################
    #
    #  Now convert $ETHDEV to a bridge and bridge it with the TAP interface
    #
    /usr/sbin/brctl addbr $BRDEV
    /usr/sbin/brctl addif $BRDEV $ETHDEV
    /usr/sbin/brctl setfd $BRDEV 0
    /sbin/ifconfig $ETHDEV 0.0.0.0
    /sbin/ifconfig $BRDEV $HOSTIP netmask $HOSTNETMASK broadcast $HOSTBCASTADDR up
    #
    #  Set the default route to the $BRDEV interface
    #
    /sbin/route add -net 0.0.0.0/0 gw $HOSTDEFAULTGATEWAY
    #
    #  Bridge in the tap device
    #
    /usr/sbin/brctl addif $BRDEV $TAPDEV
    /sbin/ifconfig $TAPDEV 0.0.0.0
    ;;


addtap)
    #
    ############################################################################################
    #
    BRDEV=$2
    TAPDEV=$3
    USERID=$4
    #
    echo BRDEV=$BRDEV
    echo TAPDEV=$TAPDEV
    echo USERID=$USERID
    if [ "$TAPDEV" = "" ] || [ "$BRDEV" = "" ] || [ "$USERID" = "" ]
    then
        echo "Error: missing argument"
        exit 1
    fi
    #
    ############################################################################################
    #
    /usr/sbin/tunctl -t $TAPDEV -u $USERID
    /sbin/ifconfig $TAPDEV up
    /usr/sbin/brctl addif $BRDEV $TAPDEV
    /sbin/ifconfig $TAPDEV 0.0.0.0
    ;;


deltap)
    #
    ############################################################################################
    #
    BRDEV=$2
    TAPDEV=$3
    #
    echo BRDEV=$BRDEV
    echo TAPDEV=$TAPDEV
    if [ "$TAPDEV" = "" ] || [ "$BRDEV" = "" ]
    then
        echo "Error: missing argument"
        exit 1
    fi
    #
    ############################################################################################
    #
    /sbin/ifconfig $TAPDEV down
    /usr/sbin/brctl delif $BRDEV $TAPDEV
    /usr/sbin/tunctl -d $TAPDEV
    ;;

destroy)
    #
    ############################################################################################
    #
    BRDEV=$2
    ETHDEV=$3
    TAPDEV=$4
    #
    echo TAPDEV=$TAPDEV
    echo ETHDEV=$ETHDEV
    echo BRDEV=$BRDEV
    if [ "$TAPDEV" = "" ] || [ "$ETHDEV" = "" ] || [ "$BRDEV" = "" ]
    then
        echo "Error: missing argument"
        exit 1
    fi
    /sbin/ifconfig $BRDEV
    #
    ############################################################################################
    #
    HOSTIP=`/sbin/ifconfig $BRDEV | grep "inet addr" | gawk -- '{ print $2 }' | gawk -F : -- '{ print $2 }'`
    HOSTNETMASK=`/sbin/ifconfig $BRDEV | grep "inet addr" | gawk -- '{ print $4 }' | gawk -F : -- '{ print $2 }'`
    HOSTBCASTADDR=`/sbin/ifconfig $BRDEV | grep "inet addr" | gawk -- '{ print $3 }' | gawk -F : -- '{ print $2 }'`
    HOSTDEFAULTGATEWAY=`/sbin/route -n | grep ^0.0.0.0 | gawk -- '{ print $2 }'`
    #
    ############################################################################################
    #
    echo HOSTIP=$HOSTIP
    echo HOSTNETMASK=$HOSTNETMASK
    echo HOSTBCASTADDR=$HOSTBCASTADDR
    echo HOSTDEFAULTGATEWAY=$HOSTDEFAULTGATEWAY
    if [ "$HOSTIP" = "" ] || [ "$HOSTNETMASK" = "" ] || [ "$HOSTBCASTADDR" = "" ] || [ "$HOSTDEFAULTGATEWAY" = "" ]
    then
        echo "Warning: missing network configuration data"
        exit 1
    fi
    #
    /sbin/ifconfig $BRDEV down
    /usr/sbin/brctl delbr $BRDEV
    /sbin/ifconfig $TAPDEV down
    /usr/sbin/tunctl -d $TAPDEV
    #
    if [ "$HOSTIP" = "" ] || [ "$HOSTNETMASK" = "" ] || [ "$HOSTBCASTADDR" = "" ]
    then
        echo "Warning: Unable to configure $ETHDEV because configuration data is missing"
        echo "         Execute manually: ifconfig $ETHDEV <inetaddr> netmask <mask> broadcast <broadcast> up"
    else
        /sbin/ifconfig $ETHDEV $HOSTIP netmask $HOSTNETMASK broadcast $HOSTBCASTADDR up
    fi
    #
    #  Set the default route to the $ETHDEV interface
    #
    /sbin/route add -net 0.0.0.0/0 gw $HOSTDEFAULTGATEWAY
    ;;


*)
    if [ "$VERB" != "help" ] && [ "$VERB" != "" ]
    then
        echo "Error: Invalid command verb."
        echo ""
    fi

    echo "Usage:"
    echo ""
    echo "  To create the bridge, create tap interface and add it to the bridge: "
    echo ""
    echo "      Execute:     $0 create BRDEV ETHDEV TAPDEV USERID"
    echo ""
    echo "      Example:     $0 create br0 eth0 tap0 vaxuser"
    echo ""
    echo "  To create additional tap interface and add it to the bridge: "
    echo ""
    echo "      Execute:     $0 addtap BRDEV TAPDEV USERID"
    echo ""
    echo "      Example:     $0 addtap br0 tap2 vaxuser2"
    echo "                   $0 addtap br0 tap9 vaxuser9"
    echo ""
    echo "  To destroy addtional tap interface (beyond the last): "
    echo ""
    echo "      Execute:     $0 deltap BRDEV TAPDEV"
    echo ""
    echo "      Example:     $0 deltap br0 tap1"
    echo ""
    echo "  To destroy the bridge and the only remaining tap interface: "
    echo ""
    echo "      Execute:     $0 destroy BRDEV ETHDEV TAPDEV"
    echo "      Optionally:  ifconfig ETHDEV <recorded inet addr> netmask <mask> broadcast <broadcast> up"
    echo ""
    echo "      Example:     $0 create br0 eth0 tap0"
    echo "      Optionally:  ifconfig eth0 ..."
    echo ""
    echo "  To check current host network configuration, use commands:"
    echo ""
    echo "      brctl show"
    echo "      brctl show BRDEV  (e.g. brctl show br0)"
    echo "      brctl showmacs BRDEV  (e.g. brctl showmacs br0)"
    echo "      ifconfig"
    echo "      route -n"
    echo ""
    ;;
esac
