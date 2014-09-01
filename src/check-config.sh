#!/bin/sh

excode=0

case "$1" in
'CONFIG=x86-dbg')
;;
'CONFIG=x86-rel')
;;
'CONFIG=x64-dbg')
;;
'CONFIG=x64-rel')
;;
*)
echo 'Error: Invalid value of CONFIG'
echo '       Valid values are: x86-dbg, x86-rel, x64-dbg, x64-rel'
excode=1
;;
esac

case "$2" in
'USE_NETWORK=0')
;;
'USE_NETWORK=1')
;;
*)
echo 'Error: Invalid value of USE_NETWORK'
echo '       Valid values are: 0, 1'
excode=1
;;
esac

if [ "$OSTYPE" = "" ]
then
echo 'Error: Missing value of OSTYPE'
echo '       Did you remember to "export OSTYPE" in your shell?'
excode=1
fi

exit $excode