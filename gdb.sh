#!/bin/bash 
#===============================================================================
#
#          FILE:  gdb.sh
# 
#         USAGE:  ./gdb.sh 
# 
#   DESCRIPTION:  
# 
#       OPTIONS:  ---
#  REQUIREMENTS:  ---
#          BUGS:  ---
#         NOTES:  ---
#        AUTHOR:  Zakaria ElQotbi (zskdan), zakaria.elqotbi@redbend.com
#       COMPANY:  RedBend, Paris
#       VERSION:  1.0
#       CREATED:  20/11/2016 22:04:08 CET
#      REVISION:  ---
#===============================================================================

[ -z "$(pidof openocd)" ] && \
    sudo nuttx-configs/px4-same70xplained/tools/oocd.sh $PWD 2>&1 >/dev/null

cgdb -d arm-none-eabi-gdb -x nuttx-configs/px4-same70xplained/gdbinit
