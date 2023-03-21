#!/usr/bin/env bash

print_help() {
    if [ $# -gt 0 ] ; then
        echo -e "ERROR: $*\n"
    fi
    echo "Usage: $0 [options] inputlcm"
    echo "Generate CPP and ROS message definitions from lcm messages".
    echo "Options:"
    echo "  -a      Use all files in lcm/*.lcm"
    echo "  -h      Print this help and exit"
    echo -e "\nSample usage:"
    echo " $0 -a"
    echo "     Generate CPP and ROS message defs, for each file lcm/*.lcm"
    exit 2
}

FULLDIR=false

while getopts ':ah' flag; do
  case "${flag}" in
    a) FULLDIR=true
       echo " * Using all files in lcm/*.lcm" >&2 ;;
    h) print_help ;;
    \?) print_help "Invalid option: -$OPTARG" ;;
  esac
done

shift $((OPTIND-1))

if [ $# -eq 0 ] && [ "$FULLDIR" = false ] ; then
    print_help "Must specify either input files or -a"
fi

if [ "$FULLDIR" = true ] ; then
    INFILES=lcm/*.lcm
else
    INFILES=$@
fi

unamestr=`uname`

for INFILE in $INFILES ; do
    echo "Processing LCM message file: $INFILE" >&2
    
    # Get the topic name, message type and package name
    MESSAGE_TYPE=$(cat $INFILE | awk '/struct/ {print $2}')
    PACKAGE_NAME=$(cat $INFILE | awk -F'[ ;]' '/package/ {print $2}')
    NEW_PACKAGE_NAME=$PACKAGE_NAME"_rehash"
    HASH_VALUE=$(cat $INFILE | awk '/HASH/ {print $3}')
    
    echo -n -e "\tGenerating CPP message autosrc/$PACKAGE_NAME/$MESSAGE_TYPE.hpp with lcm-gen..."
    # Create lcm CPP header (in package subfolder)
    lcm-gen -x $INFILE --cpp-hpath include/
    test $? != 0  && { echo "LCM conversion failed, skipping $INFILE"; continue; }
    echo "done."
done

