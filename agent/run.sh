#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

source env/bin/activate
case $challenge in
    1)
        python3 challenge1.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        python3 challenge2.py -h "$host" -p "$pos" -r "$robname"
        mv map.map $outfile.map
        ;;
    3)
        python3 challenge3.py -h "$host" -p "$pos" -r "$robname"
        mv planning.path $outfile.path
        ;;
    4)
        python3 challenge4.py -h "$host" -p "$pos" -r "$robname"
        mv planning.path $outfile.path
        mv map.map $outfile.map
        ;;
esac

deactivate
