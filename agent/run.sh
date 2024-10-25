#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 -c1 | -c2 | -c3"
    exit 1
fi

source env/bin/activate
case "$1" in
    -c1)
        python3 challenge1.py
        ;;
    -c2)
        python3 challenge2.py
        ;;
    -c3)
        python3 challenge3.py
        ;;
    *)
        echo "Invalid option. Use -c1, -c2 ou -c3."
        exit 1
        ;;
esac

deactivate
