#!/bin/bash

while true; do
    ./startC2
    echo "C2 crashed with exit code $?.  Respawning.."
    sleep 1
done