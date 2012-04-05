#!/bin/bash

for a in `git commit | grep deleted | awk 'BEGIN {FS=" "} {print $3}'`; do
    git rm /$a
done
