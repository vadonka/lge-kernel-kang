#!/bin/sh

#!/bin/bash
revisioncount=`git log --oneline | wc -l`
projectversion=`git describe --tags --long`
cleanversion=${projectversion%%-*}

echo "$projectversion-$revisioncount" > VERSION
#echo "$cleanversion.$revisioncount" > VERSION
