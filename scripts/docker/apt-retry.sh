#!/bin/sh

i=0
r=1

while [ $i -lt 5 -a $r != 0 ]; do \
    i=$(($i + 1))
    apt-get -y update && \
    apt-get install -y $@
    r=$?
done

return $r
