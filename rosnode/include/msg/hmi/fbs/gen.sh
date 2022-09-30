#!/bin/sh

flatc -c *.fbs
cp *_generated.h ../generated
mv *_generated.h /home/wdq/work/workspace/code/RBN100/rbn100message/msg/hmi/generated
