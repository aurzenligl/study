#!/bin/bash

DIR=/tmp/tracer

echo "starting..."

sleep 3
echo "setting up..."

mkdir -p $DIR
echo 'done'
echo 'done' >> $DIR/signal0
sleep 3
