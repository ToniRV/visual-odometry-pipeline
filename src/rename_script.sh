#!/bin/bash
FILES=/home/tonirv/Documents/visual-odometry-pipeline/src/images/*
i=0
for f in $FILES
do
  i=$(($i+1))
  filename=$(printf "%04d" $i)
  echo "Processing $f file..."
  new_name=/home/tonirv/Documents/visual-odometry-pipeline/src/$filename.jpg
  cp "$f" "$new_name"
  # take action on each file. $f store current file name
done
