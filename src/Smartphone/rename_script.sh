#!/bin/bash
FILES=/home/tonirv/Downloads/dataset/*
i=0
for f in $FILES
do
  i=$(($i+1))
  echo "Processing $f file..."
  new_name=/home/tonirv/Downloads/dataset/$i.jpg
  mv "$f" "$new_name"
  # take action on each file. $f store current file name
done
