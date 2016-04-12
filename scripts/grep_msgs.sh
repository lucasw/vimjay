#!/bin/sh
type=Image
rosmsg list |
while IFS= read -r line
do
  rosmsg show $line | grep $type && echo $line
  #test=`rosmsg show $line | grep $type` 
  #echo $test \n
  #if testecho $line
done
