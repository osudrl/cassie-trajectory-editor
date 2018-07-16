#!/bin/bash

rm bin/pdik.o
make &> /dev/null
bash runwithtimeout.sh &> /dev/null

if [ $? -ne 139 ]
then
./visualize.py
else
echo -1
fi



