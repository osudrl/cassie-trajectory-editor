#!/bin/bash

sudo apt install libglfw3-dev
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$DIR/../mjpro150/bin"

