#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cat $DIR/../model/cassie.xml | grep "body name=" | sed "s;\s*<body name='\([^\']*\)\(.*\);\1;" | grep -v \! | cat -n