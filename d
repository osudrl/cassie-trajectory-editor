#!/bin/bash

echo $@ | sed 's;\(.*\);git commit *.md -m "\1";' | bash
git push &> /dev/null &


