

echo $1 > keyskip.txt
rm bin/ik.o
make
./traj | grep -o 'poses in .* seconds' | sed 's;[a-z\ ];;g' > secs.txt
./visualize.py &
sleep 2
