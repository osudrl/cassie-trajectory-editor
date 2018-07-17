./auto.py | tee auto-swingleg.csv
rm dropdata.bin
cp leg-lift-data.bin dropdata.bin
./auto.py | tee auto-liftleg.csv

