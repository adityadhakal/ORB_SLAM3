#! /bin/bash
for i in 10 20 30 40 50 60 70 80 90 100
do
    echo $i
    tail -n 200 "${i}p/tracking_times.txt" | datamash -t, mean 1 pstdev 1 mean 2 pstdev 2
done
