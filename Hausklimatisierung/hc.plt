set yrange [0:30]
set title("Temperature");
set xlabel("min");
set ylabel("°C");
set grid;
plot "h.dat" using 1:2 with lines,  "h.dat" using 1:3 with lines
pause 2
reread
