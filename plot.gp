set terminal "pngcairo" size 600, 600
set output "out/3m3s.png"

set size square
unset key
plot "3m3s.dat" u 1:2 w l, "" u 3:4 w l, "" u 5:6 w l

set output "out/1m4s.png"
plot "1m4s.dat" u 1:2 w l
    