# <map-file> <agents-file> <hwy-file> <mode> <res>
cd build
make -j8
cd ..
time build/driver --map $1 --agents $2 --highway $3 --delta 0.40 -f 1.5 --highway-weight 2 --objective $4
# dot -Tpdf stp.dot -o stp.pdf
# evince stp.pdf
cd ../visualizer
./vis.sh ../code/$1 ../code/schedule.json 0.4 $5
cd ../code
cp ../visualizer/output.mp4 .
