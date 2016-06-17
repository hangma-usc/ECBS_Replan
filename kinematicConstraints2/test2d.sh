MAP=~/projects/phd/mapf/codeHang/examples/tiny_map.json
MAP_OLDSTYLE=~/projects/phd/mapf/codeHang/examples/tiny_map
AGENTS=~/projects/phd/mapf/codeHang/examples/tiny.agents
AGENT_KINEMATICS=~/projects/phd/mapf/kinematicConstraints/examples/agents32.json

cd ../codeHang/build
make -j8
./driver -m $MAP -a $AGENTS -o test.json
cd ../../kinematicConstraints2/build
make -j8
cd ..
# build/kinematicConstraints2 -a $AGENT_KINEMATICS -s ../codeHang/build/test.json --delta 0.25
build/kinematicConstraints2 -a $AGENT_KINEMATICS -s examples/schedule_tiny.json --delta 0.25
# dot -Tpng -o stp.png stp.dot
dot -Kneato -Tsvg -o stp.svg stp.dot

cd ../visualizer
./visDiscrete.sh $MAP_OLDSTYLE ../codeHang/build/test.json
./vis.sh $MAP_OLDSTYLE ../kinematicConstraints2/schedule.json 0.4 30
cd ../kinematicConstraints2
cp ../visualizer/output.mp4 .
cp ../visualizer/iter.zip .


