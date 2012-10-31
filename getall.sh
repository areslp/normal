#!/bin/sh
echo "start"
./updatecpp
# low rank 
# matlab -nodesktop -nosplash -nojvm -r "lrr;quit;"
matlab -nodesktop -nosplash -nojvm -r "lrr_neighborhood;quit;"
echo "end"
