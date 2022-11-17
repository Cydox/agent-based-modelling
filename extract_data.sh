FILES="./instances/assignments/*.txt"

for f in $FILES
do
  if [ -f "$f" ]
  then
    filename="${f%.*}"
    echo $filename
    mkdir -p results/$(basename $filename)
    sudo podman cp 9_$(basename $filename):/app/simulation_results results/$(basename $filename)/simulation_results
    sudo podman cp 9_$(basename $filename):/app/results.csv results/$(basename $filename)/results_intermediate.csv
  else
    echo "Warning: Some problem with \"$f\""
  fi
done
