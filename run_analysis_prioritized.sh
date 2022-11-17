FILES="./instances/assignments_prioritized/*.txt"

for f in $FILES
do
  if [ -f "$f" ]
  then
    filename="${f%.*}"
    echo $filename
    podman run -it --name prioritized_$(basename $filename) agent-based python3 /app/run_orchestrator_multi.py --instance /app/$filename.txt --solver Prioritized
  else
    echo "Warning: Some problem with \"$f\""
  fi
done
