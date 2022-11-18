FILES="./instances/assignments_distributed/*.txt"

for f in $FILES
do
  if [ -f "$f" ]
  then
    filename="${f%.*}"
    echo $filename
    sudo podman run -itd --name distributed_$(basename $filename) agent-based python3 /app/run_orchestrator_multi.py --instance /app/$filename.txt --solver Distributed
  else
    echo "Warning: Some problem with \"$f\""
  fi
done
