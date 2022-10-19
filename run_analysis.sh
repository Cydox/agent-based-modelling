FILES="./instances/assignments/*.txt"

for f in $FILES
do
  if [ -f "$f" ]
  then
    filename="${f%.*}"
    echo $filename
    sudo podman run -itd --name $(basename $filename) agent-based python3 /app/run_orchestrator_multi.py --instance /app/$filename.txt --solver CBS
  else
    echo "Warning: Some problem with \"$f\""
  fi
done
