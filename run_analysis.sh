FILES="./instances/assignments/*.txt"

for f in $FILES
do
  if [ -f "$f" ]
  then
    filename="${f%.*}"
    echo $filename
    sudo podman run -itd --name $(basename $filename) agent-based python3 --instance /app/$filename --solver CBS
  else
    echo "Warning: Some problem with \"$f\""
  fi
done
