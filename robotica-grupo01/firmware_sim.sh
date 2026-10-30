#!/usr/bin/env bash

NAME=simulator
IMG=danielwang123321/uf-ubuntu-docker

docker pull --platform linux/amd64 "$IMG"

if docker container inspect "$NAME" >/dev/null 2>&1; then
  docker start "$NAME"
else
  docker run -dit --name "$NAME" --platform linux/amd64 \
  -p 18333:18333 -p 502:502 -p 503:503 -p 504:504 \
  -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 \
  "$IMG"
fi

docker exec -i "$NAME" /bin/bash -lc '/xarm_scripts/xarm_start.sh 6 9'