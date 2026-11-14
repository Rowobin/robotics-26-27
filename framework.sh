# ./framework.sh
# /xarm_scripts/xarm_start.sh 6 9

docker pull danielwang123321/uf-ubuntu-docker
docker run -it --name uf_software \
  -p 18333:18333 -p 502:502 -p 503:503 -p 504:504 \
  -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 \
  danielwang123321/uf-ubuntu-docker
# inside that container: start the firmware, selecting the Lite 6 profile
# (arguments "6 9") — see UFACTORY's support article
# "How to install UFACTORY Studio in Docker"