docker run --rm -it \
  --name g1_sim_container \
  --network host \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  --volume ~/g1_sim_docker/src:/workspace/src \
  g1_sim