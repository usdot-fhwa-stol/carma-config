xhost + && \
docker run \
-ti \
--rm \
-e DISPLAY \
--env QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--net=host \
--name test1a \
usdotfhwastol/carma-platform:latest bash