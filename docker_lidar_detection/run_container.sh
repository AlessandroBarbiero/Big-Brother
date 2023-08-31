export DOCKER_REGISTRY="nvcr.io"
export DOCKER_NAME="nvidia/tao/tao-toolkit"
export DOCKER_TAG="5.0.0-pyt"

export DOCKER_CONTAINER=$DOCKER_REGISTRY/$DOCKER_NAME:$DOCKER_TAG

docker run -it --gpus all \
-v /home/ale/docker/shared:/shared \
$DOCKER_CONTAINER