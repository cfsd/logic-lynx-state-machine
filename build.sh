
docker build -t logic-lynx-state-machine-v0.0.1 -f Dockerfile.armhf .
docker save  logic-lynx-state-machine-v0.0.1 > logic-lynx-state-machine-v0.0.1.tar
