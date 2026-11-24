#!/usr/bin/env bash
# One-time host setup for QR-SimBench on Ubuntu 22.04.
# Installs Docker Engine + NVIDIA Container Toolkit so ROS 2 Jazzy / Gazebo Harmonic
# can run in a container with the local GPU. Run it yourself:  bash setup_host.sh
set -euo pipefail

echo "== Docker Engine (official apt repo) =="
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

echo "== NVIDIA Container Toolkit =="
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

echo "== Let '$USER' use docker without sudo =="
sudo usermod -aG docker "$USER"

echo "== Check (uses sudo because the group change needs a new login) =="
sudo docker run --rm --gpus all ubuntu:24.04 bash -c 'echo container OK' && nvidia-smi -L
echo
echo "Done. Log out and back in (or reboot) so 'docker' works without sudo, then tell Claude."
