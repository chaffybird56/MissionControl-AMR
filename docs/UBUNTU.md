# Ubuntu setup (22.04 / 24.04)

MissionControl-AMR runs ROS 2 Humble, FastAPI, and the Next.js dashboard in **Docker Compose** on Linux. GitHub Actions CI uses `ubuntu-latest` for Python smoke tests; the full stack is intended to run the same way on your machine.

## Requirements

- Ubuntu **22.04** or **24.04** (64-bit)
- **8 GB+ RAM** available to Docker (16 GB recommended)
- **10 GB+** free disk
- User in the `docker` group (so you do not need `sudo` for every compose command)

## 1. Install Docker

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-plugin make git curl
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

**Log out and log back in** (or reboot) so group membership applies. Verify:

```bash
docker --version
docker compose version
docker run --rm hello-world
```

## 2. Clone and start

```bash
git clone https://github.com/chaffybird56/MissionControl-AMR.git
cd MissionControl-AMR/ops
docker compose up -d
```

Check containers:

```bash
docker compose ps
```

## 3. Open the dashboard

From `ops/net.env` / compose defaults (see README if ports differ):

| Service    | URL                      |
|-----------|--------------------------|
| Dashboard | http://localhost:3002    |
| API       | http://localhost:8002    |
| Metrics   | http://localhost:8080/metrics |

Health check:

```bash
curl -s http://localhost:8002/health || curl -s http://localhost:8002/api/status
```

## 4. Useful commands (from `ops/`)

```bash
make status          # container status
make logs            # all logs
make logs-ros2       # ROS 2 core
make logs-webots     # Webots simulation
make down            # stop stack
make up              # start again
./../scripts/verify_topics.sh
```

## Webots on Ubuntu (optional)

The compose file can start Webots in a container. For a **GUI** you need X11:

```bash
sudo apt-get install -y x11-xserver-utils
xhost +local:docker
export DISPLAY=${DISPLAY:-:0}
cd MissionControl-AMR/ops
docker compose up -d
```

If Webots fails (common on headless servers), the **API and dashboard may still run**; inspect with `make logs-webots`. For simulation-only development without a display, use a machine with a desktop session or VNC.

## Troubleshooting

**Permission denied on Docker**

```bash
groups | grep docker   # should list docker after re-login
```

**Ports already in use**

Edit `ops/net.env` or stop conflicting services, then `docker compose up -d` again.

**Low memory / containers exit**

```bash
docker stats --no-stream
free -h
```

Give Docker more RAM or close other workloads; 8 GB minimum for the full stack.

**ROS topics missing**

```bash
cd MissionControl-AMR
./scripts/verify_topics.sh
make logs-ros2
```

## Reporting issues

When opening a GitHub issue, include:

- `lsb_release -a`
- `docker --version` and `docker compose version`
- Output of `docker compose ps` from `ops/`
- Relevant log snippet: `make logs` or `make logs-ros2`
