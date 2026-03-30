# Setup — Kubernetes (K3s)

Deploy the dashboard as a proper Kubernetes service using K3s. This gives you an always-on dashboard accessible by hostname from any device on your network.

---

## Prerequisites

- K3s cluster running (1 control plane + workers)
- MetalLB configured with an IP pool
- Nginx Ingress Controller deployed
- Local container registry (or Docker Hub account)
- kubectl configured on your host machine
- Helm installed
- Raspberry Pi agent running (see [setup-bare.md](setup-bare.md) Step 1)

---

## Step 1 — Set Up K3s Cluster

If you don't have K3s running yet, here's a quick setup for a single-node cluster:

```bash
# On your server — install K3s
curl -sfL https://get.k3s.io | sh -

# Copy kubeconfig to your workstation
mkdir -p ~/.kube
scp user@SERVER_IP:/etc/rancher/k3s/k3s.yaml ~/.kube/config
sed -i 's/127.0.0.1/SERVER_IP/' ~/.kube/config

# Verify
kubectl get nodes
```

For a multi-node cluster setup, see the [K3s documentation](https://docs.k3s.io/).

---

## Step 2 — Install MetalLB

MetalLB provides LoadBalancer IPs for services on bare metal clusters:

```bash
helm repo add metallb https://metallb.github.io/metallb
helm repo update
helm install metallb metallb/metallb --namespace metallb-system --create-namespace
```

Configure an IP pool (use IPs outside your DHCP range):

```bash
kubectl apply -f - <<EOF
apiVersion: metallb.io/v1beta1
kind: IPAddressPool
metadata:
  name: homelab-pool
  namespace: metallb-system
spec:
  addresses:
  - 192.168.1.200-192.168.1.220   # adjust to your network
---
apiVersion: metallb.io/v1beta1
kind: L2Advertisement
metadata:
  name: homelab-l2
  namespace: metallb-system
spec:
  ipAddressPools:
  - homelab-pool
EOF
```

---

## Step 3 — Install Nginx Ingress Controller

```bash
helm repo add ingress-nginx https://kubernetes.github.io/ingress-nginx
helm repo update
helm install ingress-nginx ingress-nginx/ingress-nginx \
  --namespace ingress-nginx \
  --create-namespace
```

Check the assigned IP:
```bash
kubectl get svc -n ingress-nginx
# Note the EXTERNAL-IP — this is your ingress IP
```

---

## Step 4 — Set Up Local Registry

```bash
kubectl apply -f k8s/registry.yaml

# Check assigned IP
kubectl get svc registry -n kube-system
# Note the EXTERNAL-IP (e.g. 192.168.1.202)
```

Configure Docker to allow the insecure registry:

```bash
sudo nano /etc/docker/daemon.json
```

```json
{
  "insecure-registries": ["YOUR_REGISTRY_IP:5000"]
}
```

```bash
sudo systemctl restart docker
```

Configure K3s nodes to trust the registry. On each node:

```bash
sudo mkdir -p /etc/rancher/k3s
sudo tee /etc/rancher/k3s/registries.yaml <<EOF
mirrors:
  "YOUR_REGISTRY_IP:5000":
    endpoint:
      - "http://YOUR_REGISTRY_IP:5000"
EOF
sudo systemctl restart k3s  # or k3s-agent on worker nodes
```

---

## Step 5 — Build and Push Dashboard Image

Clone the repo and configure the Pi IP:

```bash
git clone https://github.com/gregholliday/picar-x-ai.git
cd picar-x-ai
sed -i 's/192.168.1.225/YOUR_PI_IP/g' dashboard/picar_dashboard.html
```

Build and push:

```bash
docker build -t YOUR_REGISTRY_IP:5000/picar-dashboard:latest dashboard/
docker push YOUR_REGISTRY_IP:5000/picar-dashboard:latest
```

---

## Step 6 — Deploy to K3s

Update the image in the deployment manifest:

```bash
sed -i 's|YOUR_REGISTRY/picar-dashboard|YOUR_REGISTRY_IP:5000/picar-dashboard|g' \
  k8s/dashboard-deployment.yaml
```

Update the hostname if you want something other than `picar.lab`:

```bash
nano k8s/dashboard-deployment.yaml
# Change host: picar.lab to your preferred hostname
```

Deploy:

```bash
kubectl apply -f k8s/dashboard-deployment.yaml
```

Verify:

```bash
kubectl get pods -n default
kubectl get svc -n default
kubectl get ingress -n default
```

---

## Step 7 — Configure DNS

### Option A — Router DNS (Recommended)
Add a local DNS record in your router pointing your hostname to the Nginx Ingress IP:
```
picar.lab → YOUR_INGRESS_IP
```

### Option B — Hosts File
Add to `/etc/hosts` on each machine:
```
YOUR_INGRESS_IP  picar.lab
```

On Windows: `C:\Windows\System32\drivers\etc\hosts`

### Note on .local domains
Avoid using `.local` as a domain suffix — Linux reserves it for mDNS which
causes resolution failures. Use `.lab`, `.home`, or `.internal` instead.

---

## Step 8 — Run the Navigator (Optional)

```bash
pip install -r navigator/requirements.txt
sed -i 's/192.168.1.225/YOUR_PI_IP/g' navigator/picar_navigator.py
python3 navigator/picar_navigator.py
```

---

## Updating the Dashboard

After making changes to the dashboard HTML:

```bash
sed -i 's/192.168.1.225/YOUR_PI_IP/g' dashboard/picar_dashboard.html
docker build -t YOUR_REGISTRY_IP:5000/picar-dashboard:latest dashboard/
docker push YOUR_REGISTRY_IP:5000/picar-dashboard:latest
kubectl rollout restart deployment/picar-dashboard -n default
```

---

## Troubleshooting

### Pods stuck in ImagePullBackOff
- Check registry is reachable: `curl http://YOUR_REGISTRY_IP:5000/v2/_catalog`
- Check K3s registries.yaml is correct on all nodes
- Restart K3s after registry config changes

### Dashboard shows but camera feed is blank
- Camera stream comes directly from Pi (port 9000), not through K3s
- Verify: `curl http://YOUR_PI_IP:9000/mjpg`
- Check Pi agent is running: `sudo systemctl status picar-agent`

### picar.lab doesn't resolve
- Check DNS record in router or hosts file
- On Linux with systemd-resolved, add to `/etc/systemd/resolved.conf`:
  ```ini
  [Resolve]
  DNS=YOUR_ROUTER_IP
  Domains=~lab ~.
  ```
  Then: `sudo systemctl restart systemd-resolved`
