# UTM Mac Guide & startup

You will need to download Ubuntu Server image. Check your chip to decide for amd or arm architeture.

[Setup UTM following this guide](https://docs.getutm.app/guides/ubuntu/).

## Virtio Shared Directory

Shared directory allow your UTM files to not get lost in the case a problem occurs and allow the edit from host.

First, set up on UTM the shared directory path. Then, run:

```
sudo mkdir -p /mnt/utmshare
sudo modprobe 9p 9pnet 9pnet_virtio
sudo mount -t 9p -o trans=virtio,version=9p2000.L,msize=262144,access=client share /mnt/utmshare
ls -la /mnt/utmshare
```

#### Mount on boot

1. `sudo nano /etc/fstab`
2. Add on last line:

`share  /mnt/utmshare  9p  trans=virtio,version=9p2000.L,msize=262144,access=client,nofail,x-systemd.automount,x-systemd.idle-timeout=30  0  0`
3. Active & test it without rebooting

```
sudo systemctl daemon-reload
sudo mount -a
ls /mnt/utmshare
```

#### Problems with permission

First, try to change on HOST the permission to read & write on macOS folder info.

If it doesn't work, maybe you need to recreate your mount. If you already created the mount, grab your id with 
```
id -u
id -g
```

then change uid & gid to use it:

```
sudo umount /mnt/utmshare
sudo mount -t 9p -o trans=virtio,version=9p2000.L,msize=262144,uid=1000,gid=1000 share /mnt/utmshare
```

OR

``
sudo mkdir -p /mnt/utmshare/my-folder
sudo chown -R "$USER":"$USER" /mnt/utmshare/my-folder
cd /mnt/utmshare
``

For git permission problem:

`fatal: detected dubious ownership in repository at '/mnt/utmshare/biomass-uav-stack'`

use:

`git config --global --add safe.directory /mnt/utmshare/biomass-uav-stack`

## Basic tools

#### Vs code

[Download deb](https://code.visualstudio.com/download#)

Example for Apple Silicon Build - ARM
```
cd ~/Downloads
sudo apt update
sudo apt install ./code_*_arm64.deb 
```

#### SSH key for git email

1. Generate key

`ssh-keygen -t ed25519 -C "my-mail@example.com"`

2. Add key to ssh-agent

``
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
``

3. Copy key & add to git platform
``cat ~/.ssh/id_ed25519.pub``

#### Docker

1. Add docker repo & add GPG key

```
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg

sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo $VERSION_CODENAME) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
```

2. Install docker engine & docker compose

`sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin`

3. Init service

```
sudo systemctl enable --now docker
sudo systemctl status docker --no-pager
```

4. Config docker to use as user, not asking for sudo

```
sudo usermod -aG docker $USER
newgrp docker
```