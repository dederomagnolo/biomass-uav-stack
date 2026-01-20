# UTM Mac Guide & startup

You will need to download Ubuntu Server image. Check your chip to decide for amd or arm architeture.

[Setup UTM following this guide](https://docs.getutm.app/guides/ubuntu/).

## Basic tools

#### Vs code

[Download deb](https://code.visualstudio.com/download#)

Example for Apple Silicon Build - ARM
```
cd ~/Downloads
sudo apt update
sudo apt install ./code_*_arm64.deb 
```

## Virtio Shared Directory

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


## SSH key for git, use git email

1. Generate key

`ssh-keygen -t ed25519 -C "my-mail@example.com"`

2. Add key to ssh-agent

``
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
``

3. Copy key & add to git platform
``cat ~/.ssh/id_ed25519.pub``
