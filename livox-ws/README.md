# Harpia - UFSCar - Monitoramento Florestal

Este workspace reúne aquisição do LiDAR MID-360, odometria e mapeamento com
FAST-LIO, segmentação de solo, modelagem de terreno e detecção de vegetação
com tree mapper. A ZED Mini é um sensor adicional para coleta de imagens. O sistema foi desenvolvido para rodar em em uma `Jetson Orin NX`, com 16GB de memória.

### Especificações dos periféricos

- Jetson Orion NX 16GB
- LiDAR MID-360
- Câmera de profundidade ZED-M

### Especificações do container

Arquivo docker compose: `compose.livox.yaml`

- Ubuntu 24.04
- ROS 2 Jazzy

## 1. Preparar o host

```bash
git clone https://github.com/dederomagnolo/biomass-uav-stack.git
cd biomass-uav-stack
```

### Nvidia
Para GPU NVIDIA, instalar:
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

Após instalar o toolkit, configure o runtime:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Confira JetPack/L4T e CUDA pelas ferramentas da plataforma (`cat /etc/nv_tegra_release`, `nvcc --version`); não use a ausência de `nvidia-smi` como diagnóstico de falha em Jetson.

### Obter os pacotes ROS do workspace

| Pacote | Origem / observação |
| --- | --- |
| FAST-LIO ROS 2 | [Ericsii/FAST_LIO](https://github.com/Ericsii/FAST_LIO) |
| Ground segmentation | [dfki-ric/ground_segmentation](https://github.com/dfki-ric/ground_segmentation) e [ground_segmentation_ros2](https://github.com/dfki-ric/ground_segmentation_ros2) |
| Terrain modelling | [JoaoRafaelGuimaraes/terrain_modelling](https://github.com/JoaoRafaelGuimaraes/terrain_modelling) |
| Tree mapper | [dederomagnolo/tree-mapper](https://github.com/dederomagnolo/tree-mapper); acesso privado - Débora R. Romagnolo |
| ZED ROS 2 | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper); versão deve corresponder ao SDK |

Como preparar o workspace:

```bash
mkdir -p livox-ws/src
cd src
git clone https://github.com/Ericsii/FAST_LIO.git
git clone https://github.com/dfki-ric/ground_segmentation.git
git clone https://github.com/dfki-ric/ground_segmentation_ros2.git
git clone https://github.com/JoaoRafaelGuimaraes/terrain_modelling.git
git clone https://github.com/dederomagnolo/tree-mapper.git
```

## 2. Construir o container e executar o Livox MID-360 + FAST-LIO

O [Dockerfile Livox](../ros2-docker/Dockerfile.fastlio) instala Livox SDK2 e
`livox_ros_driver2` em `/opt/livox_ws`.

### Preparar volumes, GPU e display

Antes de iniciar, revise [compose.livox.yaml](../compose.livox.yaml):

Configure o armazenamento dos bags diretamente no `compose.livox.yaml`:

- `volumes[].source`: caminho absoluto do HD ou da pasta no host, que deve existir antes de iniciar.
- `volumes[].target`: caminho do volume dentro do container (`/mnt/pendrive`).
- `environment.BAG_OUTPUT_DIRECTORY`: pasta de gravação dentro desse volume (`/mnt/pendrive/bags`).

Essas configurações não dependem de um arquivo `.env`.

### Configurar a rede e iniciar os sensores

O arquivo [MID360_config.json](config/MID360_config.json)
usa **host `192.168.1.50`** e **sensor `192.168.1.179`**. Substitua todos os
campos `*_ip` do host e o IP do sensor pelos valores reais. Confira `ip -br addr` e `ping 192.168.1.179` (ajustando o endereço). Libere no firewall os fluxos UDP configurados no JSON; `network_mode: host` compartilha a rede do host Linux.

### Iniciar pelo launcher `start.sh`

Após configurar a rede, compilar os pacotes e instalar o SDK/wrapper da ZED,
inicie o container **no host, a partir da raiz do repositório**:

```bash
docker compose -f compose.livox.yaml up -d --build
docker exec -it biomass-uav-stack-livox bash
```

O launcher carrega automaticamente `src/tree-mapper/tmux/start.yaml`, sem
assistente no terminal. Antes de iniciar, edite esse arquivo para definir o
preset, tópico, frame e recursos habilitados. Configuração para o Livox:

```yaml
preset: livox
cloud_topic: /cloud_registered
target_frame: camera_init
open_web: true
run_plotters: true
detect_shrubs: false
```

**Dentro do container**, execute:

```bash
cd /root/biomass-uav-ws/src/tree-mapper/tmux
bash start.sh
```

Para carregar outro arquivo de configuração do launcher, use
`bash start.sh --config /caminho/start.yaml`. Esse arquivo segue o formato
acima; os parâmetros dos nós continuam em `config/interface_topics.yaml`.

Se a sessão já existir, `bash start.sh` apenas se conecta a ela. Para aplicar
alterações no `start.yaml`, pare qualquer gravação e execute
`bash start.sh --restart`: isso encerra e recria os processos da sessão.
Use `--no-attach` para iniciar sem entrar no tmux.

No preset Livox, o script inicia uma sessão tmux com roteador Zenoh, ZED,
driver Livox, FAST-LIO, acumulação de pontos, segmentação de solo, terreno,
tree mapper e RViz.

Mantenha o robô parado até aparecer **`IMU Initial Done`** na janela do FAST-LIO.

Com a interface web habilitada, acesse `http://localhost:8080` no host ou
`http://<IP-da-Jetson>:8080` em outro computador da mesma rede.

Para encerrar os processos da sessão:

`Ctrl + B + K`

Ou

```bash
cd /root/biomass-uav-ws/src/tree-mapper/tmux
bash stop.sh
```

### Gravação de bags

O gravador do tree-mapper pode ser iniciado e parado pela interface web. Cada
gravação cria uma pasta `coleta_<timestamp>` no destino configurado. Pare a
gravação pela interface antes de desconectar o HD ou recriar o container.

**Configurar o destino**

Os caminhos são definidos diretamente no [compose.livox.yaml](../compose.livox.yaml),
sem depender do `.env`. Trecho relevante do serviço `biomass_livox`:

```yaml
environment:
  BAG_OUTPUT_DIRECTORY: "/mnt/pendrive/bags"
volumes:
  - type: bind
    source: "/media/laris/e50a717d-e7f4-4b57-84b5-09e9dca07507"
    target: "/mnt/pendrive"
    bind:
      create_host_path: false
```

| Campo | Função |
| --- | --- |
| `source` | Caminho do HD montado no host. Altere este campo ao trocar de disco. |
| `target` | Caminho em que o HD aparece dentro do container. |
| `BAG_OUTPUT_DIRECTORY` | Pasta dentro do container em que o gravador salva os bags. |

Nesse exemplo, `/mnt/pendrive/bags` no container corresponde a
`/media/laris/e50a717d-e7f4-4b57-84b5-09e9dca07507/bags` no host.
Se mudar o `target`, ajuste também `BAG_OUTPUT_DIRECTORY` para apontar para
uma pasta dentro dele.

Mantenha `output_directory: ""` em
`src/tree-mapper/config/interface_topics.yaml` para usar o destino do Compose.
Um valor não vazio nesse parâmetro tem prioridade sobre `BAG_OUTPUT_DIRECTORY`.
Sem ambos, o gravador usa `bags` na raiz do pacote tree-mapper; caminhos
relativos também são resolvidos a partir dessa raiz. Alterações nesse parâmetro
exigem reiniciar o processo do gravador.

**Verificar o HD no host**

Conecte e monte o HD antes de iniciar o container. No host, identifique a
partição e confira o ponto de montagem usado em `source`:

```bash
lsblk -o NAME,TRAN,SIZE,FSTYPE,LABEL,MOUNTPOINTS
findmnt --mountpoint /media/laris/e50a717d-e7f4-4b57-84b5-09e9dca07507
df -h /media/laris/e50a717d-e7f4-4b57-84b5-09e9dca07507
```

Substitua o caminho acima se usar outro disco. Confirme que `findmnt` mostra
a partição esperada e a opção `rw`. Se aparecer `ro`, o HD está montado como
somente leitura e a gravação não poderá começar. A pasta existir, por si só,
não confirma que o HD está montado. `create_host_path: false` evita criar um
caminho inexistente, mas não verifica se há um disco montado nele.

**Aplicar alterações e testar**

Alterações nos volumes ou em `BAG_OUTPUT_DIRECTORY` no Compose exigem recriar
o container; editar o arquivo não modifica o container em execução. Depois de
parar a gravação, execute no host, na raiz do repositório:

```bash
docker compose -f compose.livox.yaml up -d --force-recreate
```

Isso encerra os processos do container anterior. Inicie novamente a pilha pelo
`start.sh`, conforme a seção acima. Não é necessário reconstruir a imagem
para trocar esses caminhos.

Confira o espaço e teste a escrita pelo container:

```bash
docker exec biomass-uav-stack-livox sh -c '
  df -h /mnt/pendrive &&
  mkdir -p /mnt/pendrive/bags &&
  touch /mnt/pendrive/bags/.teste-escrita &&
  rm /mnt/pendrive/bags/.teste-escrita
'
```

Inicie uma gravação curta pela interface e pare-a. Confira se a pasta
`coleta_<timestamp>` foi criada em `bags` no HD antes de iniciar a coleta.
