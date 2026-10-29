#!/usr/bin/env bash
# Read-only discovery: print the mounted USB paths for manual YAML editing.
set -euo pipefail
case "${1:-}" in
  --help|-h)
    echo 'Uso: ./scripts/verify-usb.sh [UUID]'
    echo 'Mostra os caminhos do disco montado para copiar no YAML. Não altera nada.'
    exit 0 ;;
  --*) echo 'Opção inválida. Use --help.' >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo 'Use --help.' >&2; exit 2; }
command -v findmnt >/dev/null || { echo 'findmnt não encontrado.' >&2; exit 1; }
command -v python3 >/dev/null || { echo 'python3 não encontrado.' >&2; exit 1; }
python3 - "${1:-e50a717d-e7f4-4b57-84b5-09e9dca07507}" <<'PY'
import json
import subprocess
import sys

uuid = sys.argv[1]
result = subprocess.run(
    ['findmnt', '--json', '--list', '--output', 'SOURCE,TARGET,UUID,OPTIONS'],
    capture_output=True, text=True, check=True,
)
mounts = [mount for mount in json.loads(result.stdout).get('filesystems', [])
          if mount.get('uuid') == uuid]
if not mounts:
    sys.exit(f'Disco com UUID {uuid} não está montado. Monte-o no host e execute novamente.')
for mount in mounts:
    print(f'# UUID: {uuid}')
    print(f'# Dispositivo atual: {mount["source"]}')
    print('# compose.livox.yaml — volume do pendrive:')
    print('- type: bind')
    print(f'  source: {json.dumps(mount["target"], ensure_ascii=False)}')
    print('  target: /mnt/pendrive')
    print('  bind:')
    print('    create_host_path: false')
    print()
print('# interface_topics.yaml — field_bag_recorder.ros__parameters:')
print('output_directory: /mnt/pendrive/bags')
print('# interface_topics.yaml — tree_snapshot_exporter.ros__parameters:')
print('snapshot_mirror_dir: /mnt/pendrive/snapshots')
if any('ro' in (mount.get('options') or '').split(',') for mount in mounts):
    sys.exit('ATENÇÃO: montagem somente leitura; os caminhos acima não garantem gravação.')
print('\n# Caminhos identificados; nenhum teste de escrita foi realizado.')
PY
