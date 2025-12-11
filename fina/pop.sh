#!/bin/bash

# Python을 이용한 YAML 파싱 함수
parse_yaml_pairs() {
  python3 - << 'EOF'
import yaml
import sys
import json

try:
    with open('config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    
    sources = []
    targets = []
    
    for pair in config.get('paths', []):
        sources.append(pair.get('source', ''))
        targets.append(pair.get('target', ''))
    
    # JSON으로 출력하여 bash에서 파싱
    result = {
        'sources': sources,
        'targets': targets
    }
    print(json.dumps(result))
    
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)
EOF
}

# JSON 파싱
json_output=$(parse_yaml_pairs)

source=($(echo "$json_output" | python3 -c "import json, sys; data=json.load(sys.stdin); print(' '.join(data['sources']))"))
target=($(echo "$json_output" | python3 -c "import json, sys; data=json.load(sys.stdin); print(' '.join(data['targets']))"))

for i in "${!target[@]}"; do
  src="autoware/src/${source[$i]}"
  dst="fina_ws/src/${target[$i]}"
  
  # 1. 소스 디렉토리 존재 여부 확인
  if [ ! -d "$src" ]; then
    echo "[WARNING] Source directory '$src' does not exist. Skipping..."
    continue
  fi
  
  # 2. 타겟 디렉토리가 이미 존재하는지 확인
  if [ -d "$dst" ]; then
    echo "[INFO] Target directory '$dst' already exists."
    
    # 3. 소스의 내용만 타겟으로 복사 (재귀적 중복 방지)
    # 주의: 타겟 경로 끝에 / 가 있으면 다르게 동작함
    if [ -d "$dst/" ]; then
      echo "[INFO] Copying contents of '$dst' to '$src'..."
        
      cp -r "$dst"/* "$src"
    else
      # 타겟 디렉토리가 파일일 경우
      echo "[ERROR] '$dst' exists but is not a directory. Skipping..."
      continue
    fi
  else
    # 4. 타겟 디렉토리가 없으면 통과 (최초 실행 시)
    echo "[INFO] Target directory '$dst' does not exist. Skipping..."
    # 만약 타겟 디렉토리를 생성하고 싶다면:
    mkdir -p "$dst"
    cp -r "$src"/* "$dst"/
  fi
done

echo "[DONE] Sync completed."