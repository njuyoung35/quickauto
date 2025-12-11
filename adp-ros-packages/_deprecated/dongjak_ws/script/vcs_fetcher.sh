#!/bin/bash

set -e  # 오류 발생 시 스크립트 중단

REPOS_FILE="${1:-.repos}"

if [[ ! -f "$REPOS_FILE" ]]; then
    echo "Error: .repos file not found: $REPOS_FILE"
    exit 1
fi

echo "Parsing .repos file: $REPOS_FILE"

# 임시 파일 생성
TEMP_FILE=$(mktemp)

# .repos 파일 파싱 함수
parse_repos_file() {
    local current_repo=""
    
    while IFS= read -r line || [[ -n "$line" ]]; do
        line=$(echo "$line" | sed 's/[ \t]*$//')  # trim
        
        # 빈 줄이나 주석 건너뛰기
        if [[ -z "$line" ]] || [[ "$line" == \#* ]]; then
            continue
        fi
        
        # 새 repository 시작 (key: 형식)
        if [[ "$line" =~ ^[a-zA-Z0-9_-]+: ]]; then
            current_repo="${line%:}"
            echo "Found repository: $current_repo"
            
        # url 처리
        elif [[ "$line" =~ ^[[:space:]]*url:[[:space:]]*(.+) ]] && [[ -n "$current_repo" ]]; then
            url="${BASH_REMATCH[1]}"
            echo "Found URL for $current_repo: $url"
            echo "$current_repo|url|$url" >> "$TEMP_FILE"
            
        # version 처리
        elif [[ "$line" =~ ^[[:space:]]*version:[[:space:]]*(.+) ]] && [[ -n "$current_repo" ]]; then
            version="${BASH_REMATCH[1]}"
            echo "Found version for $current_repo: $version"
            echo "$current_repo|version|$version" >> "$TEMP_FILE"
            
        # subpaths 처리
        elif [[ "$line" =~ ^[[:space:]]*-[[:space:]]*(.+) ]] && [[ -n "$current_repo" ]]; then
            subpath="${BASH_REMATCH[1]}"
            echo "Found subpath for $current_repo: $subpath"
            echo "$current_repo|subpath|$subpath" >> "$TEMP_FILE"
        fi
    done < "$REPOS_FILE"
}

# repository 클론 및 sparse checkout 함수
clone_with_sparse_checkout() {
    local repo_name="$1"
    local url="$2"
    local version="$3"
    local subpaths=("${@:4}")
    
    echo "Processing $repo_name..."
    
    if [[ -d "$repo_name" ]]; then
        echo "Directory $repo_name already exists. Skipping..."
        return
    fi
    
    # 빈 git repository 생성
    mkdir -p "$repo_name"
    cd "$repo_name"
    
    # git 초기화
    git init
    git remote add origin "$url"
    
    # sparse checkout 설정
    git config core.sparseCheckout true
    
    # sparse-checkout 파일 생성
    for subpath in "${subpaths[@]}"; do
        echo "$subpath" >> .git/info/sparse-checkout
    done
    
    # 지정된 버전 풀
    git pull origin "$version"
    
    cd ..
    echo "Successfully cloned $repo_name with sparse checkout"
}

# 메인 실행 부분
echo "Parsing .repos file..."
parse_repos_file

# 파싱된 데이터로 repository 처리
declare -A repo_urls
declare -A repo_versions
declare -A repo_subpaths

while IFS='|' read -r repo_name key value; do
    case "$key" in
        "url")
            repo_urls["$repo_name"]="$value"
            ;;
        "version")
            repo_versions["$repo_name"]="$value"
            ;;
        "subpath")
            repo_subpaths["$repo_name"]+="$value"$'\n'
            ;;
    esac
done < "$TEMP_FILE"

# 각 repository 처리
for repo_name in "${!repo_urls[@]}"; do
    url="${repo_urls[$repo_name]}"
    version="${repo_versions[$repo_name]:-main}"  # 기본값 main
    
    # subpaths 배열로 변환 - 더 간단한 방법
    filtered_subpaths=()
    if [[ -n "${repo_subpaths[$repo_name]}" ]]; then
        # IFS를 임시로 변경하여 개행으로 분리
        OLD_IFS="$IFS"
        IFS=$'\n'
        for subpath in ${repo_subpaths[$repo_name]}; do
            if [[ -n "$subpath" ]]; then
                filtered_subpaths+=("$subpath")
            fi
        done
        IFS="$OLD_IFS"
    fi
    
    if [[ ${#filtered_subpaths[@]} -eq 0 ]]; then
        echo "Warning: No subpaths found for $repo_name. Skipping..."
        continue
    fi
    
    echo "Cloning $repo_name from $url (version: $version)"
    echo "Subpaths: ${filtered_subpaths[*]}"
    
    clone_with_sparse_checkout "$repo_name" "$url" "$version" "${filtered_subpaths[@]}"
done

# 임시 파일 정리
rm -f "$TEMP_FILE"

echo "All repositories processed successfully!"