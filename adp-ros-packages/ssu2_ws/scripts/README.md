# scripts

## dependencies

```bash
sudo apt install xonsh
pip install datamodel-code-generator
```

## process_universe.xsh

git 의존성 복사, 인터페이스 패키지 생성, 일반 패키지 골격 (package.xml, CMakeLists.txt, README.md) 생성 등을 수행하는 스크립트입니다.

[xonsh](https://github.com/xonsh/xonsh)라는 python-powered shell을 필요로 합니다.

### 읽는 것

워크스페이스 경로에 있는~

- universe.yaml : 모든 패키지의 정보를 담고 있는 yaml 파일
- interface_universe.yaml : 인터페이스 패키지의 정보를 담고 있는 yaml 파일
- repos_universe.yaml : git clone할 저장소의 정보를 담고 있는 yaml 파일
- meta_config.yaml : author, license, cmake_version, rust_edition 등의 정보를 담고 있는 yaml 파일

### 만드는 것

~를 읽어서 저장소를 복사하고, 인터페이스 패키지를 생성하고, 그 외 노드 및 알고리즘 및 런치 관련 패키지들을 생성해줍니다.

- pkg:type = repo
- pkg:type = interface
- pkg:type = node
- pkg:type = algorithm
- pkg:type = launch

## schemas_to_pydantics.xsh

### 읽는 것

워크스페이스 경로에 있는~

- universe.schema.yaml : universe.yaml의 jsonschema 파일
- meta_config.schema.yaml : meta_config.yaml의 jsonschema 파일

### 만드는 것

~를 읽어서 pydantic 클래스를 명세한 파이썬 파일로 변환해줍니다. 이 때 [datamodel-code-generator](https://github.com/koxudaxi/datamodel-code-generator)를 사용합니다.

- universe.py
- meta_config.py

~이것들은 별도 수정이 이뤄지지 않은, 순수 codegen 생성 파일들입니다.

process_universe.xsh는 이 pydantic 클래스들을 참조하므로, 이 .xsh는 최초 실행하고 가능한 기존 것은 바꾸지 않는 게 좋습니다. 물론 패키지 명세에 추가할 것을 추가하는 것은 문제가 덜 됩니다.

참고로 저 .schema.yaml들을 참고해서, universe.yaml 등 파일을 작성할 때 어떤 구조로 해야 하는지 참고하실 수 있습니다. interface_universe.yaml, repos_universe.yaml도 모두 universe.schema.yaml을 따르며, 오직 meta_config.yaml만 meta_config.schema.yaml을 따릅니다.

