# !/usr/bin/env xonsh

import xml.etree.ElementTree as ET
from collections import defaultdict

def analyze_xml_structure(xml_file):
    """
    XML 파일의 구조를 재귀적으로 분석합니다.
    
    Args:
        xml_file (str): 분석할 XML 파일 경로
    
    Returns:
        dict: XML 구조 정보를 담은 딕셔너리
    """
    # XML 파싱
    tree = ET.parse(xml_file)
    root = root = tree.getroot()
    
    # 루트 요소 정보
    root_info = {
        'tag': root.tag,
        'attributes': set(root.attrib.keys()),
        'child_elements': set()
    }
    
    # 전체 구조를 저장할 딕셔너리
    structure_analysis = {
        'root': root_info,
        'element_hierarchy': defaultdict(dict),
        'attribute_analysis': defaultdict(set),
        'tag_statistics': defaultdict(int)
    }
    
    # 1레벨 자식 요소 분석
    first_level_tags = set()
    for child in root:
        first_level_tags.add(child.tag)
        structure_analysis['tag_statistics'][child.tag] += 1
    
    structure_analysis['first_level_elements'] = first_level_tags
    
    # 각 요소별 상세 분석
    for element_tag in first_level_tags:
        element_analysis = analyze_element_structure(root, element_tag)
        structure_analysis['element_hierarchy'][element_tag] = element_analysis
        
        # 속성 분석
        for elem in root.findall(f".//{element_tag}"):
            structure_analysis['attribute_analysis'][element_tag].update(elem.attrib.keys())
    
    return structure_analysis

def analyze_element_structure(root, element_tag, max_depth=3):
    """
    특정 요소의 구조를 분석합니다.
    
    Args:
        root: XML 루트 요소
        element_tag (str): 분석할 요소 태그
        max_depth (int): 최대 분석 깊이
    
    Returns:
        dict: 요소 구조 정보
    """
    element_info = {
        'tag': element_tag,
        'attributes': set(),
        'children': defaultdict(set),
        'child_structure': {},
        'max_depth_reached': False
    }
    
    elements = root.findall(f".//{element_tag}")
    if not elements:
        return element_info
    
    # 첫 번째 요소를 기준으로 구조 분석 (모든 요소가 동일한 구조라고 가정)
    sample_element = elements[0]
    element_info['attributes'] = set(sample_element.attrib.keys())
    
    # 재귀적으로 자식 구조 분석
    analyze_children_recursive(sample_element, element_info['child_structure'], 
                             current_depth=1, max_depth=max_depth)
    
    # 모든 요소에서 자식 태그 수집
    for elem in elements:
        for child in elem:
            element_info['children']['direct_children'].add(child.tag)
            # 모든 자손 태그 수집
            for descendant in elem.iter():
                if descendant != elem:
                    element_info['children']['all_descendants'].add(descendant.tag)
    
    return element_info

def analyze_children_recursive(element, result_dict, current_depth, max_depth):
    """
    재귀적으로 자식 요소의 구조를 분석합니다.
    
    Args:
        element: 현재 분석 중인 요소
        result_dict (dict): 결과를 저장할 딕셔너리
        current_depth (int): 현재 깊이
        max_depth (int): 최대 분석 깊이
    """
    if current_depth > max_depth:
        result_dict['max_depth_exceeded'] = True
        return
    
    children_structure = {}
    
    for child in element:
        child_tag = child.tag
        if child_tag not in children_structure:
            children_structure[child_tag] = {
                'attributes': set(child.attrib.keys()),
                'children': {}
            }
        
        # 재귀적으로 자식 분석
        analyze_children_recursive(child, children_structure[child_tag]['children'], 
                                 current_depth + 1, max_depth)
    
    result_dict.update(children_structure)

def print_analysis_results(analysis):
    """
    분석 결과를 보기 좋게 출력합니다.
    """
    print("=" * 60)
    print("XML 구조 분석 결과")
    print("=" * 60)
    
    print(f"\n루트 요소: {analysis['root']['tag']}")
    print(f"루트 속성: {analysis['root']['attributes']}")
    
    print(f"\n1레벨 요소들: {analysis['first_level_elements']}")
    
    print("\n요소별 통계:")
    for tag, count in analysis['tag_statistics'].items():
        print(f"  {tag}: {count}개")
    
    print("\n요소별 상세 구조:")
    for element_tag, element_info in analysis['element_hierarchy'].items():
        print(f"\n[{element_tag}]")
        print(f"  속성: {element_info['attributes']}")
        print(f"  직접 자식: {element_info['children']['direct_children']}")
        
        if element_info['child_structure']:
            print("  자식 구조:")
            print_child_structure(element_info['child_structure'])
    
    print("\n요소별 속성 분석:")
    for element_tag, attributes in analysis['attribute_analysis'].items():
        print(f"  {element_tag}: {attributes}")

def print_child_structure(structure, indent=2):
    """
    자식 구조를 들여쓰기로 출력합니다.
    """
    for tag, info in structure.items():
        spaces = " " * indent
        print(f"{spaces}{tag} (속성: {info['attributes']})")
        if info['children']:
            print_child_structure(info['children'], indent + 2)

# 사용 예시
if __name__ == "__main__":
    xml_file = "lanelet2_map.osm"
    
    try:
        analysis = analyze_xml_structure(xml_file)
        print_analysis_results(analysis)
        
        # 추가적인 통계 정보
        print("\n" + "=" * 60)
        print("요약 정보")
        print("=" * 60)
        total_elements = sum(analysis['tag_statistics'].values())
        print(f"총 요소 개수: {total_elements}")
        print(f"고유 태그 종류: {len(analysis['tag_statistics'])}개")
        
    except FileNotFoundError:
        print(f"파일을 찾을 수 없습니다: {xml_file}")
    except ET.ParseError as e:
        print(f"XML 파싱 오류: {e}")

tree = ET.parse(xml_file)
root = tree.getroot()

node_tags = set()
for node in root.findall("node"):
    for child in node:
        k = child.attrib['k']
        node_tags.add(k)
way_tags = set()
for way in root.findall("way"):
    for child in way:
        if child.tag == 'tag':
            k = child.attrib['k']
            way_tags.add(k)
relation_tags = set()
for relation in root.findall("relation"):
    for child in relation:
        if child.tag == 'tag':
            k = child.attrib['k']
            relation_tags.add(k)

print(node_tags)
print(way_tags)
print(relation_tags)