import os
from ament_index_python.packages import get_package_share_directory

package_name = "ur_description"
relative_path = "meshes/ur5e/collision/shoulder.stl"

# 패키지의 공유 디렉토리 경로 찾기
package_path = get_package_share_directory(package_name)

# 전체 파일 경로 구성
mesh_file_path = os.path.join(package_path, relative_path)

print(mesh_file_path)  # 실제 파일 경로 출력
