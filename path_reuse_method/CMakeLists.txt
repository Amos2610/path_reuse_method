cmake_minimum_required(VERSION 3.10)
project(path_reuse_method)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(nlohmann_json REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(path_reuse_method_interfaces REQUIRED)
ament_python_install_package(${PROJECT_NAME})

include_directories(include)
ament_export_include_directories(include)


# path_seed_server.cpp
add_executable(path_seed_server src/path_seed_server.cpp)
ament_target_dependencies(path_seed_server
  # 必要な依存関係を追加
  rclcpp 
  path_reuse_method_interfaces
)
# # 生成された型をインクルードできるように
# rosidl_target_interfaces(path_seed_server
# ${PROJECT_NAME} "rosidl_typesupport_cpp")
# install(TARGETS path_seed_server
#   DESTINATION lib/${PROJECT_NAME}
# )

# # decorder.cpp
# add_library(decoder STATIC
#   src/decoder.cpp
# )
# ament_target_dependencies(decoder rclcpp)
# target_link_libraries(decoder Eigen3::Eigen nlohmann_json::nlohmann_json)
# target_include_directories(decoder PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(path_seed_server decoder) # path_seed_serverにdecoderをリンク

# dummy.cpp
add_library(${PROJECT_NAME}_lib SHARED
  src/dummy.cpp
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs)

install(PROGRAMS
    path_reuse_method/path_seed_client.py
    path_reuse_method/decode_server.py
    path_reuse_method/encode_server.py
    test/test_xarm6_move_by_pr_method.py
    path_reuse_method/generate_path_seed.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS path_seed_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# launch/ をインストール
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()