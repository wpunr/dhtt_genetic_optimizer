cmake_minimum_required(VERSION 3.8)
project(dhtt_genetic_optimizer)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dhtt REQUIRED)
find_package(dhtt_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(dhtt_genetic_plugins SHARED src/potentials/fixed_potential.cpp)
target_include_directories(dhtt_genetic_plugins PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(dhtt_genetic_plugins dhtt::dhtt_lib)
#ament_target_dependencies(dhtt_genetic_plugins dhtt dhtt_msgs pluginlib rclcpp)

pluginlib_export_plugin_description_file(dhtt plugins.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS dhtt_genetic_plugins
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_test test/test_fixed_potential.cpp)
    target_include_directories(${PROJECT_NAME}_test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${PROJECT_NAME}_test dhtt_genetic_plugins)
endif ()

# Python
# https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Python-Documentation.html
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# https://github.com/bponsler/ros2-support/blob/master/tutorials/creating-a-mixed-cpp-and-python-package.md
install(PROGRAMS scripts/run_dhtt_genetic_optimizer.py DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    set(_pytest_tests
            test/test_dhtt_genetic_optimizer.py
    )
    foreach (_test_path ${_pytest_tests})
        get_filename_component(_test_name ${_test_path} NAME_WE)
        ament_add_pytest_test(${_test_name} ${_test_path}
                APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
                TIMEOUT 60
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
    endforeach ()
endif ()

ament_package()
