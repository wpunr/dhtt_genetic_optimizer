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

add_library(${PROJECT_NAME} SHARED src/dhtt_genetic_optimizer.cpp src/potentials/fixed_potential.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} dhtt::dhtt_lib ${pluginlib_TARGETS})
#ament_target_dependencies(${PROJECT_NAME} dhtt dhtt_msgs pluginlib rclcpp)

# first argument is the package with the base class
pluginlib_export_plugin_description_file(dhtt plugins.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_test test/test_fixed_potential.cpp)
    target_include_directories(${PROJECT_NAME}_test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif ()

# Python
# https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Python-Documentation.html
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# https://github.com/bponsler/ros2-support/blob/master/tutorials/creating-a-mixed-cpp-and-python-package.md
install(PROGRAMS scripts/run_dhtt_genetic_optimizer.py dhtt_genetic_optimizer/eval.py DESTINATION lib/${PROJECT_NAME})

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
