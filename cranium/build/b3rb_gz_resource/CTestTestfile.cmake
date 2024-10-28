# CMake generated Testfile for 
# Source directory: /home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource
# Build directory: /home/vyom/cognipilot/cranium/build/b3rb_gz_resource
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/test_results/b3rb_gz_resource/copyright.xunit.xml" "--package-name" "b3rb_gz_resource" "--output-file" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/ament_copyright/copyright.txt" "--command" "/opt/ros/humble/bin/ament_copyright" "--xunit-file" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/test_results/b3rb_gz_resource/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "200" WORKING_DIRECTORY "/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_copyright.cmake;51;ament_add_test;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;22;ament_copyright;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource/CMakeLists.txt;22;ament_package;/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/test_results/b3rb_gz_resource/lint_cmake.xunit.xml" "--package-name" "b3rb_gz_resource" "--output-file" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/test_results/b3rb_gz_resource/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource/CMakeLists.txt;22;ament_package;/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/test_results/b3rb_gz_resource/xmllint.xunit.xml" "--package-name" "b3rb_gz_resource" "--output-file" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/vyom/cognipilot/cranium/build/b3rb_gz_resource/test_results/b3rb_gz_resource/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource/CMakeLists.txt;22;ament_package;/home/vyom/cognipilot/cranium/src/b3rb_simulator/b3rb_gz_resource/CMakeLists.txt;0;")
