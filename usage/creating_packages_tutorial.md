Creating Packages (by Example):
```
cd catkin_ws/src
catkin_create_pkg tutorial roscpp std_msgs
cd tutorial
mkdir msg
cd msg
touch Test.msg
```

Open your favorite editor
```
Header header
int32 test_integer
```

package.xml, this is useful for updating dependencies so that your tutorial project to recognize new msgs
```
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```

CMakeLists.txt

1. add `message_generation` to find_packages
2. add the `msg` file to `add_message_files`
3. add `std_msgs` for the package to recognize the int32 type used in your Test.msg
4. at the very bottom add the executable (to be written) and libraries
  ..1. `executable(tutorial src/tutorial.cc)`
  ..2. `target_link_libraries(tutorial ${catkin_LIBRARIES})`

Create an executable file for you project called `tutorial.cc`.


Now your package should be set up. Run:
'catkin_make`



