# Create a library called "pid_controller" (in Linux, this library is created
# with the name of either libmyLib1.a or myLib1.so).
add_library (pid_controller
  # list of cpp source files:
  ../src/pid_controller.cpp
  )

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(pid_controller PUBLIC
  # list of directories:
  .
  )
