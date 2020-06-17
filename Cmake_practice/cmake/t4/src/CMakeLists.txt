
# 添加可执行文件main
ADD_EXECUTABLE(main main.cpp)

INCLUDE_DIRECTORIES(/usr/include/hello)

# 把共享库link到可行文件main上
TARGET_LINK_LIBRARIES(main hello)
# 链接静态库
#TARGET_LINK_LIBRARIES(main libhello.a)

