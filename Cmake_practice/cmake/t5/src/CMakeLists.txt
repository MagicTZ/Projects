
ADD_EXECUTABLE(curltest main.cpp)

INCLUDE_DIRECTORIES(/usr/include)
TARGET_LINK_LIBRARIES(curltest curl)

