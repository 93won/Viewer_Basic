# Make library

add_library(graphOptimizer SHARED
Config.cpp
Viewer.cpp
Utils.cpp
)
target_link_libraries(graphOptimizer ${THIRD_PARTY_LIBS})
