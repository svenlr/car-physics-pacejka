find_package (Python3 COMPONENTS Interpreter Development NumPy)
file(MAKE_DIRECTORY $ENV{HOME}/.local/include)
if (NOT EXISTS "$ENV{HOME}/.local/include/matplotlibcpp.h")
    file(DOWNLOAD
            https://raw.githubusercontent.com/lava/matplotlib-cpp/ef0383f1315d32e0156335e10b82e90b334f6d9f/matplotlibcpp.h
            $ENV{HOME}/.local/include/matplotlibcpp.h
            )
endif ()
set(matplotlib_INCLUDE_DIRS $ENV{HOME}/.local/include ${Python3_INCLUDE_DIRS})
set(matplotlib_LIBRARIES ${Python3_LIBRARIES})
