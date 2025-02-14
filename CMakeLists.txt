cmake_minimum_required(VERSION 3.10)

# Указываем имя проекта
project(RobotKinematics)

# Указываем, что проект использует C++ стандарт (например, C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# Добавляем флаги для AVX (учитываем поддержку разных компиляторов)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mavx -mavx2 -mfma")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(/arch:AVX2)
endif()


# Включаем автоматическое подключение MOC, UIC и RCC (для работы с Qt)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Ищем и подключаем Qt
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Quick Qml)

file(GLOB PROJECT_FILES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/*.h")
add_executable(${PROJECT_NAME} ${PROJECT_FILES}) # Создаем исполняемый файл

# Подключаем Qt к нашему проекту
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Quick
    Qt6::Qml
)

# Добавляем файл ресурсов
qt6_add_resources(RESOURCE_FILES resources.qrc)

# Включаем файлы ресурсов в проект
target_sources(${PROJECT_NAME} PRIVATE ${RESOURCE_FILES})
