cmake_minimum_required(VERSION 3.14)

project(RobotKinematics VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Quick)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Quick)

# Определяем, что платформа x86 (AVX доступен только для x86/x86_64)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i[3-6]86)$")
    set(ARCH_X86 TRUE)
else()
    set(ARCH_X86 FALSE)
endif()

set(PROJECT_SOURCES
        src/main.cpp
        src/RobotKinematics.h
        src/RobotKinematics.cpp
        qml.qrc
)

include(CheckCXXCompilerFlag)

if(ARCH_X86)
    message(STATUS "Платформа x86: проверяем поддержку AVX")

    # Проверка флага AVX
    check_cxx_compiler_flag("-mavx" COMPILER_SUPPORTS_AVX)

    if(COMPILER_SUPPORTS_AVX)
        # Указываем путь к тестовому файлу
        set(AVX_TEST_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/check_avx.cpp")
        # Пытаемся скомпилировать и запустить тестовый код с флагом -mavx
        try_run(RUN_RESULT COMPILE_RESULT
            ${CMAKE_BINARY_DIR}
            ${AVX_TEST_SOURCE}
            CMAKE_FLAGS "-DCMAKE_CXX_FLAGS=-mavx")

        if(COMPILE_RESULT EQUAL 0 AND RUN_RESULT EQUAL 0)
            message(STATUS "Runtime AVX support detected")
            add_compile_options("-mavx")
            add_compile_definitions(AVX_SUP=1)
        else()
            message(STATUS "No runtime AVX support detected")
            add_compile_definitions(AVX_SUP=0)
        endif()
    else()
        message(STATUS "Compiler does not support -mavx")
        add_compile_definitions(AVX_SUP=0)
    endif()
else()
    message(STATUS "Платформа не x86, AVX и связанные расширения отключены")
    add_compile_definitions(AVX_SUP=0)  # Отключаем AVX для других платформ
endif()

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(untitled
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET untitled APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(untitled SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(untitled
          ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(untitled
  PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.untitled)
endif()
set_target_properties(untitled PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS untitled
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_import_qml_plugins(untitled)
    qt_finalize_executable(untitled)
endif()
