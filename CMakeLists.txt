project(enet_lwip)
cmake_minimum_required(VERSION 2.8)

# set cross compilation information
set(CMAKE_SYSTEM_NAME CodeSourcery)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(PROJECT_SOURCE_DIR "/home/alex/Projects/UC-projects/CM-MUX-EHO/mux")

# specify the toolchain
set(TOOLCHAIN_PATH   /home/alex/CodeSourcery)
set(TOOLCHAIN_PREFIX ${TOOLCHAIN_PATH}/bin/arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}as)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)

enable_language(ASM)

# set compiler flags
set(CPU "-mcpu=cortex-m3")
set(FPU " ")
set(CMAKE_ASM_FLAGS "-mthumb ${CPU} ${FPU} -MD")
set(CMAKE_C_FLAGS "-mthumb ${CPU} ${FPU} -std=gnu99 -ffunction-sections -fdata-sections -MD -Wall")
set(CMAKE_CXX_FLAGS "-mthumb ${CPU} ${FPU} -ffunction-sections -fdata-sections -MD -Wall -std=c++11")


# set linker flags
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "-nostartfiles -T${PROJECT_SOURCE_DIR}/enet_lwip.ld  -specs=${PROJECT_SOURCE_DIR}/lm3.specs")

# add processor specific definitions
add_definitions(-DGCC_ARMCM3_LM3S9B95)
add_definitions(-Dsprintf=usprintf)
add_definitions(-Dsnprintf=usnprintf)
add_definitions(-Dprintf=uipprintf)

add_definitions(-DTARGET_IS_TEMPEST_RB1)
add_definitions(-DPART=LM3S9B95)
add_definitions(-Dgcc)

include_directories(
    "${TOOLCHAIN_PATH}/arm-none-eabi/include"
    ${PROJECT_SOURCE_DIR}
    "inc"
    "enet_lwip"
    "third_party"
    "driverlib"
    "third_party/lwip-1.3.1/src/include"
    "third_party/lwip-1.3.1/src/include/ipv4"
    "third_party/lwip-1.3.1/apps/httpserver_raw"
    "third_party/lwip-1.3.1/ports/stellaris/include"
    "third_party/lwip-1.3.1/apps"
    "utils"
     )

# add source files to the project
aux_source_directory(./enet_lwip SRC_LIST)
aux_source_directory(./utils SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})


# add linked library to the project
target_link_libraries(${PROJECT_NAME}
    ${TOOLCHAIN_PATH}/arm-none-eabi/lib/thumb/libc.a
    ${PROJECT_SOURCE_DIR}/driverlib/gcc/libdriver.a
)

# define objcopy macro
macro(OBJCOPY_FILE EXE_NAME)
    set(FO ${CMAKE_CURRENT_BINARY_DIR}/${EXE_NAME}.bin)
    set(FI ${CMAKE_CURRENT_BINARY_DIR}/${EXE_NAME})
    message(STATUS ${FO})
    add_custom_command(
        OUTPUT ${FO}
        COMMAND ${CMAKE_OBJCOPY}
        ARGS -O binary ${FI} ${FO}
        DEPENDS ${FI}
    )
    get_filename_component(TGT "${EXE_NAME}" NAME)
    add_custom_target("target-objcopy_${TGT}" ALL DEPENDS ${FO} VERBATIM)
    get_directory_property(extra_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    set_directory_properties(
        PROPERTIES
        ADDITIONAL_MAKE_CLEAN_FILES "${extra_clean_files};${FO}"
    )
    set_source_files_properties("${FO}" PROPERTIES GENERATED TRUE)
endmacro(OBJCOPY_FILE)

# set the objcopy for binary file
objcopy_file(${PROJECT_NAME})
