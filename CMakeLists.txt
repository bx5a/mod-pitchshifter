cmake_minimum_required(VERSION 3.0)

project(SuperWhammy CXX)

include(ExternalProject)

set(src ${CMAKE_CURRENT_SOURCE_DIR})

# install locally for now. A manual copy is required
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install/mod-superwhammy.lv2")

# external project needed: fftw3
set(fftw_source_dir ${CMAKE_BINARY_DIR}/fftw-source)
set(fftw_install_dir ${CMAKE_BINARY_DIR}/fftw-install)
ExternalProject_add(external_fftwf
  URL "http://www.fftw.org/fftw-3.3.5.tar.gz"
  URL_MD5 6cc08a3b9c7ee06fdd5b9eb02e06f569
  SOURCE_DIR ${fftw_source_dir}
  BINARY_DIR ${fftw_source_dir}
  INSTALL_DIR ${fftw_install_dir}
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ./configure
    --prefix=${fftw_install_dir}
    --enable-shared=no
    --enable-static=yes
    --enable-single
    CPPFLAGS=-fPIC
)
# create the harmonize.wisdom files
set(wisdom_install_dir ${CMAKE_BINARY_DIR}/wisdom-install)
file(MAKE_DIRECTORY ${wisdom_install_dir})
add_custom_target (
    generate_harmonizer_wisdom ALL
    DEPENDS ${wisdom_install_dir}/harmonizer.wisdom
 )
add_custom_command(
  OUTPUT ${wisdom_install_dir}/harmonizer.wisdom
  COMMAND ${fftw_install_dir}/bin/fftwf-wisdom
  ARGS -n -x -o ${wisdom_install_dir}/harmonizer.wisdom rof1024 rob1024 rof1536 rob1536 rof2048 rob2048 rof2176 rob2176 rof2304 rob2304 rof2432 rob2432 rof2560 rob2560 rof3072 rob3072 rof4096 rob4096
  DEPENDS external_fftwf
)

link_directories(${fftw_install_dir}/lib)
set(fftw_libraries libfftw3f.a)
set(fftw_include_dir ${fftw_install_dir}/include)

# external project needed: armadillo
set(armadillo_source_dir ${CMAKE_BINARY_DIR}/armadillo-source)
set(armadillo_install_dir ${CMAKE_BINARY_DIR}/armadillo-install)
ExternalProject_add(external_armadillo
  URL "http://sourceforge.net/projects/arma/files/armadillo-7.500.1.tar.xz"
  URL_MD5 65961c62a740e502f5b249e7e908bf93
  SOURCE_DIR ${armadillo_source_dir}
  BINARY_DIR ${armadillo_source_dir}
  INSTALL_DIR ${armadillo_install_dir}
  UPDATE_COMMAND ""
  CMAKE_ARGS
    "-DCMAKE_INSTALL_PREFIX=${armadillo_install_dir}"
    "-DBUILD_SHARED_LIBS=OFF"
)
link_directories(${armadillo_install_dir}/lib)
set(armadillo_libraries libarmadillo.a)
set(armadillo_include_dir ${armadillo_install_dir}/include)

# external project needed: lv2 plugin framework
set(lv2_source_dir ${CMAKE_BINARY_DIR}/lv2-source)
set(lv2_install_dir ${CMAKE_BINARY_DIR}/lv2-install)
ExternalProject_add(external_lv2
  URL "http://lv2plug.in/spec/lv2-1.14.0.tar.bz2"
  URL_MD5 0b56285a77aac3c93b50c36203aa91ee
  SOURCE_DIR ${lv2_source_dir}
  BINARY_DIR ${lv2_source_dir}
  INSTALL_DIR ${lv2_install_dir}
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ./waf configure --lv2dir=${lv2_install_dir}/lv2 --prefix=${lv2_install_dir}
  BUILD_COMMAND ./waf
  INSTALL_COMMAND ./waf install
)
set(lv2_include_dir ${lv2_install_dir}/include)

# project definition
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -Wall -fPIC -DPIC")
set(shared_sources
  ${src}/Shared_files/angle.cpp
  ${src}/Shared_files/angle.h
  ${src}/Shared_files/Exp.cpp
  ${src}/Shared_files/Exp.h
  ${src}/Shared_files/GainClass.cpp
  ${src}/Shared_files/GainClass.h
  ${src}/Shared_files/HarmonizerLib.cpp
  ${src}/Shared_files/HarmonizerLib.h
  ${src}/Shared_files/PitchDetectionClass.cpp
  ${src}/Shared_files/PitchDetectionClass.h
  ${src}/Shared_files/PitchShifterClasses.cpp
  ${src}/Shared_files/PitchShifterClasses.h
  ${src}/Shared_files/window.cpp
  ${src}/Shared_files/window.h

  ${src}/Freezer/fft.cpp
  ${src}/Freezer/fft.h
  ${src}/Freezer/freeze.cpp
  ${src}/Freezer/freeze.h
)
add_library(SuperWhammy
  SHARED
  ${src}/SuperWhammy/src/SuperWhammy.cpp
  ${shared_sources}
)
add_dependencies(SuperWhammy
  external_fftwf
  external_armadillo
  external_lv2
  generate_harmonizer_wisdom
)
target_include_directories(SuperWhammy
  PRIVATE
    ${src}/Shared_files
    ${fftw_include_dir}
    ${armadillo_include_dir}
    ${lv2_include_dir}
)
target_link_libraries(SuperWhammy
  ${fftw_libraries}
  ${armadillo_libraries}
)

set_target_properties(SuperWhammy
  PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    OUTPUT_NAME "mod-superwhammy"
)

# install commands
set(ttl_sources
  ${src}/SuperWhammy/ttl/manifest.ttl
  ${src}/SuperWhammy/ttl/SuperWhammy.ttl
  ${wisdom_install_dir}/harmonizer.wisdom
)
install(FILES ${ttl_sources} DESTINATION ".")
install(DIRECTORY ${src}/SuperWhammy/ttl/modgui DESTINATION ".")
install(TARGETS SuperWhammy LIBRARY DESTINATION ".")
