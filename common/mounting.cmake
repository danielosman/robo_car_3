# Turns common/mounting.json into a C header with the robot's mounting:
#   static const mounting_t ROBOT_MOUNTING = {...}; (the type: common/mounting.h)
# Checks every field; a missing or wrong one stops the configure with its name.
# Included by common/CMakeLists.txt; also runs alone (run_tests.sh):
#   cmake -DMOUNTING_JSON=common/mounting.json -DMOUNTING_H=build/host/robot_mounting.h -P common/mounting.cmake
if(CMAKE_VERSION VERSION_LESS 3.19)
    message(FATAL_ERROR "the mounting needs CMake 3.19 or newer (string(JSON))")
endif()

# A number at the path (e.g. tof z_m) as a C float, into `out`.
function(_mounting_number out json file)
    string(JSON type ERROR_VARIABLE err TYPE "${json}" ${ARGN})
    string(REPLACE ";" "." name "${ARGN}")
    if(err MATCHES "not found")
        message(FATAL_ERROR "${file}: ${name} is missing")
    elseif(err)
        message(FATAL_ERROR "${file}: ${err}")
    elseif(NOT type STREQUAL "NUMBER")
        message(FATAL_ERROR "${file}: ${name} must be a number, not ${type}")
    endif()
    string(JSON value GET "${json}" ${ARGN})
    set(${out} "${value}" PARENT_SCOPE)
endfunction()

function(_mounting_float out json file)
    _mounting_number(value "${json}" "${file}" ${ARGN})
    if(NOT value MATCHES "[.eE]")
        string(APPEND value ".0")
    endif()
    set(${out} "${value}f" PARENT_SCOPE)
endfunction()

function(_mounting_int out json file)
    _mounting_number(value "${json}" "${file}" ${ARGN})
    string(REPLACE ";" "." name "${ARGN}")
    if(NOT value MATCHES "^-?[0-9]+$")
        message(FATAL_ERROR "${file}: ${name} must be a whole number, not ${value}")
    endif()
    set(${out} "${value}" PARENT_SCOPE)
endfunction()

# Writes `header` from `json_file`; the file is only touched when it changes.
function(mounting_header json_file header)
    file(READ "${json_file}" json)
    set(f "${json_file}")
    foreach(k x_m y_m z_m zone_deg)
        _mounting_float(tof_${k} "${json}" "${f}" tof ${k})
    endforeach()
    _mounting_int(tof_turn "${json}" "${f}" tof zones_turn_deg)
    if(NOT tof_turn MATCHES "^(0|90|180|270)$")
        message(FATAL_ERROR "${f}: tof.zones_turn_deg must be 0, 90, 180 or 270, not ${tof_turn}")
    endif()
    foreach(k x_m y_m z_m f_px tilt_deg)
        _mounting_float(cam_${k} "${json}" "${f}" camera ${k})
    endforeach()
    foreach(k width_px height_px)
        _mounting_int(cam_${k} "${json}" "${f}" camera ${k})
    endforeach()
    foreach(pair tof_z_m:tof.z_m tof_zone_deg:tof.zone_deg cam_z_m:camera.z_m cam_f_px:camera.f_px
                 cam_width_px:camera.width_px cam_height_px:camera.height_px)
        string(REPLACE ":" ";" pair "${pair}")
        list(GET pair 0 var)
        list(GET pair 1 name)
        string(REGEX REPLACE "f$" "" v "${${var}}")
        if(NOT v GREATER 0)
            message(FATAL_ERROR "${f}: ${name} must be more than 0, not ${v}")
        endif()
    endforeach()
    set(rows "")
    foreach(r 0 1 2)
        set(row "")
        foreach(c 0 1 2)
            _mounting_int(v "${json}" "${f}" imu robot_from_sensor ${r} ${c})
            if(NOT v MATCHES "^(-1|0|1)$")
                message(FATAL_ERROR "${f}: imu.robot_from_sensor.${r}.${c} must be -1, 0 or 1, not ${v}")
            endif()
            list(APPEND row "${v}")
        endforeach()
        list(JOIN row ", " row)
        list(APPEND rows "{${row}}")
    endforeach()
    list(JOIN rows ", " imu)

    set(text "// Made by common/mounting.cmake from common/mounting.json: edit that file, not this one.
#pragma once
#include \"mounting.h\"

static const mounting_t ROBOT_MOUNTING = {
    .tof = {.x_m = ${tof_x_m}, .y_m = ${tof_y_m}, .z_m = ${tof_z_m}, .zone_deg = ${tof_zone_deg}, .zones_turn_deg = ${tof_turn}},
    .camera = {.x_m = ${cam_x_m}, .y_m = ${cam_y_m}, .z_m = ${cam_z_m}, .f_px = ${cam_f_px}, .tilt_deg = ${cam_tilt_deg},
               .width_px = ${cam_width_px}, .height_px = ${cam_height_px}},
    .imu = {.robot_from_sensor = {${imu}}},
};
")
    set(old "")
    if(EXISTS "${header}")
        file(READ "${header}" old)
    endif()
    if(NOT old STREQUAL text)
        file(WRITE "${header}" "${text}")
    endif()
endfunction()

if(CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
    if(NOT MOUNTING_JSON OR NOT MOUNTING_H)
        message(FATAL_ERROR "usage: cmake -DMOUNTING_JSON=<json> -DMOUNTING_H=<header> -P mounting.cmake")
    endif()
    mounting_header("${MOUNTING_JSON}" "${MOUNTING_H}")
endif()
