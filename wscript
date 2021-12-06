#!/usr/bin/env python
# encoding: utf-8

def build(bld):
    vehicle = bld.path.name
    bld.ap_program(
        program_name='rc-fmu-ap',
        program_groups=['bin', 'rc-fmu-ap'],
        use=[vehicle + '_libs', 'ap'],
        source = bld.path.ant_glob(
            [ 'src/*.cpp',
              'src/calibration/*.cpp',
              'src/comms/*.cpp',
              'src/control/*.cpp',
              'src/gimbal/*.cpp',
              'src/guidance/*.cpp',
              'src/nav/*.cpp',
              'src/sensors/*.cpp',
              'src/state/*.cpp',
              'src/util/*.cpp' ]
        ),
        includes = 'src',
        defines=['ALLOW_DOUBLE_MATH_FUNCTIONS'],
    )
