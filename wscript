#!/usr/bin/env python
# encoding: utf-8

def build(bld):
    vehicle = bld.path.name
    bld.ap_program(
        program_name='rc-fmu-ap',
        program_groups=['bin', 'rc-fmu-ap'],
        use=[vehicle + '_libs', 'ap'],
        defines=['ALLOW_DOUBLE_MATH_FUNCTIONS'],
    )
    
