#!/usr/bin/env python3

import cppyy

include_directories = "${INCLUDE_DEPENDS}"
install_path = "${CMAKE_INSTALL_PREFIX}"

for directory in include_directories.split(';'):
    if directory.startswith('/'):
        cppyy.add_include_path(directory)

cppyy.include('log2plot/logger.h')
cppyy.include('log2plot/config_manager.h')
cppyy.load_library(f'{install_path}/lib/liblog2plot.so')

# put methods / classes from log2plot C++ namespace into this one
for m in dir(cppyy.gbl.log2plot) + ['Logger', 'ConfigManager']:
    if not m[0] == '_':
        globals()[m] = getattr(cppyy.gbl.log2plot, m)

del m, install_path

# Wrap std::vector<double>
Vec = cppyy.gbl.std.vector['double']


def copy(src, dst: Vec):
    '''
    copy any Python type into the corresponding underlying std::vector
    does not check sizes, just assumes they match
    '''
    for i in range(dst.size()):
        dst[i] = src[i]
