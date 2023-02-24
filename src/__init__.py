#!/usr/bin/env python3

import cppyy
import os
# install_path/lib/pythonX.Y/dist-packages/log2plot/__init__.py
install_path = os.path.abspath(__file__)
for _ in range(5):
    install_path = os.path.dirname(install_path)

cppyy.add_include_path(install_path + '/include')
cppyy.include('log2plot/logger.h')
cppyy.include('log2plot/config_manager.h')
cppyy.load_library(install_path + '/lib/liblog2plot.so')

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
