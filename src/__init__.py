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


class log2plot:

    @staticmethod
    def Vec(size):
        '''
        Initialize an underlying std::vector<double> to be logged
        '''
        return cppyy.gbl.std.vector['double']([0 for _ in range(size)])

    @staticmethod
    def copy(src, dst):
        '''
        copy any Python type into the corresponding underlying std::vector
        does not check sizes, just assumes they match
        '''
        for i in range(dst.size()):
            dst[i] = src[i]


methods = dir(cppyy.gbl.log2plot) + ['Logger', 'ConfigManager']

for m in methods:
    if not m[0] == '_':
        setattr(log2plot, m, getattr(cppyy.gbl.log2plot, m))
