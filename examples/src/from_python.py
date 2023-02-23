#!/usr/bin/env python3

from log2plot import log2plot
import os
from numpy.random import randn

logger = log2plot.Logger(os.path.dirname(__file__) + '/../')

# v is an underlying std::vector<double> that is the C++ object
v = log2plot.Vec(4)
logger.save(v, 'python', 'x_', 'x-value')
logger.setLineType('[C0, r.-, go, k--]')

for _ in range(100):

    actual_vector = randn(4)
    # we have to copy the values to v
    log2plot.copy(actual_vector, v)

    logger.update()

logger.plot()
