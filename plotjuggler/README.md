# PlotJuggler plugin

This folder contains a plugin for [PlotJuggler](https://github.com/facontidavide/PlotJuggler) to load YAML files generated from log2plot.

You need to have PlotJuggler installed. A good choice for the installation folder is `~/.local/share/PlotJuggler` as it a default plugin folder for PlotJuggler:

```
 mkdir build && cd build
 cmake .. -DCMAKE_INSTALL_PREFIX=~/.local/share/PlotJuggler
 make install
```

# YAML file format

Only a part of what is exported from `log2plot` is loaded by PlotJuggler, namely:

- `dataType` (`iteration-based`, `time-based`, `XY` or `3D pose`) to get the representation
- `data` that is a list of list of double (e.g. a matrix)
- `legend` that should have the correct size with regards to the data rows:
    - same as a row for `iteration-based`
    - size of a row + 1 for `time-based` (begins by the time stamp)
    - half the size of a row for `XY`, as a row is (x1,y1,x2,y2,etc.)
    - size 1 for `3D pose` as a single 3D trajectory is represented through 6 values in thie case
