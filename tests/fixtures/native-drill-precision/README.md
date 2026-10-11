# Native drill precision fixture

These untouched KiCad 10.0.6 exports and exact native integer hole metadata come
from `build/native-drill-export-integration-01/native-controls-revision-02`.
The source board SHA-256 is recorded in the metadata. The six features include
plated and unplated round holes, rotated plated and unplated slots, a 0.4064 mm
via, and separate 0.600123/0.600124 mm native tools that round to the same CLI
diameter. Tests check exact tool restoration, unchanged commands and coordinates,
and rejection before writes when identity, coverage or matching is ambiguous.
