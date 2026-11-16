These are actual source extracts, added for the CRC32 and fragment-planner
equivalence regression. They are not captured device data or synthetic frames.

`baseline-functions.c.txt` contains the four unmodified functions from the
production source at `a656d53f2efc447507f5247a116a22aed35dcc04`. The full source
and extracted bytes are identified in `provenance.json`. Tests read this file
directly, without requiring a historical Git object.

`esp-idf-v5.5.5-crc32-le.c.txt` preserves Espressif's CRC32 LE table/function
and license from the local ESP-IDF software reference. It supplies the host
ROM-API boundary and is compared with the independent old bitwise code and
real encoder golden bytes. Running it is not execution of the ESP32-S3 ROM
and does not measure target timing.

Run `python3 tools/test_ble_crc_planner.py`. Generated encoder outputs stay in
a temporary directory; the existing scientific golden frames are only read.
