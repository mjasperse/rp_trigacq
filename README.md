# rp_trigacq: RedPitaya basic triggered acquisition
Uses "oscilloscope" functionality packaged in the fpga.c distributed with the ecosystem to do single-shot triggered acquisition of 16k datapoints.
Converts resulting ADC integers into floating point values using the calibration, and writes results to STDOUT in binary format for speed.
