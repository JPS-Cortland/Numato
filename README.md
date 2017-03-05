# Numato
Code for the MimasV2 development board by Numato Lab, along with the AC97 Extender Board. Most of the code was copied from
http://web.mit.edu/6.111/www/f2007/handouts/labs/lab4.html (following the link to lab4.v).
The MIT code was written for their own development board based on the Xilinx Virtex II FPGA
The most important modules are those that handle communication with the LM4550 audio IO chip - AC97 codec.
This code needed some minor adjustments to make it compatible with the MimasV2.
The "recorder" module was modified so that different filters could be written and selected using pushbuttons, etc.

In this version, all of the verilog modules appear in a single file lab4_edit.v
This is probably not the best.
My son Bobby has been working on this project, and I have been serving as an "advisor". But since both of us are new 
to Verilog and GitHub, it may take a while before things begin to actually look organized.  - John Sikora
