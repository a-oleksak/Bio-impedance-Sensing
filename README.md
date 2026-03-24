Bio-impedance-Sensing
=====================

This is the repository for NYIT Senior Design 2025-26, skin cancer diagnosis with impedance sensing <br/> 
The following is contained within the `main branch` of this repository:
- libraries needed to run arduino ide code,
- demo codes for each component and subsystem,
- draft of our overall code

Please consider visiting our [website](https://a-oleksak.github.io/Bio-impedance-Sensing/).

Compatibility
=============
This library *should* be compatible with any verison of Arduino IDE, albeit perhaps with some changes. It was developed and tested with version 2.3.7.

Installation
============
Simply move the entire contents of the folder `libraries` to your `Arduino/libraries` folder, usually in your home directory or documents folder. The contents of the folder `code` can be opened from any location with the Arduino IDE program. 
ProcessingGUI_seniorDesign.pde is a file that has to be open with Processing IDE, we will be using this application and code as serial monitor and data visualization.

Hardware
========
**Impedance Sensing Subsystem**​
* AD5933 Evaluation Board​
* Coax RG-174 (Shielded Wire) ​
* OPA376 Guard Op Amp​
* CD4053BE Multiplexer (2:1) ​

**Motor Subsystem**​
* A4988 Motor Driver​
* 1.8° Nema 17 Stepper Motor ​

Connections & Wiring
====================
<img src="https://github.com/a-oleksak/Bio-impedance-Sensing/blob/website/img/Copy%20of%20Senior%20Design%20Connections%20Diagram%20Version%204.png?raw=true" width="750" height="750" >

References
==========
A lot of code was referenced from [this AD5933 repo](https://github.com/mjmeli/arduino-ad5933) and [this stepper motor repo](https://github.com/laurb9/StepperDriver), libraries are used as provided.
