# EVAM 1 Firmware

> Source code for the EVAM 1's CAN bus firmware

<p align="center">
<img src="https://user-images.githubusercontent.com/6837599/138331854-9dfa00b7-021b-4b2a-a48f-78b1f74af131.png" width="20%" \>
</p>


- [ ] 🏎️ Automotive Hardware
- [x] 🛞 Automotive Software
- [ ] 🖥️ Supporting Software and Simulations
- [ ] 📚 Code Library
- [ ] 🛠 Helper Script
- [ ] 📖 Documentation / Datasheets
- [ ] ❓ Miscellaneous / Uncategorized

This repository contains the source code for the EVAM 1's custom CAN bus nodes.


## Directories

* `CANBus 2/Nodes` - Arduino programs for each subsystem
    * `/TPS/TPS_code` - Throttle Position Sensor
    * `/BMS/BMS_code` - Battery Management System
    * `/Wheels` - All code related to propulsion
        * `ECU_code` - Engine Control Unit
        * `FW_code` - Front Wheels, corresponding to MCU_Double in the electronics design
        * `RLW_code` - Rear Left Wheel, corresponding to MCU_Single in the electronics design
        * `RRW_code` - Rear Right Wheel, corresponding to MCU_Single in the electronics design
    * `/LED/RL_Code` - Rear lights code (not implemented in actual vehicle)
    * `/SAS/SAS_code` - Steering Angle Sensor, originally meant for torque vectoring. Currently not used
    * `/HUD/HUD_code` - Heads Up Display / Dashboard
* `CANBus 2/Test Sketches` - Original test programs for Arduino programs
* `Notes` - Documentation used during this project


> Refer to `CANBus 2/CAN Bus Messages.xlsx` for specific information about each node. This document provides detailed information on the CAN messages, message IDs, descriptions, senders, receivers, and message formats.


## Documentation

Please refer to EVAM internal OneDrive for further documentation, including specific datasheets on the electrical system components


## Caveats

No known caveats yet


## Known issues

No known issues yet


## Contributing

If you encounter any issues with this repository, please do not hesitate to open an issue.

