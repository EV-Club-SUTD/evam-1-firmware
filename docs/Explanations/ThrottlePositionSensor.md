# Throttle Position Sensor (TPS)

This article aims to explain the software design of the Throttle Position Sensor (TPS) subsystem in the EVAM 1.0 CANBus 1 and 2 implementation.

[[_TOC_]]

## Background

The throttle position sensor is an essential part of the electric vehicle, that senses the position of the throttle pedal and transmits that data to the rest of the CAN bus nodes. 

It is a safety critical component with a recommended rating of ASIL-B at the minimum.

## General System Requirements

### System Requirements

> This is where we put what the system is supposed to look like, what it is expected to do in the software. This section does NOT cover the software itself, just the requirements

### System Interfaces

> This describes how it interfaces with other parts. In our example of the TPS, it is fairly straightforward, there's only the voltage supply, CAN bus, and analog voltage converter that's connected to the pedal

## Implementation

### Overall system description

> This describes the current implementation in software 

### Function references

|Function Name |Input Parameters |Outputs |Description |
|:--- |:--- |:--- |:--- |
|productCode|`parameter1: string`|a string containing...|Code of the document product to return the schema for. <br> <ul><li>Here is a bulleted list with a \| (pipe) inside a table.</li><li>Another bulleted list.<ul><li>An indented list</li></ul></li><li>Back to the list.</li></ul> |
|||||

### Known Bugs

> This section contains all the known bugs spotted in the code

## Improvements and future plans

> What can be fixed/improved in the future (e.g. for CANBus 3.0)

- General System Requirements (what the system is SUPPOSED to look like)
    - System Requirements: what it’s expected to do
    - System Interfaces: how this interfaces with other parts
- Implementation Details (what the system CURRENTLY looks like)
    - Overall system description
    - Function descriptions (on a per function basis): what each function does, the input and output variables
    - Known bugs
- Improvement plan (what should be fixed in the future)
