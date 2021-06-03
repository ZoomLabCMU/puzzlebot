# Assemble the robot

## Electronics
The circuit schematics `puzzlebot_schematics.sch` and board layout `puzzlebot_board.brd` are included in [`files/`](/files/).  
A full list of components:
- MCU ATMega328P x1
- 8M Oscillator x1
- Resistors (0603)
    - 2 Ohm x2
    - 330 Ohm x1
    - 1k Ohm x1
    - 2k Ohm x1
    - 10k Ohm x3
- Capacitors (0603)
    - 22 pF x2
    - 0.01 uF x1
    - 0.1 uF x3
    - 2.2 uF x1
- Battery CR2 x1
- WiFi Module ESP8266 x1
- Motor Driver DRV8833 x2
- Others
    - 6 pin on-on switch x1
    - 4 pin tactile switch x1
    - 0603 LED x1
    - 4 pin female header x2
    - 6 pin male header x1
    - CR2 battery holder x1

## 3D Printing
The robot body STL file can be found at [`files/puzzlebot_chasis.STL`](/files/puzzlebot_chasis.STL). Any 3D printer can be used. We used __Prusa__ with 0.15 mm precision, with NinjaFlex Cheetah filament. Support is generated with NinjaFlex material and removed after printing.

## Assembly
<img src="/img/gears_and_rod.jpg" width="250"/>

Robot body includes: 
- 3D printed robot chasis 
- 50 mm rods x3
- 32-10 Module 0.5 gears x4
- 26-08 Module 0.5 gears x2

Assemble the above mentioned parts as shown:
<p float="left">
  <img src="/img/robot_top.jpg" height="200" /> 
  <img src="/img/robot_side.jpg" height="200" />
</p>

Attach the circuit board and tracking markers (a print of different color): 

<img src="/img/robot_body.png" width="250" />
