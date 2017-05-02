# Scout Car for Automatic Mapping

Scout car for automatic mapping is a bot which can explore and map an area automatically.

## Requirements

### Hardware
  * Arduino WeMos D1 R2 (any arduino with WiFi shield works but pin mapping and arduino code will change)
  * 12V battery
  * 2 DC motors + wheels
  * L298N motor driver
  * Wheel encoder with IR sensor 
  * IR proximity sensor 
  * MPU 9250 9-dof imu sensor
  * Chassis to mount the components
  
### Software
  * Arduino IDE - https://www.arduino.cc/en/main/software
  * Optional - Microsoft Visual Studio + Visual Micro - https://www.visualstudio.com/downloads/ http://www.visualmicro.com/page/Arduino-Visual-Studio-Downloads.aspx
  * Python 3.x (tested with 3.6) - https://www.python.org/downloads/
  * socket, parse, numpy, matplotlib libraries
    
Socket is installed by default in most python distributions. For other libraries, use the command 

    pip install <library_name>
    
## Getting Started

First, connect the componenets according to the below diagram

![Ciruit](Circuit.png?raw=true)

Note that the battery should be 12V. The red component on the top right is MPU9250 and the component below it is the IR sensor. The pin numbers on the WeMos might be different from what is given in the diagram. Physically the connections look the same (Ex.: the wire connected to the second pin from the top in the diagram will be connected to the second pin from the top in WeMos)

Once the hardware connections are done, open the [ino file](V1_1_MoveInGivenDirectionWemos/V1_1_MoveInGivenDirectionWemos/V1_1_MoveInGivenDirectionWemos.ino) (if you are using visual studio, open the [visual studio solution](V1_1_MoveInGivenDirectionWemos/V1_1_MoveInGivenDirectionWemos.sln)). Build and upload to the WeMos board (see this [tutorial](http://www.instructables.com/id/Programming-the-WeMos-Using-Arduino-SoftwareIDE/) if you haven't programmed WeMos before).

Now, connect to WeMos wifi network from your PC and run the [main python script](PCSide/main.py). If everything works fine, you should be seeing the bot moving around and a map being generated on the PC. After exploration, the map will be saved in the same folder ([Example](PCSide/map.png)). If there is some problem, see the troubleshooting section.

## Troubleshooting

1. **No WiFi network of WeMos**: Check if the bot is on and LED of the WiFi shield in WeMos is glowing. If not, reset WeMos and check again. If it still doesn't work, then try uploading the [WeMos Web Server](TempCodes/Wemos_WebServer) project and check if it works. If it fails then there is some problem with the WiFi shield.

2. **The bot continuously rotates in one place or it doesn't move in the right direction**: The magnetometer is most probably not calibrated properly. Upload the [Calibration](TempCodes/MPU9250_Yaw) project and open the serial monitor. Reset WeMos and move the bot in shape of 8 (like during compass calibration in phones) or just rotate around all 3 axes (*Note that the calibration needs to be done away from strong magnets and when the magnetometer is mounted on the bot*). After a while, the yaw values will be displayed in the serial monitor. Check if they are correct. If so, copy the bias, scale values to the main ino file (into magBias and magScale). If not, reset and try again. 

    In case the calibration fails after multiple attempts, print mx, my, mz values from magnetometer and copy them to an excel sheet. Find min, max value of mx, my, and mz. Find bias = (max + min)/2 and scale = (max - min)/2 for x, y, and z. 
    Calculate x' = (x - biasX)/scaleX, and similarly y' and z'. Scatter plot x', y', z' pairwise and check if they are points in the interior of a unit circle. If so, then use the bias, scale values. Else, repeat the calibration.
    
    In the unlikely case that the yaw values are not stable, change the values of Kp, Ki in MahonyQuaternionUpdate function till it gives stable values quickly and then do the calibration.
    
3. **The bot stops (and detects wall) even if the wall is far**: Check the trigger range of IR proximity sensor and set it such that the bot has enough space to rotate after detecting the wall. IR proximity sensor is known to give bad readings in the presence of sunlight. So it is better to block sunlight during mapping. If you need to bot to map outdoor environment, then a different sensor should be used to detect walls.

4. **The bot moves very near/ very far at a time**: Set the CELL_DIS in [maparray.py](PCSide/maparray.py) higher/lower.

5. **The bot doesn't map the full area**: Set the MAP_SIZE to a higher value.
    
4. **The bot moves very slowly/ one of the wheels doesn't turn**: The battery is getting discharged. Replace/recharge them. 

5. **UDP connection times out**: It may happen due to multiple reasons. First, open the serial monitor and reset WeMos to see whether it is creating a WiFi hotspot. Also, check if the IP address of WeMos is same as the one written in [communication.py](PCSide/communication.py) and run again. Second, replace/recharge the battery. Third, in [main.py](PCSide/main.py), increase the delay such that the bot is able to move and stop before a new UDP request is sent by the PC (5 seconds is generally enough) and run again. If neither of the methods work, then the reason is unknown.

## Known issues

1. WeMos doesn't reset when it is connected to the encoder (through D3 pin). A temporary fix is to remove the connection, reset the board and connect the encoder afterwards.

2. Connection between PC and bot times out sometimes. The timeout is extremely rare when the battery is fully charged and there are no other strong WiFi networks in the surrounding. Other observations were - 
    * Timeouts are less frequent when the delay between two consecutive requests by the PC is high
    * More timeouts when an UDP packet reaches arduino while the motor is moving
    * No timeouts when only UDP communication is taking place (no sensors/motor)
    * No timeouts when encoder is removed (not fully tested - might just be coincidence)

3. There are oscillations when the bot rotates in place. This can be solved by turning the bot slowly so that there is no overshoot.

4. The bot can deviate from its path due to incorrect rotation/movement.

5. If the bot deviates a bit and wrongly detects a wall (physically) in the cell (of the map) which it had previously detected as open, then the bot gets stuck there and the mapping doesn't continue.

6. If the proximity sensor fails to detect a wall (due to not being in the line of sight) and the bot crashes into it, it can't recover and continue mapping.

## Future work

1. **Solving existing issues**: Using a motor with finer speed control solves the first issue. The second issue is solved partially by using encoders in addition to magnetometer for rotation and is fully solved if the PC code is able to detect wrong movement and correct it. The third issue won't occur if the bot movement is correct. Fourth issue can be solved by adding more sensors around the bot (it still can't recover from physical crashes)

2. **Reducing power consumption**: Right now, the exact power usage is not calculated and the hardware/code is not optimized for minimum power consumption. It will be necessary since the bot should be able to map the required area in a single charge.

3. **Mapping/Exploration which doesn't depend on accurate movement**: The idea of this bot may be used for applications like cleaning the house where it may not be possible to move the bot accurately due to its size. Also, the resolution of the map will be bad if the bot is assumed to be smaller than or equal to one cell size. Hence, the bot needs to be able to explore/map in arbitrary directions.

4. **Dynamic updation of map**: Right now, the bot assumes the area to map as static and generates a map. But in reality, there may be obstacles which are moved around and hence, the map needs to be changed dynamically.
