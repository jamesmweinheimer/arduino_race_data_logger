# Arduino Race Data Logger

There are many data loggers on the market today but none cater to the club racer or tinkerer. Many of the features found on high-end loggers can be duplicated to the level that is useful to the amateur racer with inexpensive components. I've hacked together this assembly of off the shelf parts and code to get a reasonable amount of data with the least amount of work and expense.

The base board is an Arduino MEGA 2560 with the following components
* Adafruit Ultimate GPS
* Adafruit ADXL345
* Generic (sunfounder) SD Card module
* Adafruit i2c multiplexer
* [Melexis MLX90621 sensors] (https://www.mouser.com/ProductDetail/Melexis/MLX90621ESF-BAB-000-SP?qs=sGAEpiMZZMsMyYRRhGMFNgHcPSCzKwAOQdsKBaip0RY%3d) on [custom boards] (https://www.tindie.com/products/onehorse/mlx90621-4-x-16-ir-array/)
* [Common automotive potentiometers] (https://www.ebay.com/itm/Headlight-Level-Sensor-Toyota-Prius-Tacoma-Lexus-Mazda-RX8-89405-48020-/282219321790) for analog inputs 

Current Features
* Data logs to an SD Card in .csv file format
* GPS @ 10 Hz for
  * Time
  * Latitude
  * Longitude
  * Speed
  * Heading
* Accelerometer
* MLX90621 IR temperature 16x4 arrays for tire temperature (could also be used for brake temperature)
* Wheel position (vertical, for measure of suspension travel)
* Brake pressure (front and rear)
* Steering angle

Many, many more features can be easily added with off the shelf sensors and code.

My vision for this project is to 
1. find the best off the shelf sensors, 
2. find a good 5V power supply to keep them all happy and safe from noisy automotive 12v power environments, and 
3. integrate as many as possible into one compact board and insert it into a lightweight case for mounting into a vehicle. 
4. Separate the tire temperature sensor arrays and move them to another board so it has the hard work of parsing pixels and send only data packets back to the main board that is writing data to the SD card. The MLX90621 sensors are extremely slow (~1ms per pixel), so moving this off-board would save valuable time for writing data and reading from other sensors.

However, my time is very limited. I need help! To say my code could be optimized is an understatement.

The code has been tested for functionality with the listed components and works reasonably well. It has run for many days without hiccups.
