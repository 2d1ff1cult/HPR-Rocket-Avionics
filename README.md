# HPR Rocket Avionics
Code for data acquisition electronics on a Tripoli Level 1 High Power Rocket (HPR)

Model as of 4/2/2024:
As of 4/2/2024, this is the first iteration. Currently just collects data and ~~stores on SD card~~ displays to ground station serial monitor.
Uses nRF24L01+ 2.4GHz modules for radio communication.

Features the uses of remove before flight (RBF) tags connected to SPDT switches in a custom CAD assembly to actuate GPS and logging first, then the Arduino Nano.
![handdrawn schematic](https://github.com/2d1ff1cult/HPR-Rocket-Avionics/assets/48054365/8c7cd668-5859-4eaa-9e12-67d637941a6b)

Why? GPS cold starts are a big issue; starting the GPS at least 15 minutes before the entire data logging circuit allows for satellite acquisition as you drive to rocket launch destination.

Sample data looks like this:
![sampledata](https://github.com/2d1ff1cult/HPR-Rocket-Avionics/assets/48054365/d7054caf-d038-4269-9d08-bafcae685749)


Second iteration will be an upgrade to 900MHz with RYLR998 and data logging. Possibly even control loops for fin actuation.

Future goals involve creating an OSD visualization of the data (which are altitude, air pressure, speed, XYZ accel, and GPS) similar to SpaceX launches. This is what inspired this work
![Screenshot 2024-04-02 210506](https://github.com/2d1ff1cult/HPR-Rocket-Avionics/assets/48054365/badb42e5-d1ed-4bc6-a198-2d41835992ec)
