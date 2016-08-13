# KomaRoof

An Arduino sketch for controlling a vnh5019 motor shield for a roll-on/off roof. The sketch includes a serial interface using [NMEA commands](http://www.hhhh.org/wiml/proj/nmeaxor.html).

The roof consists a roof that moves back and forth on a set of rails and a lock functionality using a linear actuator.

## Command set

### TEST

Queries the board name and version number. This message is also sent automatically when the serial port is opened.

Example: `$TEST*16`  
Response: `$KOMAROOF,VER=1.0*63`

### STATUS

Queries the current status of the roof.

Example: `$STATUS*14`  
Response: `$STATUS,ROOF=OPEN,LOCK=OPEN,L1=OFF,L2=OFF,POS=99*74`

`ROOF` - Status of the roof. The valid states are `OPEN`, `CLOSED`, `OPENING`, `CLOSING` and `ERROR`.  
`LOCK` - Status of the lock. Valid states are `OPEN`, `CLOSED`, `OPENING`, `CLOSING` and `ERROR`.  
`L1`, `L2` - Limit switches, `ON` or `OFF`  
`POS` - Position of the roof, from 0 to 100.

### OPEN

Unlocks the lock and opens the roof.

Example: `$OPEN*14`  
Response: `$OPEN,OK*3C`  

### CLOSE

Closes the roof and locks the lock.

Example: `$CLOSE*56`  
Response: `$CLOSE,OK*7E`  

### STOP

Stops the roof if it is moving.

Example: `$STOP*18`  
Response: `$STOP,OK*30`  
