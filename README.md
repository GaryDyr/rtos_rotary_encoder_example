<p align="center"><b>FreeRTOS Sketch for ESP32 with Rotary Encoder</b></p>

<b><p align="center">April 1, 2022</p></b>

The sketch, *rtos\_rotary\_encoder.ino,* is an example of using RTOS to
monitor a rotary encoder module, and acquire data based on the encoder
settings. The rotary encoder is the typical five pin (Vcc, GND, SW, DT,
CLK) module with both rotary and button press functions. One reason for
publishing the sketch is that I was unable to find any clear example of
using RTOS with a rotary encoder, relative to an ESP32/Arduino
environment. There are many great examples of how to use RTOS with a
button and an Arduino or ESP module, but not with a rotary encoder.

This document is not written as a beginner tutorial for FreeRTOS. This
discussion only will make sense, if already experienced with RTOS,
or after reading and viewing the FreeRTOS documents and examples, and
online tutorials.

Though possible to write code using RTOS to natively deal with the
rotary encoder, that was not the path taken here. Instead, two existing
libraries for a rotary encoder, *ESPRotary* and *Button2*, provide the
encoder status to an encoder RTOS task. The original libraries and
examples should be consulted for details on how these libraries are
used. With my ESP32S modules, even on a solderless breadboard, both
libraries have shown good debounce behavior (with only minor tweaking of
the task delay time) to have them reliably (\> ~90% of the time)
register the expected change state.

The sketch is based on the concept that a button press signals a user's
desire to change to some specific operation mode, for example, take
sensor data, stop data collection, calibrate a sensor, or send data to
the Serial Monitor and/or the cloud. With the exception of stopping data
acquisition, rotor position indicates the desired operation to run,. The
current rotor position is printed, but only two changes are recognized
by the code. A positive change from position "0" to "1" and a negative
change to "-1". (Note that according to the way ESPRotary operates, a
CCW change produces a value of "255" which is converted to "-1".) The
desired operation is initiated by momentarily clicking the rotary
encoder's button. At the end of each procedure, the rotor state is reset
to "0", regardless of the original position of the rotor. For example,
initially clicking the button without changing the rotor position,
starts the data acquisition and reading tasks. Clicking again, suspends
data acquisition. If the rotor is rotated CCW by one detent, then the
button clicked, the position is printed and the Serial Monitor will
print "Doing something like a calibration", and finally exits back to a
wait state.

The sketch is a minimal example, simply laying the groundwork for more
advanced functioning. Because of the nature of RTOS, the advantage is
that multiple tasks can look like near parallel monitoring and
processing. There are three tasks defined: *EncoderTask, DataTask,* and
*DataEaterTask* and two queues: *EncoderQueue* and *DataQueue*. Both
ESP32 cores are engaged. EncoderTask runs on Core 0 and the remaining
tasks on Core 1. The queues are based on C++ *struct* definitions; one
structure, *Encoder\_s*, amounts to a state object for the encoder. The
other structure, *Data\_t*, has a time and data element pair as floats.
There is also code built in that uses the ESP32 onboard LED to signal
whether a signal has been received and what stage a task may be at. This
acts as a self-contained visual monitoring system, without the need for
any hardwired output display of any kind, if desired. (This was
particularly useful in the scale streaming repository for monitoring
weight changes, where no output display was connected.)

*EncoderTask* monitors the changes to the encoder rotor or button, based
on two standard ESPRotary functions, *r.loop()* and *b.loop()*, which
poll the encoder module. In turn, these two functions have callbacks
defined at the tail end of the sketch that are triggered by changes, and
dictate how to dispose of the action. Especially important is the
*click()* function. which is the button click callback; this calls the
*onButtonClick()* function that handles what tasks are running.

*DataTask* task is a simple task that generates time and data pairs to
mimic sensor input. A time value is generated by monitoring the
milliseconds difference from the start of a button click and the data
value is a simple random number call as an example case that fills the
elements of a *Data\_t TX\_Data\_t* struct. (Note that this random call
is likely not as random as you might expect, because no seed is applied,
but serves for the example.) The *TX\_Data\_t* elements are *stime* and
*wt.* The use of "*wt*" suggests that this could be part of an endeavor
to continuously read load cell values.

Filling the struct is where a sensor would usually be called in for
useful work. Within the typical infinite loop of the task is an if
statement that provides a maximum limit to the maximum amount of data
allowed, and suspends the data sending and data receiving task once the
limit is reached. This conditional can serve as an output limit so that
memory, or other storage limit, such as an SD card file limit is not
breached.

The data object is sent to a two item queue send handler, *DataQueue*,
i.e., two *TX\_data\_t* struct objects (not separate elements). The
*DataEaterTask* takes the first in data from the send queue and dumps it
to a queue receive handler struct, *Data\_t recieveData*. The time/data
pair is printed to the Serial Monitor. It is within this task or other
additional tasks that the data would be displayed, stored, or sent
somewhere. (A *Data\_t RX\_Data\_t* struct is also defined, but unused.

Under typical start up conditions of an ESP32, the data acquisition and
display rate to the Serial Monitor is slightly over 1 ms/read, or
approaching 1 kHz. (I kHz is the standard tick rate for ESP32 FreeRTOS
on an ESP32). However, the queues are set to *portMAXDELAY* , which
means infinite delay until data shows up in the queue. In some cases,
this may help avoid struggling with timing issues between sensors and
final output.

There are a couple of references to functions commented out that are not
included in this sketch. They can be found in the main sketch of the
load cell streaming repository. However, that sketch does not use RTOS,
but works fine without it. Obviously, one impetus for the current work
was curiosity as to the applicability of RTOS to continuously monitor
load cell data. Despite the positive result, the conversion was not
done. Instead, it is likely the framework here will be used as a
starting point for other sensor monitoring.

There are quite a few comments in the sketch, which may help decipher
some processes. The *onButtonClick()* function may still require a bit
of time to understand it's relationship with the tasks, and the
callbacks from the encoder libraries.

For reference, the following pins for a 38 pin ESP32S module ("KeeYees
Development Board, 2.4 GHz Dual Core WLAN WiFi + Bluetooth 2-in-1
Microcontroller ESP-WROOM-32 Chip CP2102") were used for the rotary
encoder using the Node32S board Arduino IDE driver:

| ROTARY\_ENCODER\_CLK\_PIN    | GPIO32 |
| ---------------------------- | ------ |
| ROTARY\_ENCODER\_DT\_PIN     | GPIO35 |
| ROTARY\_ENCODER\_BUTTON\_PIN | GPIO34 |
| Encoder VCC                  | 3.3V   |
| Encoder GND                  | GND    |

There is nothing special about the pin assignments, other than
convenience. Note that the KeeYees module was narrower than other
Chinese versions, so that the a row of holes on each side of the
solderless breadboard were exposed, instead of the usual single set of
holes being exposed on a single breadboard.

Because only two tasks directly process data, no semaphores were needed
to control access.