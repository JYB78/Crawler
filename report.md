# Quest 6
Authors: Cindy Bai, Jess Barry, Zach Carlson

2019-12-10

## Summary
![screenshot](./images/simple.png)
Above is a high-level flow chart of our quest 6 implementation. We have four different modes for the crawler: STOP(0), AUTO(1), MANUAL FORWARD(2), MANUAL BACKWARD(3). More detailed flow charts are included in the Sketches and Photos section below.


During our demo, we were able to demonstrate all criteria listed on rubric except for when doing the second turn. (A more detailed list of things we have successfully demonstrated is shown below.) We ended up fixing the error a few minutes after our demo and was in line for a re-do, but unfortunately we did not get to re-do the demo since we had to leave for senior design. So, we asked for a beacon and returned to the demo room later that day and we successfully completed the entire course around BOTH corners. We took a video of our crawler going around the room and that video clip is included in our team video linked below.

Here is a list of things that we have demonstrated working during our demo:
- Crawler stop/go in reaction to beacon IR signal
- Current split time recorded when seeing a "R" signal from beacon
- Most recent split time shown on the Alpha Display on top of Crawler
- Drive straight lines along the wall in Auto mode
- Go around (first) 90 degree corner without collision in Auto mode
- All split times sent to server are recorded into a level database (log also shown on the web client)
- Live streaming from webcam visible from web client
- Allow both automatic and manual switches between modes
- Remote control in manual mode: forward, backward, steering, change speed
- Able to automatically decode QR code after driving the crawler to the front of the QR through manual control from web client
- Server, web client, and database hosted remotely on RPi

We have alsoe implemented some functionalities that are not required by the objective criteria of the quest:
- Can go both forward AND backward when in Manual mode (mode 3 or 4)
- PID wheelSpeed control implemented in BOTH Manual & Auto Mode
- User speed control for both moving forward AND backward: 0.1/0.3/0.5 m/sec
- Displays total time passed since the first starting green light (1st beacon) on web client
- Able to switch between Auto & Manual Mode from web client



## Evaluation Criteria



## Solution Design

## Server & Web Client
### Server
#### Task 1: UDP Socket to ESP32
A UDP socket is created to communication with ESP32. Data received will be saved to a file named "data.csv". This file contains all data transferred from the ESP32 except fo the split time since the split time will be recorded into the levelDB.
The "data.csv" file contains:
- Side distance
- Front distance
- Current steering angle
- Current wheel speed
- Total time (in sec) since the start of the crawler running
- Current Mode
** A sample "data.csv file is included inside the code folder"

#### Task 2: Host Web Client
The second task of the server is to host the web client (index.html) and handle different POST requests for corresponding buttons. Once a button action is received by the server, it will let the ESP32 know through the UDP socket created in task 1, and the ESP32 will handle these messages accordingly.


#### Task 3: Level Database for recording split times
The last task of the server is to store the split time received into the levelDB. We parse and look at each message sent by the ESP to the server, and if the message contains the current split time ("curSplit"), it will be stored into the levelDB. Additionally, we confirms that the new split time is recorded correctly by doing a readDB() which creates a read stream of the database and prints everything store inside the database into the console.


### Web Client
![screenshot](./images/web1.png)
Here is a screenshot of our web client. It displays everything that we need to know when driving the crawler.

The manual speed control is done with selecting the desired speed and clicking on the enter button next to it. The options we have for the speed are: 0.1, 0.3, and 0.5. We did not set the Manual speed options to be too high since we still want to drive the crawler under control.

The Left and Right buttons works do that it +/- 15 degrees to the current steering angle when clicked once. We do have checking mechanisms to take sure that the angle will not go out of bound (max angle (i.e. max left) = 90 deg && min angle (i.e. max right) = 0 deg) The center button will bring the steering servo back to center at 45 degree.

Current mode is also displayed on the web client. Options includes: "Stop", "Automatic", "Manual Forward", "Manual Backward".

The front and side distance readings are also displayed on the web client when in Auto mode.

The web client communicates with the server through POST requests. It uses a POST request to get the content of the data.csv file returned from the server, and parse the data stored and update what's being displayed on the screen. When a button is clicked, it also sends a POST request to the server with the corresponding pathname as argument. The server will then distinguish the different POST requests and send corresponding messages to the ESP32.

#### Webcam & QR Decoding
The webcam and QR decoding are also implemeted inside the web client (i.e. at browser level). We were using jsQR for QR scanning, and we can simply embed the webcam view using an <img> tag with class="video". jsQR works by basically feeding in the current frame of the video every-so-often (using setInterval function), and checks for QR code in the captured frame. We implemented so that it feeds in a new frame every second, and when a QR code is found, it decodes it and print the message onto the web client. The image below is a screenshot of the web client with the QR decoded and displayed:
![screenshot](./images/QR.png)

## Crawler (ESP32)

### Wheel Speed Control
In general, we are using a few key global variables for overall speed control: speed, setWheel(Speed), mywheelSpeed.
We are using PID to control the speed in BOTH Auto and Manual mode. The "dt" (i.e. the timer interval to call PID function) for PID is 1 second, meaning that the global speed variable will be adjusted by comparing the "setWheel" & "mywheelSpeed" variable. Since we have can have our crawler going backwards, we have to make sure that we are add the output of PID to the current speed when going backward (Mode 3: manual_backward) and subtracting when going forward (Mode 1: auto && Mode 2: manual_forward).
#### Auto Mode:
The set wheel speed (setWheel) in Auto mode is 0.1 meters per second since we wanted the crawler to move slowly to give time for sensor and IR readings.
#### Manual Mode:
The set wheel speed (setWheel) in Manual mode is 0.1 by default (both forward and backward). User can change the setWheel to 0.1, 0.3, or 0.5 m/sec through selecting in the dropdown list on the web client.

### Steering
(Max Left: 90 deg, Center/Straight: 45 deg, MAX Right: 0 deg)
#### Auto Mode:
There are mainly two components that controls the steering in Auto mode:
1. (Right) Side microLIDAR Sensor - Straight

    We mounted a microLIDAR on the right side of the crawler. This part is the exact same thing as what we had for Quest 4. The goal is to adjust the steering angle accordingly so that the crawler maintain a certain set distance away from a wall to its right. We have the "setPoint_side" (set-point side distance) to be 75 cm since the beacons will be place 75 cm away from the wall. We are using the "P" part of the PID of adjust the steering angle and the variables "Kp" is 1.

2. Front LIDAR Sensor - Turns (90 deg to the left)

    This is one of the new feature that we have added for this Quest. Before in Quest 4, we used the front range sensor to detect collisions. Now, we are using it to make turns. The goal is to turn the crawler 90 degrees to the left when it sees a front obstacle such as a wall. We implemented this functionality so that a "turn" flag is switched on when the front sensor gets a reading smaller than 2.5 m. We have a function called "myTurn()" which is responsible for controlling the steering when making turns; this function will maintain max left (90 deg) for 7.5 second and restore regular steering control (with the side microLIDAR sensor). These turning distance and time for turning are obtained through continuous experimenting and calibration.

Note: both front and side distance readings are sent to the server and displayed on the web client.

#### Manual Mode:
In Manual mode, the steering control is pretty straight forward where it will be controlled by user input from the web client. By default, the steering will reset to center (45 deg) when switching from Auto to Manual mode. There are Left and Right buttons on the web client. When a user clicks the Left/Right button, it will +/- 15 deg from the current steering (mySteering) to generate the new steering. The current steering angle is also displayed on the web client in real time. We also have out-of-bound checks so that the user input will always be in the valid range (90 - 0 deg).


### Traffic Light Beacons & Split Time
#### IR Receiver
The IR receiver continuously reads in data and iterates through each byte. It checks to see if the the reading contains "R" or "G" and will stop and move forward accordingly. We have global variable called "totalTime", which is the total time (in sec) passed since the first "G" signal; this variable is also displayed on the web client. This variable is managed by a timer interrupt function through the use of a flag. We designed our IR receiver function so that it stops when it sees a "R" and record the current totalTime as the curSplit (most current split time) since ideally we want to be able to stop at all three beacons on the way. The string format of the new curSplit ("str_split", converted to string in Alpha Display function) is then send to the server and recorded into the levelDB.

#### Alpha Display
The curSplit is split up into two variables representing seconds and minutes, then both are turned into strings using itoa(). The string form of the curSplit is in the formate of "min:sec", and this string is stored as a global variable called "str_split"; this string is what is send to the server through the UDP socket and store in the database since it is easier to read the time this way. Then based on the values of those strings, the indexes are set depending on the amount of digits each variable has. Once the indexes are assigned, each display buffer value is assigned based on the index into the alphafont table.

### Timer
The timer is used in keeping track of the totalTime and manages the PID function to adjust wheelSpeed as mentioned above. The timer is done with a timer interrupt and a global flag (inside the app_main()).


### UDP Socket to Server (RPi)
A UDP socket is created using the IP of the RPi (Port 3333) for the ESP/crawler to communicate with the server hosted on the RPi. We can obtain the IP of the RPi from the router's device list page since all components in this quest are connected to wifi from the group router.
#### Sending
Messages sent are all in the form of "name,value" with on space. This unified formate makes it easier for the server and web client to parse and understand the messages once they are received. Here is the list of all things (name) sent to the server:
- mode
- wheelSpeed
- totalTime
- steering
- curSplit
- side
- front

#### Receiving
In addition, the crawler also needs to be able to receive and react to user control from the web client in Manual mode. When a button is pressed, the server will send corresponding messages to esp. Here is the list of all possible messages and what they mean:
- F = Manual Forward Mode
- B = Manual Backward Mode
- Stop = Stop Mode
- A = Auto Mode
- L = Left Steering (mySteering += 15)
- R = Right Steering (mySteering -= 15)
- C = Center Steering (mySteering = 45)
- s1/s2/s3 = speed option 1/2/3 (0.1, 0.3, 0,5 m/s)

## Sketches and Photos
### Detailed Flow Chart
![screenshot](./images/sketch_server.png)
![screenshot](./images/sketch_crawler.png)

### Our Crawler
![screenshot](./images/side.png)
![screenshot](./images/front.png)
![screenshot](./images/back.png)

## Supporting Artifacts
- [Link to repo]()
- [Link to video demo]()
Our demo video was shot after the testing set up was disconnected. Since our demo in class was unable to show the crawler successfully making it around the second corners of the track, we filmed our crawler completing the track with one beacon at the beginning. We also included our QR code successfully being read and a short demo of our crawler working wirelessly because our time allowed.


## References

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
