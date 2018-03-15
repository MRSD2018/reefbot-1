Setup proces for commencing work with Reefbot (camera yet to be integrated0:

1. clone this repository "reefbot-1": https://github.com/MRSD2018/reefbot-1.git to your catkin workspace
Forked from: https://github.com/mkiefer1/reefbot-1.git Thanks Max.
2. Next build your catkin workspace "catkin_make"
If your ros catkin workspace is setup and all prior dependencies resolved this shoud be 100% success.
We will revisit the software setup after setup of the physical robot. 
4. You need the control box, and the necessary power and data cable (harness) to the control box (AC supply from the wall)
and the harness [data + power] from the control box to the robot. 
5. Plug in and power the control box. The indicator on the wifi router should glow.
6. Engage the e-stop in the control box. 
7. Next connect the [robot to controller harness].
5. Release e-stop
6. Did the LEDs (internal) on the robot blink?
Did the head lights on the robot blink?
Did the robot greet you (tune)?

If yes, your setup is read?
If not, -- refer to the debug guide.

Continuing with the software setup process:
7. Connect to "reefbot" wifi.
To check the connected devices login as admin w/ "password" @ 192.168.1.1
You might not notice the robot as a connected device - but FYI the robot IP is 192.168.1.1
8. roscore
9. migrate to the reefbot-contrller/bin directory in reefbot-1 repo.
10. execute test_joy python file "./test_joy.py"
10. rosrun joy joy_node
