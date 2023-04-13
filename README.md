# Real Time Programing : a 4th year project from the GEI department at INSA Toulouse 

## Sources 
This repository contains code from <em> https://github.com/INSA-GEI/dumber.git </em>, branch <em> evoxx-dumber-v3 </em>. <br>
Modifications have been made on <strong>software/raspberry/superviseur-robot/tasks.h</strong> and <strong>software/raspberry/superviseur-robot/tasks.cpp</strong> files. <br>
To use said project <em> (free software)</em>, use <strong> git clone https://github.com/INSA-GEI/dumber.git -b evoxx-dumber-v3 </strong> <br>
Commands for using the software are described on the following page : <br>
<em> https://moodle.insa-toulouse.fr/pluginfile.php/142046/mod_resource/content/1/guide.pdf </em>

## Features
Features for the behavior of a robot's supervisor have been implemented such as described on the following page : <br>
<em> https://moodle.insa-toulouse.fr/pluginfile.php/142045/mod_resource/content/1/sujets_robot.pdf </em> <br>
## See a non exhaustive list of the proposed features just below :
<ul>
<li><strong>Features 5 & 6 : </strong>The supervisor must detect communication errors with the monitor, inform the user and put every element in their initial state.</li>
<li><strong>Features 8 & 9 :</strong>Communication between the robot and the supervisor must be monitored to detect a communication loss on the medium. <br> If 3 consecutive messages are lost or return errors, communication must be closed and the supervisor must be put back to its initial state</li>
<li><strong>Feature 11 :</strong>A watchdog can be enabled or disabled before the robot is started. <br> If enabled, it must be reloaded every second to detect communication errors.</li>
<li><strong>Feature 13 :</strong>Robot's battery level is returned and showed to the user every 500 ms. </li>
<li><strong>Features 14, 15 & 16 :</strong>Camera can be continuously :<br>
  - Opened, which starts periodic image acquisition <br>
  - Closed, which stops periodic image acquisition </li>
<li><strong>Features 17 : </strong>If an arena exists on the last acquired image, it is outlined. User can decide whether or not they accept it. <br>
If accepted, the outline is kept on future images acquired. <br>
If rejected, the outline is discarded and image acquisition continues normally.</li>
<li><strong>Features 18 & 19 : </strong>Enables or disables position acquisition on images <em> (see feature 15)</em>.</li>
</ul>
