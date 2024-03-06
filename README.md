# Role-Playing with Robot Characters: Increasing User Engagement through Narrative and Gameplay Agency

![cover](https://github.com/SeboLab/role-playing-robots/blob/main/images/cover.png)

This repository contains software to run the experiment and data analysis for the paper
"[Role-Playing with Robot Characters: Increasing User Engagement through Narrative and Gameplay Agency](https://hri.cs.uchicago.edu/publications/HRI_2024_Ng_Role_Playing_Robots.pdf)",
published at the [19th Annual ACM/IEEE International Conference on Human-Robot Interaction](https://humanrobotinteraction.org/2024/)
(HRI 2024).

Created by [Spencer Ng](mailto:spencerng@uchicago.edu), [Ting-Han Lin](mailto:tinghan@uchicago.edu), [You Li](mailto:youli21@uchicago.edu),
and [Sarah Sebo](mailto:sarahsebo@uchicago.edu) at the [Human-Robot Interaction Lab](https://hri.cs.uchicago.edu/) at
the University of Chicago.

## System Architecture & Repository Structure

![architecture](https://github.com/SeboLab/role-playing-robots/blob/main/images/architecture.png)

The system to run the experiment consists of the following hardware and software components:

1. **Server desktop**: Linux PC inside the study room to control the robots and output to the monitor
    1. **ROS backend**: [Robotics Operating System](https://www.ros.org/) nodes that pass messages to each other. Configuration
       files are at `launch/`, `CMakeLists.txt`, and `package.xml`.
       1. **[Vosk node](https://github.com/alphacep/ros-vosk)** for real-time speech recognition of keywords from the researcher
       2. **Flask server** (`src/rprobots/server.py`) to receive commands from the Wizard of Oz controller via HTTP
       3. **Main controller** (`src/rprobots/main.py`) to translate HTTP and voice commands to robot and OBS outputs for each scene
       4. **[Vector](https://github.com/SeboLab/vector-robot) & [Misty](https://github.com/SeboLab/misty) ROS controllers**, in-house
          wrappers around the robots' native SDKs for an easier programming interface
    2. **[Open Broadcaster Software (OBS)](https://obsproject.com/) & Websocket connection (`src/rprobots/obs_websocket.py`)** to 
       programatically control the monitor's display as scenes switch
2. **Experimenter laptop**: any laptop connected to the same network as the server and able to send HTTP requests to the server's port
   (e.g., via a VPN subnet). The following is run on the experimenter laptop:
    1. **PyQt client** (`src/qtwizard/`) for Wizard of Oz control
    2. **OBS** to monitor participants and record study videos
    3. **Qualtrics survey** opened in a browser. The laptop is given to users at the end of the study to use during the post-study survey.
3. **Input/output devices**
   1. **[Misty](https://www.mistyrobotics.com/misty-ii) & [Vector](https://ddlbots.com/products/vector-robot) robots**,
      connected with their native apps to the same network as the server desktop
   2. **Microphone** for processing speech commands
   3. **Webcam** with a microphone, wired to the experimenter laptop for processing
   4. **Monitor** for displaying graphics that go along with the experience

The repository also contains the following folders:

* `data-analysis/`: scripts for analyzing study results in R, along with our anonymized study data
* `assets/`: experimenter script and physical study materials given to participants (e.g., code sheet).
   Assets displayed on the monitor can be downloaded from [this Google Drive link](https://drive.google.com/file/d/1HNctbRHQTjh-vYoUYDNJodCpxUL633qv/view?usp=sharing).

## Experiment setup

The hardware components described above are physically wired and placed as follows:

![setup](https://github.com/SeboLab/role-playing-robots/blob/main/images/setup.png)

### Robot setup

1. Set up the Misty and Vector robots in the study room, connecting them to a Wi-Fi network that 
   is accessible in the room using their respective apps. If needed, set up a router for Wi-Fi access.
2. Position the robots between the monitor, across from the participant's chair as shown above. Both 
   robots should be on their charging bases, with the bases plugged in.

### Hardware setup

1. Connect the desktop PC to power and the same network (either via Ethernet or Wi-Fi) as the two robots.
2. Connect the PC to the monitor via a display cable, ensuring that the monitor also has power.
3. Connect a microphone, mouse, and keyboard to the PC.

### Server desktop setup

The desktop server directly controls the robots and the OBS instance on the monitor. The following setup
enables you to run the server on a new Linux computer:

1. Install [ROS Noetic from this source](http://wiki.ros.org/ROS/Installation)
2. [Create a catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
3. Clone this repository to the `src` folder of your Catkin workspace.

#### Open Broadcaster Software

1. Download [OBS](https://obsproject.com/download).
2. Install [OBS Websocket 4.9-compat](https://github.com/obsproject/obs-websocket/releases).
3. Set up the websocket with **Tools > WebSocket Server Settings (4.9-compat)**, then set the port to 
   `4444` and password to `AgentJay`.
4. Download the media assets ZIP through [this Google Drive link](https://drive.google.com/file/d/1HNctbRHQTjh-vYoUYDNJodCpxUL633qv/view?usp=sharing), then unzip to a `media/` folder at the root of this repository.
5. In OBS, go to **Scene Collection > Import**, then select `RPR_OBS.json` in this repository as the 
   Collection Path. Click **Import**.
6. Switch the scene collection by going to **Scene Collection > Role Playing Robots Main**.
7. A prompt should alert you to missing files in OBS. Click **Search Directory...**, then select the 
   `media/` folder you extracted. Click **Apply**. A background with the HRDA logo should appear after
   a few seconds.

#### ROS nodes

1. Clone the [Vector Robot ROS wrapper](https://github.com/SeboLab/vector-robot) and 
   the [Misty Robot ROS wrapper](https://github.com/SeboLab/misty) to the `src` folder of your Catkin 
   workspace. Complete the steps in the "Usage" sections to install the `anki_vector_ros` and `misty_ros`
   packages and configure Vector's connnection.
2. Clone the [ROS vosk package](https://github.com/alphacep/ros-vosk) into `src` to enable offline
   voice recognition.
3. Build this package from the Catkin workspace root via `catkin_make`
4. Modify the serial number in `launch/rprobots.launch` to Vector's serial number
5. Modify the IP address in `launch/rprobots.launch` to Misty's IP address (found via the Misty app)
6. Install requirements with `pip3 install -r requirements.txt`
7. Run setup from the `src` directory: `sudo python3 setup.py install`
8. Install [Tailscale](https://tailscale.com/kb/installation/) (or an equivalent subnet solution)
   and login using your Google account. Start the connection by running `tailscale up` in the terminal.
9. Copy the IP of the PC server by running `tailscale ip --4` in the terminal. Replace the `SERVER_IP`
   variable in `src/rprobots/main.py` with this value.

### Experimenter laptop setup

These steps allow you to run the Python/Qt Wizard of Oz client on a laptop (separate from the PC) 
that the researcher controls during the experiment.

1. Clone this repository to the laptop.
2. Install Python and pip, then run `pip3 install PyQt5`
3. Install [Tailscale](https://tailscale.com/kb/installation/) and login using your Google account.
4. Replace the `SERVER_IP` variable in `src/qtwizard/wizardcontrol.py` with the IP you previously copied 
   in Step 9 when setting up ROS nodes.
5. Download [OBS](https://obsproject.com/download).
6. To run the study, connect the laptop to the webcam in the room via a long USB cable. Create an OBS 
   scene that adds an Audio Input Capture source with the webcam microphone and a Video Capture Device 
   source with the webcam. 
   Ensure the audio source from the room can be heard by going to **Audio Mixer > Gear Icon > [Audio Input source name] > Audio Monitoring > Monitor and Output**.

**Note**: Modifying the client UI (*optional, for development only*) is a two-step process. 
After making changes to `qtwizard/rpwizard.ui` in Qt Designer, run `pyuic5 rpwizard.ui -o rpwizard_ui.py` 
to regenerate the Python file. This allows new button functions to be managed in `src/wizardcontrol.py`.

## Experiment Procedure

1. Turn on the PC, monitor, laptop, Misty robot, and Vector robot.
2. On the PC's web browser, go to `<MISTY IP>/sdk/dashboard/index.html` 
   (e.g., `192.168.0.219/sdk/dashboard/index.html`) and set Misty's voice to **speech pitch** ``0.7``, 
   **speech rate** to ``0.95``, and **voice** as ``English 20 (US)``. Test Misty's voice in the web browser 
   after doing so. Please note that the address of Misty may change from time to time. If the address is 
   changed, use the phone app to check the new address.
3. Ensure the PC is connected to the same network as Misty and Vector.
4. On the PC, open OBS. If you are starting a new round of user studies, navigate to Sources and make 
   sure only *B2*, *Timer Text Caption*, *Timer Text*, *Goal Text*, *B1*, *A4*, *Monitor Background* 
   are visible (have the eye icon), and everything else is not.
5. On the PC, open a terminal and launch the main node and ROS wrapper backend. These steps will connect both Misty and Vector:
    ```
    $ cd catkin_ws/src/role-playing-robots/
    $ roslaunch launch/rprobots.launch
    ```
6. On OBS on the PC, click `Main Scene` from Scenes panel. Go to the middle left panel 
   and **Right click > Full Screen Preview > Monitor** on the main scene.
7. Move the mouse and keyboard connected to the server PC out of reach from the participant's seat.
8. Ensure the lab laptop is connected to Internet. Afterwards, navigate to `src/qtwizard/` in this repository, 
   then run `python3 wizardcontrol.py`.
9. On the experimenter laptop, test if you can let both Misty and Vector speak by clicking on the UI.
10. Open up Qualtrics that is bookmarked on the lab laptop, and use `Fn + F11` to full screen the survey.
11. Set up the camera and connect it to the lab laptop via the long USB cable.
12. On the experimenter laptop, open OBS and adjust the camera angle to include both robots, the PC monitor, 
   and the participant. Make you can hear the participant (preparing to record the interaction). If you can't hear the study room, make sure the audio source is set to **Audio Mixer > Right click on the Audio Input Capture item > Advanced Audio Properties > Audio Monitoring > Monitor and Output**. If that still doesn't work, try relaunching OBS and/or going to Settings.
13. Run the study by following the experiment script, setting up the survey and recording the study 
    each time.

## Running data analysis

Our data analysis for our study results can be reproduced using the scripts in this repository:

1. Open `data-analysis/rp-data-analysis.Rmd` using R Studio, then click on the `Knit` function to generate the `rp-data-analysis.html` file.
2. Open the `rp-data-analysis.html` file to view the data analysis. The section number for each measure matches the measures section in the paper. 

Pre-compiled results can also viewed on [the HRI Lab website](https://hri.cs.uchicago.edu/role-playing-robots/rp-data-analysis.html)

## Citation

When using this work, please include the following citation:

Spencer Ng, Ting-Han Lin, You Li, and Sarah Sebo. 2024. Role-Playing with Robot Characters: Increasing User Engagement 
through Narrative and Gameplay Agency. 
In *Proceedings of the 2024 ACM/IEEE International Conference on Human-Robot Interaction (HRI '24), March 11–14, 2024, Boulder, CO, USA*. 
ACM, New York, NY, USA, 11 pages. https://doi.org/10.1145/3610977.3634941