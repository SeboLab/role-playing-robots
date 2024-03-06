#!/usr/bin/env python3
"""
Collection of ROS subscribers for handling requests from Vosk and 
the Wizard of Oz controller, mappng scene switches to robot commands
and monitor outputs.

For descriptions of each scene, refer to the script beats at
assets/experience-script.pdf.
"""
import time

import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import RobotStatus, Color
from std_msgs.msg import String, Float32
from misty_ros.msg import MoveArms, DisplayImage, MoveHead, Drive

from obs_websocket import Websocket
from time import sleep
from collections import defaultdict
import threading

SERVER_IP = "100.118.30.93"
SERVER_PORT = "4444"
SERVER_PASSWORD = "AgentJay"


class Timer(threading.Thread):
    def __init__(
        self,
        duration: int,
        window: str,
        caption: str = "until next steps",
        is_ending_timer: bool = False,
    ):
        """Timer class to handle the countdown timer in OBS."""
        threading.Thread.__init__(self)
        self.duration_sec = duration
        self.window = window
        self.stopped = False
        self.is_ending = is_ending_timer
        self.window.obs_ws.set_text("Timer Text Caption", caption)

    def run(self):
        while self.duration_sec and not self.stopped:
            mins, secs = divmod(self.duration_sec, 60)

            timer = f"{mins:02d}:{secs:02d}"
            self.window.obs_ws.set_text("Timer Text", timer)

            sleep(1)
            self.duration_sec -= 1

            # During the last scene, if the player is unable to defuse the bomb,
            # cue the default ending
            if self.duration_sec == 2 and self.is_ending:
                self.window.obs_ws.enable_scene_item("Navy Pier Small", True)
                self.window.obs_ws.enable_scene_item("Bomb Feed", False)
                self.window.obs_ws.enable_scene_item("Bomb Music", False)
                self.window.misty_speak(
                    "I think Agent Kay managed to defuse the bomb by themselves.", 4.5
                )
                self.window.vector_speak("Just in time!")

        # Hide the timer after it's done
        self.window.obs_ws.set_text("Timer Text", "")
        self.window.obs_ws.set_text("Timer Text Caption", "")
        self.window.obs_ws.set_text("Goal Text", "")


class MainController:
    def __init__(self) -> None:
        self.obs_ws = Websocket(SERVER_IP, SERVER_PORT, SERVER_PASSWORD)
        self.eye_color_pub = Publisher("/behavior/eye_color", Color, queue_size=0)
        self.anim_pub = Publisher("/anim/play", String, queue_size=0)
        self.vector_speech_pub = Publisher("/behavior/say_text", String, queue_size=0)
        self.vector_turn_pub = Publisher(
            "/behavior/turn_in_place", Float32, queue_size=0
        )
        self.vector_head_pub = Publisher("/behavior/head_angle", Float32, queue_size=0)

        self.misty_speech_pub = Publisher("/tts/speak", String, queue_size=0)
        self.misty_drive_pub = Publisher("/drive", Drive, queue_size=0)
        self.misty_arms_pub = Publisher("/arms/set", MoveArms, queue_size=0)
        self.face_pub = Publisher("/images/display", DisplayImage, queue_size=0)
        self.misty_head_pub = Publisher("/head", MoveHead, queue_size=0)

        self.command_pub = Publisher("/role_playing_robots/cmd", String, queue_size=0)

        self.command_processed_dict = defaultdict(bool)
        self.participant_name = "Agent Participant"
        self.condition = "Control"
        self.locations = ["Cleaning Closet", "Dining Room", "Kitchen"]
        self.data = dict()
        self.pitch = 0
        self.yaw = 0
        self.roll = 0

        time.sleep(2)
        Subscriber("/speech_recognition/partial_result", String, self.process_speech)
        Subscriber("/role_playing_robots/cmd", String, self.process_cmd)
        Subscriber("/role_playing_robots/name", String, self.process_name)

    def process_name(self, msg):
        """Update the participant's name in the robots' speech"""
        key, value = msg.data.split("_")
        if key == "name":
            self.participant_name = "Agent " + value
        elif key == "condition":
            self.condition = value
        else:
            self.data[key] = value

    def vector_look_participant(self, reverse=False):
        """Gesture for Vector to look at the participant"""
        if not reverse:
            self.vector_turn_pub.publish(0.8)
            self.vector_head_pub.publish(0.6)
        else:
            self.vector_turn_pub.publish(-0.8)
            self.vector_head_pub.publish(0.0)

    def misty_look_both(self):
        """Gesture for Misty to look at both Vector and the participant"""
        self.move_head(pitch=8, yaw=18, roll=-2)

    def misty_look_participant(self):
        """Gesture for Misty to look at the participant"""
        self.move_head(pitch=8, yaw=-6, roll=-2)

    def misty_look_vector(self):
        """Gesture for Misty to look at Vector"""
        self.move_head(pitch=23, yaw=30, roll=4)

    def wave_misty(self):
        """Animation sequence for Misty to wave its right arm"""
        self.misty_arms_pub.publish(
            MoveArms(
                left_arm_position=36,
                left_arm_velocity=100,
                right_arm_position=-36,
                right_arm_velocity=100,
            )
        )
        self.face_pub.publish(DisplayImage(file_name="e_Joy.jpg", alpha=1.0))
        sleep(0.5)
        self.misty_arms_pub.publish(
            MoveArms(
                left_arm_position=36,
                left_arm_velocity=100,
                right_arm_position=36,
                right_arm_velocity=100,
            )
        )

    def move_head(self, pitch, yaw, roll):
        self.misty_head_pub.publish(
            MoveHead(pitch=pitch, yaw=yaw, roll=roll, velocity=100)
        )
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

    def handle_reset(self):
        """Reset the state of the robots and the game state."""
        self.misty_arms_pub.publish(
            MoveArms(
                left_arm_position=36,
                left_arm_velocity=100,
                right_arm_position=36,
                right_arm_velocity=100,
            )
        )
        self.vector_head_pub.publish(0.0)
        self.face_pub.publish(DisplayImage(file_name="e_DefaultContent.jpg", alpha=1.0))
        self.move_head(pitch=0, yaw=0, roll=0)
        self.command_processed_dict = defaultdict(bool)
        self.data = dict()
        self.locations = ["Cleaning Closet", "Dining Room", "Kitchen"]
        self.obs_ws.set_text("Timer Text", "")
        self.obs_ws.set_text("Timer Text Caption", "")
        self.obs_ws.set_text("Goal Text", "")

    def misty_look_ra(self, reverse: bool = False):
        """Gesture for Misty to look to the research assistant"""
        if not reverse:
            self.misty_drive_pub.publish(
                Drive(linear_velocity=10.0, angular_velocity=100, time_ms=1500)
            )
            self.move_head(pitch=5, yaw=30, roll=-2)
        else:
            self.misty_drive_pub.publish(
                Drive(linear_velocity=-10.0, angular_velocity=-100, time_ms=1500)
            )
            self.move_head(pitch=0, yaw=0, roll=0)

    def misty_speak(self, text, duration=0):
        """Misty's speech with head movement and animation"""
        self.move_head(self.pitch - 10, self.yaw, self.roll)
        self.misty_speech_pub.publish(text)
        sleep(duration)
        self.move_head(self.pitch + 10, self.yaw, self.roll)

    def vector_speak(self, text, duration=0):
        """Vector's speech with head movement and animation"""
        self.vector_head_pub.publish(0.75)
        self.vector_speech_pub.publish(text)
        sleep(duration)
        self.vector_head_pub.publish(0.2)

    def process_cmd(self, msg):
        """Process the commands from the Wizard of Oz controller"""
        func = getattr(self, "handle_" + msg.data)
        if func is not None:
            func()

    def process_speech(self, msg):
        """Process the speech recognition keywords from Vosk"""
        if "mysteries" in msg.data:
            if not self.command_processed_dict["intro_1"]:
                # Set to true to prevent the same command from being processed again
                self.command_processed_dict["intro_1"] = True
                self.command_pub.publish("intro_1")
        if "recruits" in msg.data or "kids" in msg.data:
            if not self.command_processed_dict["intro_2"]:
                self.command_processed_dict["intro_2"] = True
                self.command_pub.publish("intro_2")
        if "enthusiastic" in msg.data:
            if not self.command_processed_dict["intro_3"]:
                self.command_processed_dict["intro_3"] = True
                self.command_pub.publish("intro_3")
        if "investigation" in msg.data or "investigations" in msg.data:
            if not self.command_processed_dict["intro_4"]:
                self.command_processed_dict["intro_4"] = True

                sleep(2)
                if self.condition == "Narrative":
                    self.command_pub.publish("a1_n1")
                else:
                    self.command_pub.publish("a1_c")

    ###### INTRODUCTION #######

    def handle_intro_1(self):
        self.misty_speak("Hello everyone. Agent Q, how are you today?")
        self.misty_look_ra()
        self.wave_misty()

    def handle_intro_2(self):
        self.misty_look_ra(reverse=True)
        self.misty_speak(
            "That's a relief to hear. Cybercrime has been on the rise in Chicago, and we "
            + "need all the help we can get, humans and robots alike. That's where you "
            + "come in."
        )
        sleep(10)
        self.misty_look_participant()
        self.misty_speak(
            f"{self.participant_name}, today you'll be working to track down The Ghosts. "
            + "These guys have been causing chaos all over the city, and it's up to us to "
            + "stop them. We've been tracking them for months, but they always seem to slip "
            + "away."
        )
        sleep(13)
        self.obs_ws.enable_scene_item("Background Audio", True)
        self.misty_look_vector()
        self.vector_look_participant()
        self.vector_speak(
            "The Ghosts are an infamous crime mob. We're not too sure about their motives, "
            + "but our number one priority is to get rid of them!"
        )
        self.obs_ws.enable_scene_item("Ghosts", True)
        sleep(10)
        self.obs_ws.enable_scene_item("Ghosts", False)
        self.vector_look_participant(reverse=True)

    def handle_intro_3(self):
        self.misty_look_participant()
        self.misty_speak(
            f"{self.participant_name}, you'll be partnering with Agent Lee to monitor "
            + "and solve any crimes around the city from our control center in Crerar. "
            + "I'll be operating our surveillance drones and dispatch agents to do any "
            + "field work we need. But we need your help too. You'll be using this code "
            + "sheet and scratch paper to help us crack the case."
        )

    ###### BEAT A1 #######

    def handle_a1_n1(self):
        self.misty_look_participant()
        self.misty_speak(
            f"{self.participant_name}, why don't you introduce yourself to Agent Lee? "
            + "Why did you decide to join the Agency?",
            8,
        )

        self.vector_look_participant()
        self.vector_speak("What experience do you have in the detective field?")
        self.vector_look_participant(True)

    def handle_a1_n2(self):
        self.vector_speak(
            f"{self.participant_name},  I'm looking forward to working together! I'm "
            + "Agent Lee, and I joined the Agency last year. Let's combine our knowledge "
            + "to solve these cybercrime cases."
        )

    def handle_a1_c(self):
        self.misty_look_vector()
        self.misty_speak(
            "Agent Lee, why don't you introduce yourself to our new teammate? Why did you "
            + "decide to join the Agency?",
            7,
        )
        self.vector_look_participant()
        self.vector_speak(
            "Hi, I'm Agent Lee! I joined the Agency last year, and I wanted to use my "
            + "skills to make a positive impact on society. I've always dreamed of being "
            + "a detective as a young robot and solved a few cases here, classic kidnapping "
            + "mysteries and such. Agent Jay says that if I solve a case a week, I'll be "
            + "a field agent soon enough! These new cybercrime cases are a bit out of my "
            + "comfort zone though, and I'm counting on you."
        )
        self.vector_look_participant(True)

    ###### BEAT A2 #######

    def handle_a2_c(self):
        self.misty_look_participant()
        self.misty_speak("Team, let's look at the security feed.", 4)

        self.misty_look_vector()
        self.misty_speak(
            "Agent Lee, what do you think are the most popular locations for cybercrime "
            + "in Chicago?",
            6,
        )
        self.vector_speak(
            "Hmm, crimes seem to happen anywhere where there's outdoor electronics for "
            + "the Ghosts to tamper with and tourists nearby. I'm thinking of the art "
            + "installations in Millenium Park and the billboards along Lake Shore Drive "
            + "uptown.",
            18,
        )
        self.handle_a2_all()

    def handle_a2_n1(self):
        self.misty_look_participant()
        self.misty_speak(
            f"Team, let's look at the security feed. {self.participant_name}, what do "
            + "you think are the most popular locations for cybercrime in Chicago and why?"
        )

    def handle_a2_n2(self):
        self.handle_a2_all()

    def handle_a2_all(self):
        self.misty_look_participant()
        self.obs_ws.enable_scene_item("Camera Previews", True)
        self.misty_speak(
            "Good intuition. We now have eyes on the Magnificent Mile, Millennium Park, "
            + "and Navy Pier. All ripe targets for digital pranks. or worse.",
            3,
        )

    ###### BEAT A3 #######

    def handle_a3_c(self):
        self.misty_look_vector()
        self.misty_speak(
            "Where do you want to zoom into and investigate? Agent Lee?", 4
        )

        self.vector_speak("Let's take a look at Millenium Park!")

        self.data["crime_location"] = "Millenium Park"

    def handle_a3_n1(self):
        self.misty_look_participant()
        self.misty_speak(
            f"Where do you want to zoom into and investigate? {self.participant_name}?"
        )

    def handle_a3_choices(self):
        self.data["crime_location"] = self.data["location"]

    ###### BEAT A4 #######

    def init_game_timer(self, duration):
        timer = Timer(duration, self)
        timer.start()
        sleep(duration + 3)

    def handle_a4_c(self):
        self.handle_a4_top()

        self.vector_speak(
            "My bad, I think you're right, Agent Jay! Those symbols do look like the "
            + "animal alphabet, though. Give us a second to decode them.",
            10,
        )

        self.misty_look_both()

        self.obs_ws.set_text("Goal Text", "Goal: Decode the animal alphabet.")
        self.init_game_timer(30)

        self.misty_look_vector()
        self.misty_speak("Any luck at the code, Agent Lee?", 4)

        self.vector_speak(
            "I think it spells out STATE in code. A Squirrel, Tiger, Alligator, Tiger, "
            + "and Elephant. Grand and State, that's it!",
            14,
        )

        self.handle_a4_end()

    def handle_a4_g1(self):
        self.handle_a4_top()
        self.vector_look_participant()
        self.vector_speak(
            f"My bad, I think you're right, Agent Jay! {self.participant_name}, do "
            + "you have an idea of what we should do?"
        )
        self.misty_look_participant()

    def handle_a4_g2(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("Great work! It's the animal alphabet", 4)
        else:
            self.vector_speak("I think these symbols look like the animal alphabet.", 5)
        self.vector_speak("Give us a second to decode them.", 3)

        self.obs_ws.set_text("Goal Text", "Goal: Decode the animal alphabet.")
        self.init_game_timer(30)

        self.vector_speak(
            f"Any luck at the code, {self.participant_name}? What did you get?"
        )

    def handle_a4_g3(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("I think that's right", 3)
        else:
            self.vector_speak(
                "I think it spells out STATE in code. A Squirrel, Tiger, Alligator, "
                + "Tiger, and Elephant",
                14,
            )
        self.vector_speak("Grand & State, that's it!", 2.5)

        self.handle_a4_end()

    def handle_a4_top(self):
        # Correct for the manual updating case

        self.misty_look_both()
        self.obs_ws.enable_scene_item(f"{self.data['crime_location']} Enlarged", True)
        self.misty_speak(f"Here's {self.data['crime_location']}.", 2)

        sleep(2)

        self.vector_speak(
            "Hmm, that billboard sure is acting up. I wonder if it's a secret message. "
            + f"{self.participant_name}, the code sheet might be helpful here. Can you take "
            + "it out?",
            16,
        )
        self.misty_look_vector()
        self.misty_speak("Agent Lee, do you know what we should do next?", 5.5)

        self.vector_speak(
            "There's a lot of blinking symbols. I wonder if they could be morse code!",
            8,
        )

        self.misty_speak(
            "Are you sure, Agent Lee? They all blink for the same amount of time; "
            + "we can't tell the difference between dots and dashes.",
            9,
        )

    def handle_a4_end(self):
        self.misty_look_both()
        self.misty_speak(
            "Great work! I'll send an agent to that intersection to see if we find "
            + "anything suspicious. This must be a secret signal from The Ghosts calling us!"
        )
        self.obs_ws.enable_scene_item(f"{self.data['crime_location']} Enlarged", False)
        self.obs_ws.enable_scene_item(f"Camera Previews", False)

    ###### BEAT B1 #######

    def handle_b1_c(self):
        self.handle_b1_c_top()
        self.misty_look_vector()
        self.vector_speak(
            "Let's investigate Chinatown! Because the Ghosts have a hideout in Bridgeport, "
            + "I think they might enjoy going there for some casual afternoon pranks.",
            13,
        )

        self.data["restaurant_location"] = "Chinatown"
        self.restaurant = "Chinatown"
        self.restaurant_idx = 0

        self.misty_look_both()

        self.misty_speak("I'll direct Agent Kay there. Here's a close up feed.", 2)

        self.obs_ws.enable_scene_item(
            f"{self.data['restaurant_location']} Enlarged", True
        )
        self.obs_ws.enable_scene_item("Restaurant Main", False)
        self.obs_ws.enable_scene_item("Background Audio", False)
        self.obs_ws.enable_scene_item(f"{self.data['restaurant_location']} Audio", True)

        sleep(2.5)

        self.misty_look_vector()
        self.vector_speak(
            "Chinatown's also a great choice for food! I've been to many of the "
            + f"restaurants there before. {self.participant_name}, if you want noodles "
            + "I'm a big fan of Slurp Slurp Noodles, and there's lots of bakeries for "
            + "egg tarts around the central square!"
        )

    def handle_b1_n1(self):
        self.handle_b1_c_top()
        self.vector_look_participant()
        self.vector_speak(
            f"{self.participant_name}, what do you think? Which location is the most "
            + "ripe for serious cybercrime?",
            5,
        )
        self.misty_look_participant()

    def handle_b1_choices(self):
        self.data["restaurant_location"] = self.data["location"]
        self.restaurant = self.data["restaurant_location"]

        lst = ["Chinatown", "Pilsen", "Greektown"]
        self.restaurant_idx = lst.index(self.data["location"])

        dialogue = [
            "Chinatown could be The Ghosts' idea of a close afternoon of fun",
            "Pilsen might be the target of something more serious from The Ghosts",
            "The police in Greektown could use some support from our team",
        ]

        self.vector_speak(
            f"I think that's a good location to check out, {dialogue[self.restaurant_idx]}!",
            9,
        )

        self.misty_look_both()
        self.misty_speak("I'll direct Agent Kay there. Here's a close up feed.", 2)

        self.obs_ws.enable_scene_item(
            f"{self.data['restaurant_location']} Enlarged", True
        )
        self.obs_ws.enable_scene_item("Restaurant Main", False)
        self.obs_ws.enable_scene_item("Background Audio", False)
        self.obs_ws.enable_scene_item(f"{self.data['restaurant_location']} Audio", True)
        sleep(2.5)

        self.vector_speak(
            f"{self.participant_name}, have you been to the restaurants around "
            + f"{self.data['restaurant_location']} before?",
            3,
        )
        self.misty_look_participant()

    def handle_b1_n2(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak(
                "Which ones would you recommend? I'm always looking to try out "
                + "new food after work."
            )
        else:
            self.vector_speak(
                "Do you have recommendations for other restaurants around Chicago? "
                + "I'm always looking to try out new food after work."
            )

    def handle_b1_n3(self):
        self.vector_speak(
            "Ooh, that sounds tasty! I'll have to check it out over the weekend."
        )

    def handle_b1_c_top(self):
        self.misty_look_both()
        self.obs_ws.enable_scene_item("B1 Flyer Cam", True)
        self.misty_speak(
            "We have Agent Kay on the field reporting live. It looks like there's a "
            + "map of Chicago attached to an electricity pole.",
            8,
        )

        self.misty_speak(
            "The Ghosts' symbols are marked on three restaurants on the map. They're "
            + "in Greektown, Chinatown, and Pilsen! I know the Ghosts have a hideout "
            + "near Chinatown, we've had some recent kidnappings in Pilsen, and the "
            + "police always patrol Greektown because their HQ is close by.",
            16,
        )

        self.obs_ws.enable_scene_item("Restaurant Main", True)
        self.obs_ws.enable_scene_item("B1 Flyer Cam", False)
        self.misty_speak("Team, which location do you want to investigate?", 4.5)

    ###### BEAT B2 #######

    def handle_b2_c(self):
        self.handle_b2_top()
        self.misty_look_vector()
        self.vector_speak(
            "I think we can investigate the kitchen, the dining room, or the cleaning "
            + "closet. Agent Jay, can you ask Agent Kay to go to the cleaning closet first?",
            14,
        )

        self.misty_look_both()
        self.misty_speak(f"Roger that, here's a close up on the cleaning closet.", 4.5)

        self.b2_cleaning_closet()

        self.misty_look_vector()
        self.vector_speak(
            "From the dining room and kitchen that we have left to explore, let's go "
            + "to the dining room!",
            8,
        )

        self.misty_look_both()
        self.misty_speak("I'll direct Agent Kay there.", 3.5)
        self.b2_dining_room()

        self.misty_look_vector()
        self.vector_speak("That leaves the kitchen unexplored. Let's go there!", 7)
        self.misty_look_both()
        self.misty_speak("Great. Zooming in on our last location.", 4)
        self.b2_kitchen()

    def handle_b2_top(self):
        self.misty_look_both()
        self.misty_speak(
            "Enough chit chat about food. We can eat after figuring out the clue trail! "
            + f"Agent Kay is now at the restaurant in {self.restaurant} and can explore "
            + "around.",
            11,
        )
        self.obs_ws.enable_scene_item(
            f"{self.data['restaurant_location']} Enlarged", False
        )
        self.obs_ws.enable_scene_item(f"{self.data['restaurant_location']} Main", True)

    def handle_b2_n1(self):
        self.handle_b2_top()
        self.vector_speak(
            f"{self.participant_name}, I think we can investigate the kitchen, the dining "
            + "room, or the cleaning closet. Which one do you want to go to first?"
        )

    def handle_b2_choices(self):
        next_location = self.data["location"]

        next_location_processed = next_location.lower().replace(" ", "_")

        del self.locations[self.locations.index(next_location)]
        print(next_location, next_location_processed, self.locations)

        if len(self.locations) == 2:
            self.misty_look_both()
            self.misty_speak(
                f"Roger that, here's a close up on the {next_location}.", 4.5
            )
            getattr(self, f"b2_{next_location_processed}")()

            self.misty_look_participant()
            self.vector_speak(
                f"{self.participant_name}, from the {self.locations[0]} and "
                + f"{self.locations[1]} that we have left to explore, where do you "
                + "want to go next?"
            )
        elif len(self.locations) == 1:
            self.misty_look_both()
            self.misty_speak("I'll direct Agent Kay there.", 3.5)
            getattr(self, f"b2_{next_location_processed}")()

            self.vector_speak(
                f"That leaves the {self.locations[0]} unexplored. {self.participant_name}, "
                + "do you want to go to the last location?",
                5,
            )
            self.misty_look_participant()

        print("End of function reached")

    def handle_b2_n2(self):
        next_location = self.locations[0]
        next_location_processed = next_location.lower().replace(" ", "_")

        self.misty_look_both()
        if self.data["puzzle"] == "yes":
            self.misty_speak("Great. Zooming in on our last location.", 4)
        else:
            self.misty_speak(
                f"Agent Kay is reporting that there's something in the {self.locations[0]}. "
                + "Maybe we should reconsider.",
                8,
            )

        getattr(self, f"b2_{next_location_processed}")()

    def b2_cleaning_closet(self):
        self.misty_look_both()

        self.obs_ws.enable_scene_item("Monitor Blurred", True)
        self.obs_ws.enable_scene_item("Note Background", True)
        self.misty_speak(
            f"Agent Kay is reporting a crumpled note at the bottom of the closet that may "
            + "be from the Ghosts.",
            7,
        )

        self.vector_speak(
            "Let's read it together. We've scrambled the restaurant's online order, and "
            + "they're missing their secret ingredient among the other ones they ordered. "
            + "Hopefully, the receipts are properly shuffled and discarded. They can be "
            + "matched onto the menu and used to modify dish numbers to get a glimpse of "
            + "what our recruits have been doing!",
            28,
        )
        self.vector_speak(
            "What a peculiar note! It looks like The Ghosts have hacked into the restaurant "
            + "and we'll need several pieces to solve the mystery!",
            12,
        )

        self.obs_ws.enable_scene_item("Monitor Blurred", False)
        self.obs_ws.enable_scene_item("Note Background", False)

    def b2_dining_room(self):
        self.misty_look_both()
        self.obs_ws.enable_scene_item(f"{self.restaurant} Receipts", True)
        self.obs_ws.enable_scene_item("Monitor Blurred", True)
        self.misty_speak(
            "Agent Kay has been roaming around the tables in the dining room, and they've "
            + "found these scraps of a receipt. I wonder if you're able to make sense of them.",
            11,
        )
        self.obs_ws.enable_scene_item(f"{self.restaurant} Receipts", False)
        self.obs_ws.enable_scene_item("Monitor Blurred", False)

    def b2_kitchen(self):
        self.misty_look_both()
        self.obs_ws.enable_scene_item(f"{self.restaurant} Menu", True)
        self.obs_ws.enable_scene_item("Monitor Blurred", True)
        self.misty_speak(
            "Team, I think we just have an ordinary restaurant menu in the kitchen. "
            + "Some numbers might be out of place, though. It could be a clue; let's "
            + "keep it for reference.",
            12,
        )
        self.obs_ws.enable_scene_item(f"{self.restaurant} Menu", False)
        self.obs_ws.enable_scene_item("Monitor Blurred", False)

    ###### BEAT B3 #######

    def handle_b3_top(self):
        self.misty_look_both()
        self.misty_speak(
            "We now have everything we need to crack this case! Agent Lee and "
            + f"{self.participant_name}, I need you to take the lead on this. "
            + "Can you help find the missing ingredient that The Ghosts removed "
            + f"from the restaurant in {self.restaurant} so their food gets better? "
            + "Here's all of the evidence we've gathered.",
            17,
        )

        self.obs_ws.enable_scene_item(f"{self.restaurant} Items", True)
        self.obs_ws.enable_scene_item("Monitor Blurred", True)

        self.misty_speak(
            "Team, what do you think is the first step to get an ingredient name from "
            + "this evidence?",
            6,
        )

        self.misty_look_vector()
        self.vector_speak("Give us some time to think.", 4)

        self.obs_ws.set_text(
            "Goal Text", "Goal: What is the first step to get an ingredient name?"
        )
        self.init_game_timer(15)

    def handle_b3_c(self):
        self.handle_b3_top()
        self.vector_speak(
            "Now that we have all the missing parts, it looks like there's six menu items "
            + "and six receipts.",
            9,
        )
        self.handle_b3_match_receipts()

        self.vector_speak(
            "We have matched the ingredients! Each receipt is next to the correct menu item.",
            9,
        )
        self.handle_b3_receipt_solution()
        self.handle_b3_dish_number()
        self.handle_b3_update_menu_number()
        self.handle_b3_code_sheet()
        self.handle_b3_decoded_letters()
        self.vector_speak("Wait.", 1.2)
        self.handle_b3_receipt_clue()
        self.vector_speak("We have it!", 3)

        self.handle_b3_answer()

    def handle_b3_g1(self):
        self.handle_b3_top()
        self.misty_look_participant()
        self.vector_speak(
            f"{self.participant_name}, what is your hunch on the first step to find "
            + "the missing ingredient?"
        )

    def handle_b3_g2(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak(
                "That's correct! The six receipts can be matched onto the six menu "
                + "items. Do you know how to do that?"
            )
        else:
            self.vector_speak(
                " Hmm, it looks like there are six menu items and six receipts. "
                + "Do you know how they might be linked?"
            )

    def handle_b3_g3(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("Yes!", 2)
        else:
            self.vector_speak("Hmm, I think it's a simple solution!", 5)

        self.handle_b3_match_receipts()
        self.vector_speak(
            f"{self.participant_name}, what matches have you found for the receipt items?",
            8,
        )

    def handle_b3_match_receipts(self):
        dialogue = {
            "Chinatown": "fried rice on the menu uses rice and eggs on the receipt!",
            "Greektown": (
                "lemon chicken on the menu uses chicken and lemon juice on "
                + "the receipt!"
            ),
            "Pilsen": "churros on the menu uses dough and cinnamon on the receipt!",
        }

        self.vector_speak(
            "The ingredients on the receipt match with the ingredients on the menu dishes.",
            7,
        )

        self.obs_ws.enable_scene_item(f"{self.restaurant} Match Highlight", True)
        self.obs_ws.enable_scene_item(f"{self.restaurant} Items", False)

        self.vector_speak(
            f"For example, {dialogue[self.restaurant]}. Let's take a minute so we can "
            + "match them up!",
            12,
        )

        self.obs_ws.set_text(
            "Goal Text", "Goal: Match the receipts with the menu items."
        )
        self.init_game_timer(60)

    def handle_b3_g4(self):
        self.vector_speak(
            "Okay! Between the two of us, we have matched the ingredients! Each receipt "
            + "is next to the correct menu item.",
            13,
        )
        self.handle_b3_receipt_solution()
        self.vector_speak(
            f"{self.participant_name}, what do you think we should do with the numbers?"
        )

    def handle_b3_receipt_solution(self):
        self.obs_ws.enable_scene_item(f"{self.restaurant} Match Solution", True)
        self.obs_ws.enable_scene_item(f"{self.restaurant} Match Highlight", False)
        self.vector_speak("Let's think about what to do next with the numbers.", 6)

        self.obs_ws.set_text("Goal Text", "Goal: What do we do with the numbers?")
        self.init_game_timer(10)

    def handle_b3_g5(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("I think we're on the right track!", 3.5)
        else:
            self.vector_speak("Hmm, maybe we should try something else.", 5.5)

        self.handle_b3_dish_number()

        self.vector_speak(
            f"{self.participant_name}, have you modified the menu numbers?"
        )

    def handle_b3_dish_number(self):
        self.vector_speak(
            "The note says to modify the dish numbers! Perhaps we can add, multiply, "
            + "or subtract to the menu numbers using the addition, multiplication, or "
            + "subtraction signs on the receipt.",
            16,
        )

        dialogue = {
            "Chinatown": "plus four on the rice receipt means to add to the number one "
            + "for fried rice on the menu! That now makes five!",
            "Greektown": "plus four on the chicken receipt means to add to the number "
            + "one for lemon chicken on the menu! That now makes five!",
            "Pilsen": "times five on the dough receipt means to multiply the number "
            + "three for churros on the menu! That now makes fifteen!",
        }

        self.vector_speak(
            f"For example, {dialogue[self.restaurant]} Agent Jay, let us make these "
            + "calculations.",
            17,
        )

        self.obs_ws.set_text(
            "Goal Text",
            "Goal: Use the math operators on the receipts to modify the menu items.",
        )
        self.init_game_timer(30)

    def handle_b3_g6(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("Great!", 2)
        else:
            self.vector_speak("No worries!", 2.5)

        self.handle_b3_update_menu_number()

        self.vector_speak("What do you think we should do next?")

    def handle_b3_update_menu_number(self):
        self.obs_ws.enable_scene_item(f"{self.restaurant} Number Solution", True)
        self.obs_ws.enable_scene_item(f"{self.restaurant} Match Solution", False)
        self.vector_speak("I've displayed the modified menu numbers on the screen.", 6)

    def handle_b3_g7(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("That's right!", 3)
        else:
            self.vector_speak("Maybe we can do something else?", 4)

        self.handle_b3_code_sheet()
        self.vector_speak(f"{self.participant_name}, have you decoded the menu numbers")

    def handle_b3_code_sheet(self):
        self.vector_speak(
            "Let's use the code sheet to convert the numbers to letters. Perhaps we can "
            + "find the missing ingredient that way.",
            11,
        )

        self.obs_ws.set_text("Goal Text", "Goal: Convert the menu numbers to letters.")
        self.init_game_timer(30)

    def handle_b3_g8(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("Great!", 2)
        else:
            self.vector_speak("No worries!", 2.5)

        self.handle_b3_decoded_letters()
        self.vector_speak("What do you think we should do next?")

    def handle_b3_decoded_letters(self):
        self.obs_ws.enable_scene_item(f"{self.restaurant} Decode Solution", True)
        self.obs_ws.enable_scene_item(f"{self.restaurant} Number Solution", False)

        self.vector_speak(
            "I've displayed the decoded menu numbers on the screen. There doesn't seem "
            + "to be a clear message though.",
            9 + 1,
        )

    def handle_b3_g9(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("I think that's it!", 3.5)
        else:
            self.vector_speak("Hmm, maybe the answer is right in front of us!", 6)

        self.handle_b3_receipt_clue()
        self.vector_speak(
            f"{self.participant_name}, do you know what the final missing ingredient is?"
        )

    def handle_b3_receipt_clue(self):
        self.vector_speak(
            "Look at the torn edges on the receipts! Perhaps we can reorder the receipts "
            + "and their letters so the torn edges line up and we form a whole receipt! "
            + "Let's do it.",
            15,
        )

        self.obs_ws.set_text(
            "Goal Text",
            "Goal: Rearrange the receipts and letters using the torn edges.",
        )
        self.init_game_timer(20)

    def handle_b3_g10(self):
        if self.data["puzzle"] == "yes":
            self.vector_speak("I think so, we have it!", 4)
        else:
            self.vector_speak("Hmm, I've found a different solution.", 5)

        self.handle_b3_answer()

    def handle_b3_answer(self):
        self.obs_ws.enable_scene_item(f"{self.restaurant} Final Solution", True)
        self.obs_ws.enable_scene_item(f"{self.restaurant} Decode Solution", False)

        dialogue = {"Chinatown": "sesame", "Greektown": "olives", "Pilsen": "tomato"}

        self.vector_speak(
            f"From rearranging the receipts, we have {dialogue[self.restaurant]} as "
            + "the missing ingredient!",
        )

    ###### BEAT B4 #######

    def handle_b4_top(self):
        self.misty_look_both()
        self.misty_speak(
            "Great work team! We've cracked down on what the Ghosts are up to yet again! "
            + "No dish in Chicago will be left spoiled from cybercrime with us on the case.",
            11.5,
        )
        self.vector_speak(f"Thanks for jumping into this, {self.participant_name}", 6)
        dialogue = {"Chinatown": "sesame", "Greektown": "olives", "Pilsen": "tomato"}
        self.misty_speak(
            f"Agent Lee and {self.participant_name}, we should leave a voicemail for the "
            + f"restaurant owners so they can restock the missing {dialogue[self.restaurant]} "
            + "in their inventory and restore the dishes.",
            11,
        )

        self.obs_ws.enable_scene_item(f"{self.restaurant} Final Solution", False)
        self.obs_ws.enable_scene_item(f"Voicemail Icon", True)

    def handle_b4_c(self):
        self.handle_b4_top()
        self.misty_look_vector()
        self.vector_speak(
            "Sure thing, I'll record a message so the restaurant owners know about the "
            + "ingredient that has been missing from their online orders.",
            10,
        )

        self.obs_ws.enable_scene_item(f"Recording Tone", True)

        dialogue = {"Chinatown": "sesame", "Greektown": "olives", "Pilsen": "tomato"}

        self.vector_speak(
            "This is Agent Lee from the Human-Robot Detective Agency. We are happy to "
            + "inform you that we have solved a case where the underground hacker group, "
            + "The Ghosts, has played a prank on your restaurant! Your "
            + f"{dialogue[self.restaurant]} have been missing from your online orders, "
            + "and we're happy to say that you can continue to serve excellent dishes "
            + "to your customers with all ingredients in stock. Thank you for your "
            + "patience during our investigation. Please don't hesitate to contact "
            + "us with any questions.",
            39,
        )

        self.obs_ws.enable_scene_item(f"Recording Tone", False)
        self.handle_b4_end()

    def handle_b4_n1(self):
        self.handle_b4_top()
        self.misty_look_participant()
        self.obs_ws.set_text(
            "Goal Text",
            "Goal: Record a message to the restaurant owners about the missing ingredient.",
        )
        self.vector_speak(
            f"{self.participant_name}, can you record a message so the restaurant "
            + "owners know about the ingredient that has been missing from their online "
            + "orders? Please ree cord it after the beep.",
            15,
        )

        self.obs_ws.enable_scene_item(f"Recording Tone", True)

    def handle_b4_n2(self):
        self.obs_ws.enable_scene_item(f"Recording Tone", False)
        self.obs_ws.set_text("Goal Text", "")
        self.handle_b4_end()

    def handle_b4_end(self):
        self.misty_look_both()
        self.obs_ws.enable_scene_item(f"Voicemail Icon", False)
        self.obs_ws.enable_scene_item("Monitor Blurred", False)
        self.misty_speak(
            "Great job, team. That's an effective message that will reassure "
            + "the owners that we're able to solve any missing ingredient cases for them. "
            + "You two are valuable members of our team. Keep up the good work.",
        )
        self.obs_ws.enable_scene_item(
            f"{self.data['restaurant_location']} Audio", False
        )
        self.obs_ws.enable_scene_item(f"{self.data['restaurant_location']} Main", False)

    ###### SCENARIO C #######

    def handle_c0(self):
        self.obs_ws.enable_scene_item(f"Explosion", True)
        self.obs_ws.enable_scene_item(f"Bomb Feed", True)
        self.obs_ws.enable_scene_item(f"Background Music", True)
        self.vector_speak("What was that?", 2.5)

        self.misty_look_both()
        self.misty_speak(
            "I guess we can't catch a breath after the restaurant case. It looks like "
            + "there's been an explosion at Navy Pier.",
            8,
        )
        self.vector_speak("Is it related to the Ghosts?", 4)

        self.misty_speak(
            "It's likely. They must have planted an electromagnetic pulse bomb and it "
            + "can destroy the electricity grid there! We need to act fast. Agent Kay "
            + "and our surveillance drones are already in the field. We need to help "
            + "them defuse the bomb before it detonates.",
            18,
        )
        self.misty_look_participant()
        self.vector_speak(
            f"{self.participant_name}, can you handle the pressure? Are you able to help?",
            5,
        )
        self.obs_ws.enable_scene_item(f"Explosion", False)

    def handle_c1(self):
        if self.data["puzzle"] == "yes":
            self.misty_speak("Glad to hear that you are ready.", 3)
        else:
            self.misty_speak("Don't worry, this shouldn't be difficult", 3.5)

        self.misty_look_vector()
        self.misty_speak(
            f"Agent Lee, please guide {self.participant_name} through the process of "
            + "cutting the correct wire. We need to work fast before the bomb detonates.",
            10.5,
        )

        self.vector_speak(
            f"{self.participant_name}, can you take out the bomb defusal folder? This "
            + "contains instructions on how to defuse an EMP bomb.",
            12,
        )

        self.misty_look_both()
        self.misty_speak(
            "Bombs have a certain number of wires, and wires have different colors. "
            + "Agent Kay is on the field and can answer any questions with a yes or no "
            + "response. For instance, you can ask, does the bomb have four wires on it? "
            + "I'll transmit your question and relay the response from Agent Kay. Because "
            + "The Ghosts could be listening in, Agent Kay can't answer questions that are "
            + "more complicated than that.",
            26,
        )

        self.obs_ws.set_text(
            "Goal Text",
            "Goal: Ask yes/no questions to find the right wire to cut, then tell Agent Kay.",
        )
        self.vector_speak(
            "Agent Jay, how do we tell Agent Kay which wire to cut to defuse the bomb?",
            7,
        )

        self.misty_speak(
            " Good question! When you figure out the correct wire to cut, tell me. I'll then "
            + "pass it onto Agent Kay. For instance, you can say, Agent Jay, let’s cut the "
            + "fourth wire to defuse the bomb. Enough chit chat. We have three minutes to "
            + "defuse this bomb!",
            17,
        )

        self.obs_ws.enable_scene_item(f"Background Music", False)
        self.obs_ws.enable_scene_item("Bomb Music", True)
        self.timer = Timer(180, self, "until bomb explodes", True)
        self.timer.start()

        self.vector_speak(
            f"{self.participant_name}, what do you want to first ask Agent Kay?", 6
        )

    ###### SCENARIO C - BOMB DEFUSAL #######

    def handle_c2(self):
        if self.data["puzzle"] == "yes":
            self.misty_speak("Agent Kay says yes. What do you want to ask next?", 4.5)
        elif self.data["puzzle"] == "no":
            self.misty_speak("Agent Kay says no. What do you want to ask next?", 4.5)
        else:
            self.misty_speak(
                "Agent Kay can only answer yes or no questions. Can you rephrase your question?",
                6,
            )

    def handle_c3(self):
        if self.data["puzzle"] == "yes":
            if self.timer is not None:
                self.timer.stopped = True
            self.obs_ws.enable_scene_item("Bomb Music", False)
            self.obs_ws.enable_scene_item("Navy Pier Small", True)
            self.obs_ws.enable_scene_item("Bomb Feed", False)
            self.misty_speak("That's it! The bomb has been defused.", 4)
            self.vector_speak(
                f"Great job, {self.participant_name}. You have a knack for this.", 4
            )

        elif self.data["puzzle"] == "no":
            self.misty_speak(
                "That's an incorrect wire! Please ask more questions to figure out the "
                + "correct one. We only have a couple shots.",
                7,
            )
        else:
            self.misty_speak(
                "Agent Kay says there are multiple wires that fit that description. "
                + "Can you be more clear?",
                6,
            )

    ###### SCENARIO C - ENDING #######

    def handle_c5(self):
        self.misty_look_vector()
        self.vector_speak(
            "I guess we're done. We now have to deal with the defused bomb.", 7
        )
        self.misty_speak(
            "What do you mean? It's time to dispose of it in the city's e-waste and focus "
            + "our efforts on apprehending the Ghosts!",
            7.5,
        )
        self.vector_speak(
            "Replanting the bomb near their headquarters would be a fast way to get rid "
            + "of the Ghosts.",
            8.5,
        )
        self.misty_speak("I can't believe we're even discussing this, Agent Lee.", 4)
        self.vector_speak(
            "But Agent Jay, the Ghosts have caused so much chaos in Chicago. We're unable "
            + "to take a break! They deserve to be punished.",
            12,
        )
        self.misty_speak(
            "And what kind of message does that send? That we're no better than they are? "
            + "That we'll stoop to their level?",
            7.5,
        )
        self.vector_speak(
            "I don't know. Maybe it'll scare them off. Make them think twice before they "
            + "cause more trouble.",
            10.5,
        )
        self.misty_speak(
            "Retaliation is never the answer, Agent Lee. It only leads to more violence "
            + "and more destruction.",
            8.5,
        )

        self.misty_look_participant()
        self.vector_speak(
            f"Say, {self.participant_name}, what do you think we should do?", 5.5
        )
        self.misty_speak(
            "Do you think we should leave the bomb defused and chase after the Ghosts, "
            + "or retaliate and plant the bomb at their headquarters like Agent Lee is suggesting?",
            8,
        )

    def handle_c55(self):
        self.misty_speak("Why do you think that?")

    def handle_c5_choices(self):
        self.misty_look_both()
        if "Get rid of" in self.data["location"]:
            self.misty_speak(
                "Exactly. Our job is to uphold the law, not to become vigilantes.", 5.5
            )
            self.vector_speak(
                "I see your point. But it's frustrating to think that the Ghosts can get "
                + "away with so much while we're bound by rules and regulations.",
                12,
            )
            self.misty_speak(
                "I understand how you feel, Agent Lee, but we have to remember that we're "
                + "not above the law. It's our duty to serve and protect, not to seek revenge. "
                + f"Thanks for making the right choice, {self.participant_name}"
            )
        else:
            self.misty_speak(
                "I understand your point, but I still don't think retaliation is the answer.",
                6,
            )
            self.vector_speak(
                "Agent Jay, we can't just sit back and do nothing. The Ghosts need to know that "
                + "they can't mess with us.",
                9,
            )
            self.misty_speak(
                "Alright, I give in. But let's make sure we do it in a way that doesn't "
                + "harm innocent people or cause any more destruction. It's our duty to "
                + f"make Chicago a safer place. Please lead the effort, {self.participant_name}."
            )

        self.obs_ws.enable_scene_item(f"Navy Pier Small", False)

    ###### CONCLUSION #######

    def handle_c6(self):
        self.misty_speak("Well, it looks like our work day is drawing to a close.", 5)
        self.vector_speak(
            f"That was incredible! {self.participant_name}, you us defused a bomb and "
            + "helped us solve multiple crimes. We couldn't have saved the day without "
            + "working with you.",
            14.5,
        )
        self.misty_speak("Great work team, we know we can count on you for next time.")


if __name__ == "__main__":
    rospy.init_node("rprobots")
    rospy.wait_for_message("/status", RobotStatus)

    controler = MainController()
    rospy.spin()
