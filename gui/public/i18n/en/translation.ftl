### SlimeVR complete GUI translations
# Please developers (not translators) don't reuse a key inside another key
# or concat text with a translation string in the code, use the appropriate
# features like variables and selectors in each appropriate case!
# And also comment the string if it's something not easy to translate, so you help
# translators on what it means

## Websocket (server) status
websocket-connecting = Connecting to the server
websocket-connection_lost = Connection lost to the server. Trying to reconnect...

## Tips
tips-find_tracker = Not sure which tracker is which? Shake a tracker and it will highlight the corresponding item.
tips-do_not_move_heels = Ensure your heels do not move during recording!

## Body parts
body_part-NONE = Unassigned
body_part-HEAD = Head
body_part-NECK = Neck
body_part-RIGHT_SHOULDER = Right shoulder
body_part-RIGHT_UPPER_ARM = Right upper arm
body_part-RIGHT_LOWER_ARM = Right lower arm
body_part-RIGHT_HAND = Right hand
body_part-RIGHT_UPPER_LEG = Right thigh
body_part-RIGHT_LOWER_LEG = Right ankle
body_part-RIGHT_FOOT = Right foot
body_part-RIGHT_CONTROLLER = Right controller
body_part-CHEST = Chest
body_part-WAIST = Waist
body_part-HIP = Hip
body_part-LEFT_SHOULDER = Left shoulder
body_part-LEFT_UPPER_ARM = Left upper arm
body_part-LEFT_LOWER_ARM = Left lower arm
body_part-LEFT_HAND = Left hand
body_part-LEFT_UPPER_LEG = Left thigh
body_part-LEFT_LOWER_LEG = Left ankle
body_part-LEFT_FOOT = Left foot
body_part-LEFT_CONTROLLER = Left controller

## Skeleton stuff
skeleton_bone-NONE = None
skeleton_bone-HEAD = Head Shift
skeleton_bone-NECK = Neck Length
skeleton_bone-TORSO = Torso Length
skeleton_bone-CHEST = Chest Distance
skeleton_bone-WAIST = Waist Distance
skeleton_bone-HIP_OFFSET = Hip Offset
skeleton_bone-HIPS_WIDTH = Hips Width
skeleton_bone-LEGS_LENGTH = Legs Length
skeleton_bone-KNEE_HEIGHT = Knee Height
skeleton_bone-FOOT_LENGTH = Foot Length
skeleton_bone-FOOT_SHIFT = Foot Shift
skeleton_bone-SKELETON_OFFSET = Skeleton Offset
skeleton_bone-CONTROLLER_DISTANCE_Z = Controller Distance Z
skeleton_bone-CONTROLLER_DISTANCE_Y = Controller Distance Y
skeleton_bone-FOREARM_LENGTH = Forearm Distance
skeleton_bone-SHOULDERS_DISTANCE = Shoulders Distance
skeleton_bone-SHOULDERS_WIDTH = Shoulders Width
skeleton_bone-UPPER_ARM_LENGTH = Upper Arm Length
skeleton_bone-ELBOW_OFFSET = Elbow Offset

## Tracker reset buttons
reset-reset_all = Reset all proportions
reset-full = Reset
reset-mounting = Reset Mounting
reset-quick = Quick Reset

## Serial detection stuff
serial_detection-new_device-p0 = New serial device detected!
serial_detection-new_device-p1 = Enter your WiFi credentials!
serial_detection-new_device-p2 = Please select what you want to do with it
serial_detection-open_wifi = Connect to WiFi
serial_detection-open_serial = Open Serial Console
serial_detection-submit = Submit!
serial_detection-close = Close

## Navigation bar
navbar-home = Home
navbar-body_proportions = Body Proportions
navbar-trackers_assign = Tracker Assignment
navbar-mounting = Mounting Calibration
navbar-onboarding = Setup Wizard
navbar-settings = Settings

## Bounding volume hierarchy recording
bvh-start_recording = Record BVH
bvh-recording = Recording...

## Overlay settings
overlay-is_visible_label = Show Overlay in SteamVR
overlay-is_mirrored_label = Display Overlay as Mirror

## Tracker status
tracker-status-none = No Status
tracker-status-busy = Busy
tracker-status-error = Error
tracker-status-disconnected = Disconnected
tracker-status-occluded = Occluded
tracker-status-ok = OK

## Tracker status columns
tracker-table-column-name = Name
tracker-table-column-type = Type
tracker-table-column-battery = Battery
tracker-table-column-ping = Ping
tracker-table-column-rotation = Rotation X/Y/Z
tracker-table-column-position = Position X/Y/Z
tracker-table-column-url = URL

## Tracker rotation
tracker-rotation-front = Front
tracker-rotation-left = Left
tracker-rotation-right = Right
tracker-rotation-back = Back

## Tracker information
tracker-infos-manufacturer = Manufacturer
tracker-infos-display_name = Display Name
tracker-infos-custom_name = Custom Name
tracker-infos-url = Tracker URL

## Tracker settings
tracker-settings-back = Go back to trackers list
tracker-settings-title = Tracker settings
tracker-settings-assignment_section = Assignment
tracker-settings-assignment_section-description = What part of the body the tracker is assigned to.
tracker-settings-assignment_section-edit = Edit assignment
tracker-settings-mounting_section = Mounting position
tracker-settings-mounting_section-description = Where is the tracker mounted?
tracker-settings-mounting_section-edit = Edit mounting
tracker-settings-drift_compensation_section = Allow drift compensation
tracker-settings-drift_compensation_section-description = Should this tracker compensate for its drift when drift compensation is enabled?
tracker-settings-drift_compensation_section-edit = Allow drift compensation
# The .<name> means it's an attribute and it's related to the top key.
# In this case that is the settings for the assignment section.
tracker-settings-name_section = Tracker name
tracker-settings-name_section-description = Give it a cute nickname :)
tracker-settings-name_section-placeholder = NightyBeast's left leg

## Tracker part card info
tracker-part_card-no_name = No name
tracker-part_card-unassigned = Unassigned

## Body assignment menu
body_assignment_menu = Where do you want this tracker to be?
body_assignment_menu-description = Choose a location where you want this tracker to be assigned. Alternatively you can choose to manage all trackers at once instead of one by one.
body_assignment_menu-show_advanced_locations = Show advanced assign locations
body_assignment_menu-manage_trackers = Manage all trackers
body_assignment_menu-unassign_tracker = Unassign tracker

## Tracker assignment menu
# A -translation_key (with a dash in the front) means that it's a label.
# It can only be used in the translation file, it's nice for reusing names and that kind of stuff.
# 
# We are using it here because english doesn't require changing the text in each case but
# maybe your language does.
-tracker_selection-part = Which tracker to assign to your
tracker_selection_menu-NONE = Which tracker do you want to be unassigned?
tracker_selection_menu-HEAD = { -tracker_selection-part } head?
tracker_selection_menu-NECK = { -tracker_selection-part } neck?
tracker_selection_menu-RIGHT_SHOULDER = { -tracker_selection-part } right shoulder?
tracker_selection_menu-RIGHT_UPPER_ARM = { -tracker_selection-part } right upper arm?
tracker_selection_menu-RIGHT_LOWER_ARM = { -tracker_selection-part } right lower arm?
tracker_selection_menu-RIGHT_HAND = { -tracker_selection-part } right hand?
tracker_selection_menu-RIGHT_UPPER_LEG = { -tracker_selection-part } right thigh?
tracker_selection_menu-RIGHT_LOWER_LEG = { -tracker_selection-part } right ankle?
tracker_selection_menu-RIGHT_FOOT = { -tracker_selection-part } right foot?
tracker_selection_menu-RIGHT_CONTROLLER = { -tracker_selection-part } right controller?
tracker_selection_menu-CHEST = { -tracker_selection-part } chest?
tracker_selection_menu-WAIST = { -tracker_selection-part } waist?
tracker_selection_menu-HIP = { -tracker_selection-part } hip?
tracker_selection_menu-LEFT_SHOULDER = { -tracker_selection-part } left shoulder?
tracker_selection_menu-LEFT_UPPER_ARM = { -tracker_selection-part } left upper arm?
tracker_selection_menu-LEFT_LOWER_ARM = { -tracker_selection-part } left lower arm?
tracker_selection_menu-LEFT_HAND = { -tracker_selection-part } left hand?
tracker_selection_menu-LEFT_UPPER_LEG = { -tracker_selection-part } left thigh?
tracker_selection_menu-LEFT_LOWER_LEG = { -tracker_selection-part } left ankle?
tracker_selection_menu-LEFT_FOOT = { -tracker_selection-part } left foot?
tracker_selection_menu-LEFT_CONTROLLER = { -tracker_selection-part } left controller?

tracker_selection_menu-unassigned = Unassigned Trackers
tracker_selection_menu-assigned = Assigned Trackers
tracker_selection_menu-dont_assign = Do not assign

## Mounting menu
mounting_selection_menu = Where do you want this tracker to be?
mounting_selection_menu-close = Close

## Sidebar settings
settings-sidebar-title = Settings
settings-sidebar-general = General
settings-sidebar-tracker_mechanics = Tracker mechanics
settings-sidebar-fk_settings = Tracking settings
settings-sidebar-gesture_control = Gesture control
settings-sidebar-interface = Interface
settings-sidebar-osc_router = OSC router
settings-sidebar-utils = Utilities
settings-sidebar-serial = Serial console

## SteamVR settings
settings-general-steamvr = SteamVR
settings-general-steamvr-subtitle = SteamVR trackers
# Not all translation keys support multiline, only the ones that specify it will actually
# split it in lines (that also means you can split in lines however you want in those).
# The first spaces (not tabs) for indentation will be ignored, just to make the file look nice when writing.
# This one is one of this cases that cares about multilines
settings-general-steamvr-description =
    Enable or disable specific SteamVR trackers.
    Useful for games or apps that only support certain trackers.
settings-general-steamvr-trackers-waist = Waist
settings-general-steamvr-trackers-chest = Chest
settings-general-steamvr-trackers-feet = Feet
settings-general-steamvr-trackers-knees = Knees
settings-general-steamvr-trackers-elbows = Elbows
settings-general-steamvr-trackers-hands = Hands

## Tracker mechanics
settings-general-tracker_mechanics = Tracker mechanics
settings-general-tracker_mechanics-filtering = Filtering
# This also cares about multilines
settings-general-tracker_mechanics-filtering-description =
    Choose the filtering type for your trackers.
    Prediction predicts movement while smoothing smoothens movement.
settings-general-tracker_mechanics-filtering-type = Filtering type
settings-general-tracker_mechanics-filtering-type-none = No filtering
settings-general-tracker_mechanics-filtering-type-none-description = Use rotations as is. Will not do any filtering.
settings-general-tracker_mechanics-filtering-type-smoothing = Smoothing
settings-general-tracker_mechanics-filtering-type-smoothing-description = Smooths movements but adds some latency.
settings-general-tracker_mechanics-filtering-type-prediction = Prediction
settings-general-tracker_mechanics-filtering-type-prediction-description = Reduces latency and makes movements more snappy, but may increase jitter.
settings-general-tracker_mechanics-filtering-amount = Amount
settings-general-tracker_mechanics-drift_compensation = Drift compensation
# This cares about multilines
settings-general-tracker_mechanics-drift_compensation-description =
    Compensates IMU yaw drift by applying an inverse rotation.
    Change amount of compensation and up to how many resets are taken into account.
settings-general-tracker_mechanics-drift_compensation-enabled-label = Drift compensation
settings-general-tracker_mechanics-drift_compensation-amount-label = Compensation amount
settings-general-tracker_mechanics-drift_compensation-max_resets-label = Use up to x last resets

## FK/Tracking settings
settings-general-fk_settings = Tracking settings
settings-general-fk_settings-leg_tweak = Leg tweaks
settings-general-fk_settings-leg_tweak-description = Floor-clip can Reduce or even eliminates clipping with the floor but may cause problems when on your knees. Skating-correction corrects for ice skating, but can decrease accuracy in certain movement patterns.
# Floor clip: 
# why the name - came from the idea of noclip in video games, but is the opposite where clipping to the floor is a desired feature
# definition - Prevents the foot trackers from going lower than they where when a reset was performed
settings-general-fk_settings-leg_tweak-floor_clip = Floor clip
# Skating correction: 
# why the name - without this enabled the feet will often slide across the ground as if your skating across the ground,
# since this largely prevents this it corrects for it hence skating correction (note this may be renamed to sliding correction)
# definition - Guesses when each foot is in contact with the ground and uses that information to improve tracking
settings-general-fk_settings-leg_tweak-skating_correction = Skating correction
settings-general-fk_settings-leg_tweak-skating_correction-amount = Skating correction strength
settings-general-fk_settings-arm_fk = Arm tracking
settings-general-fk_settings-arm_fk-description = Change the way the arms are tracked.
settings-general-fk_settings-arm_fk-force_arms = Force arms from HMD
settings-general-fk_settings-skeleton_settings = Skeleton settings
settings-general-fk_settings-skeleton_settings-description = Toggle skeleton settings on or off. It is recommended to leave these on.
settings-general-fk_settings-skeleton_settings-extended_spine = Extended spine
settings-general-fk_settings-skeleton_settings-extended_pelvis = Extended pelvis
settings-general-fk_settings-skeleton_settings-extended_knees = Extended knee
settings-general-fk_settings-vive_emulation-title = Vive emulation
settings-general-fk_settings-vive_emulation-description = Emulate the waist tracker problems that Vive trackers have. This is a joke and makes tracking worse.
settings-general-fk_settings-vive_emulation-label = Enable Vive emulation

## Gesture control settings (tracker tapping)
settings-general-gesture_control = Gesture control
settings-general-gesture_control-subtitle = Tap based resets
settings-general-gesture_control-description = Allows for resets to be triggered by tapping a tracker. The tracker highest up on your torso is used for Quick Reset, the tracker highest up on your left leg is used for Reset, and the tracker highest up on your right leg is used for Mounting Reset. It should be mentioned that taps must happen within 0.6 seconds to be registered.
# This is a unit: 3 taps, 2 taps, 1 tap
# $amount (Number) - Amount of taps (touches to the tracker's case)
settings-general-gesture_control-taps = { $amount ->
    [one] 1 tap
    *[other] { $amount } taps
}
settings-general-gesture_control-quickResetEnabled = Enable tap to quick reset
settings-general-gesture_control-quickResetDelay = Quick reset delay
settings-general-gesture_control-quickResetTaps = Taps for quick reset
settings-general-gesture_control-resetEnabled = Enable tap to reset
settings-general-gesture_control-resetDelay = Reset delay
settings-general-gesture_control-resetTaps = Taps for reset
settings-general-gesture_control-mountingResetEnabled = Enable tap to reset mounting
settings-general-gesture_control-mountingResetDelay = Mounting reset delay
settings-general-gesture_control-mountingResetTaps = Taps for mounting reset

## Interface settings
settings-general-interface = Interface
settings-general-interface-dev_mode = Developer Mode
settings-general-interface-dev_mode-description = This mode can be useful if you need in-depth data or to interact with connected trackers on a more advanced level.
settings-general-interface-dev_mode-label = Developer Mode
settings-general-interface-serial_detection = Serial device detection
settings-general-interface-serial_detection-description = This option will show a pop-up every time you plug a new serial device that could be a tracker. It helps improving the setup process of a tracker.
settings-general-interface-serial_detection-label = Serial device detection
settings-general-interface-lang = Select language
settings-general-interface-lang-description = Change the default language you want to use.
settings-general-interface-lang-placeholder = Select the language to use

## Serial settings
settings-serial = Serial Console
# This cares about multilines
settings-serial-description =
    This is a live information feed for serial communication.
    May be useful if you need to know the firmware is acting up.
settings-serial-connection_lost = Connection to serial lost, Reconnecting...
settings-serial-reboot = Reboot
settings-serial-factory_reset = Factory Reset
settings-serial-get_infos = Get Infos
settings-serial-serial_select = Select a serial port
settings-serial-auto_dropdown_item = Auto

## OSC router settings
settings-osc-router = OSC router
# This cares about multilines
settings-osc-router-description =
    Forward OSC messages from another program.
    Useful for using another OSC program with VRChat for example.
settings-osc-router-enable = Enable
settings-osc-router-enable-description = Toggle the forwarding of messages.
settings-osc-router-enable-label = Enable
settings-osc-router-network = Network ports
# This cares about multilines
settings-osc-router-network-description =
    Set the ports for listening and sending data.
    These can be the same as other ports used in the SlimeVR server.
settings-osc-router-network-port_in =
    .label = Port In
    .placeholder = Port in (default: 9002)
settings-osc-router-network-port_out =
    .label = Port Out
    .placeholder = Port out (default: 9000)
settings-osc-router-network-address = Network address
settings-osc-router-network-address-description = Set the address to send out data at.
settings-osc-router-network-address-placeholder = IPV4 address

## OSC VRChat settings
settings-osc-vrchat = VRChat OSC Trackers
# This cares about multilines
settings-osc-vrchat-description =
    Change VRChat-specific settings to receive HMD data and send
    trackers data for FBT (works on Quest standalone).
settings-osc-vrchat-enable = Enable
settings-osc-vrchat-enable-description = Toggle the sending and receiving of data.
settings-osc-vrchat-enable-label = Enable
settings-osc-vrchat-network = Network ports
settings-osc-vrchat-network-description = Set the ports for listening and sending data to VRChat.
settings-osc-vrchat-network-port_in =
    .label = Port In
    .placeholder = Port in (default: 9001)
settings-osc-vrchat-network-port_out =
    .label = Port Out
    .placeholder = Port out (default: 9000)
settings-osc-vrchat-network-address = Network address
settings-osc-vrchat-network-address-description = Choose which address to send out data to VRChat (check your wifi settings on your device).
settings-osc-vrchat-network-address-placeholder = VRChat ip address
settings-osc-vrchat-network-trackers = Trackers
settings-osc-vrchat-network-trackers-description = Toggle the sending of specific trackers via OSC.
settings-osc-vrchat-network-trackers-chest = Chest
settings-osc-vrchat-network-trackers-waist = Waist
settings-osc-vrchat-network-trackers-knees = Knees
settings-osc-vrchat-network-trackers-feet = Feet
settings-osc-vrchat-network-trackers-elbows = Elbows

## Setup/onboarding menu
onboarding-skip = Skip setup
onboarding-continue = Continue
onboarding-wip = Work in progress

## WiFi setup
onboarding-wifi_creds-back = Go Back to introduction
onboarding-wifi_creds = Input WiFi credentials
# This cares about multilines
onboarding-wifi_creds-description =
    The Trackers will use these credentials to connect wirelessly.
    Please use the credentials that you are currently connected to.
onboarding-wifi_creds-skip = Skip wifi settings
onboarding-wifi_creds-submit = Submit!
onboarding-wifi_creds-ssid =
    .label = SSID
    .placeholder = Enter SSID
onboarding-wifi_creds-password =
    .label = Password
    .placeholder = Enter password

## Mounting setup
onboarding-reset_tutorial-back = Go Back to Mounting calibration
onboarding-reset_tutorial = Reset tutorial
onboarding-reset_tutorial-description = This feature isn't done, just press continue

## Setup start
onboarding-home = Welcome to SlimeVR
# This cares about multilines and it's centered!!
onboarding-home-description =
    Bringing full-body tracking
    to everyone
onboarding-home-start = Lets get set up!

## Enter VR part of setup
onboarding-enter_vr-back = Go Back to Tracker assignent
onboarding-enter_vr-title = Time to enter VR!
onboarding-enter_vr-description = Put on all your trackers and then enter VR!
onboarding-enter_vr-ready = I'm ready

## Setup done
onboarding-done-title = You're all set!
onboarding-done-description = Enjoy your full body experience
onboarding-done-close = Close the guide

## Tracker connection setup
onboarding-connect_tracker-back = Go Back to WiFi credentials
onboarding-connect_tracker-title = Connect trackers
onboarding-connect_tracker-description-p0 = Now onto the fun part, connecting all the trackers!
onboarding-connect_tracker-description-p1 = Simply connect all that aren't connected yet, through a USB port.
onboarding-connect_tracker-issue-serial = I'm having trouble connecting!
onboarding-connect_tracker-usb = USB Tracker
onboarding-connect_tracker-connection_status-connecting = Sending wifi credentials
onboarding-connect_tracker-connection_status-connected = Connected to WiFi
onboarding-connect_tracker-connection_status-error = Unable to connect to Wifi
onboarding-connect_tracker-connection_status-start_connecting = Looking for trackers
onboarding-connect_tracker-connection_status-handshake = Connected to the Server
# $amount (Number) - Amount of trackers connected (this is a number, but you can use CLDR plural rules for your language)
# More info on https://www.unicode.org/cldr/cldr-aux/charts/22/supplemental/language_plural_rules.html
# English in this case only has 2 plural rules, which are "one" and "other",
# we use 0 in an explicit way because there is no plural rule in english for 0, so we directly say
# if $amount is 0 then we say "No trackers connected"
onboarding-connect_tracker-connected_trackers = { $amount ->
    [0] No trackers
    [one] 1 tracker
    *[other] { $amount } trackers
} connected
onboarding-connect_tracker-next = I connected all my trackers

## Tracker assignment setup
onboarding-assign_trackers-back = Go Back to Wifi Credentials
onboarding-assign_trackers-title = Assign trackers
onboarding-assign_trackers-description = Let's choose which tracker goes where. Click on a location where you want to place a tracker
# Look at translation of onboarding-connect_tracker-connected_trackers on how to use plurals
# $assigned (Number) - Trackers that have been assigned a body part
# $trackers (Number) - Trackers connected to the server
onboarding-assign_trackers-assigned = { $assigned } of { $trackers ->
    [one] 1 tracker
    *[other] { $trackers } trackers
} assigned
onboarding-assign_trackers-advanced = Show advanced assign locations
onboarding-assign_trackers-next = I assigned all the trackers

## Tracker manual mounting setup
onboarding-manual_mounting-back = Go Back to Enter VR
onboarding-manual_mounting = Manual Mounting
onboarding-manual_mounting-description = Click on every tracker and select which way they are mounted
onboarding-manual_mounting-auto_mounting = Automatic mounting
onboarding-manual_mounting-next = Next step

## Tracker automatic mounting setup
onboarding-automatic_mounting-back = Go Back to Enter VR
onboarding-automatic_mounting-title = Mounting Calibration
onboarding-automatic_mounting-description = For SlimeVR trackers to work, we need to assign a mounting rotation to your trackers to align them with your physical tracker mounting.
onboarding-automatic_mounting-manual_mounting = Manually set mounting
onboarding-automatic_mounting-next = Next step
onboarding-automatic_mounting-prev_step = Previous step
onboarding-automatic_mounting-done-title = Mounting rotations calibrated.
onboarding-automatic_mounting-done-description = Your mounting calibration is complete!
onboarding-automatic_mounting-done-restart = Return to start
onboarding-automatic_mounting-mounting_reset-title = Mounting Reset
onboarding-automatic_mounting-mounting_reset-step-0 = 1. Squat in a "skiing" pose with your legs bent, your upper body tilted forwards, and your arms bent.
onboarding-automatic_mounting-mounting_reset-step-1 = 2. Press the "Reset Mounting" button and wait for 3 seconds before the trackers' mounting rotations will reset.
onboarding-automatic_mounting-preparation-title = Preparation
onboarding-automatic_mounting-preparation-step-0 = 1. Stand upright with your arms to your sides.
onboarding-automatic_mounting-preparation-step-1 = 2. Press the "Reset" button and wait for 3 seconds before the trackers will reset.
onboarding-automatic_mounting-put_trackers_on-title = Put on your trackers
onboarding-automatic_mounting-put_trackers_on-description = To calibrate mounting rotations, we're gonna use the trackers you just assigned. Put on all your trackers, you can see which are which in the figure to the right.
onboarding-automatic_mounting-put_trackers_on-next = I have all my trackers on

## Tracker manual proportions setup
onboarding-manual_proportions-back = Go Back to Reset tutorial
onboarding-manual_proportions-title = Manual Body Proportions
onboarding-manual_proportions-precision = Precision adjust
onboarding-manual_proportions-auto = Automatic calibration

## Tracker automatic proportions setup
onboarding-automatic_proportions-back = Go Back to Reset tutorial
onboarding-automatic_proportions-title = Measure your body
onboarding-automatic_proportions-description = For SlimeVR trackers to work, we need to know the length of your bones. This short calibration will measure it for you.
onboarding-automatic_proportions-manual = Manual calibration
onboarding-automatic_proportions-prev_step = Previous step
onboarding-automatic_proportions-put_trackers_on-title = Put on your trackers
onboarding-automatic_proportions-put_trackers_on-description = To calibrate your proportions, we're gonna use the trackers you just assigned. Put on all your trackers, you can see which are which in the figure to the right.
onboarding-automatic_proportions-put_trackers_on-next = I have all my trackers on
onboarding-automatic_proportions-preparation-title = Preparation
onboarding-automatic_proportions-preparation-description = Place a chair directly behind you inside your play space. Be prepared to sit down during the autobone setup.
onboarding-automatic_proportions-preparation-next = I am in front of a chair
onboarding-automatic_proportions-start_recording-title = Get ready to move
onboarding-automatic_proportions-start_recording-description = We're now going to record some specific poses and moves. These will be prompted in the next screen. Be ready to start when the button is pressed!
onboarding-automatic_proportions-start_recording-next = Start Recording
onboarding-automatic_proportions-recording-title = REC
onboarding-automatic_proportions-recording-description-p0 = Recording in progress...
onboarding-automatic_proportions-recording-description-p1 = Make the moves shown below:
onboarding-automatic_proportions-recording-steps-0 = Bend knees a few times.
onboarding-automatic_proportions-recording-steps-1 = Sit on a chair then stand up.
onboarding-automatic_proportions-recording-steps-2 = Twist upper body left, then bend right.
onboarding-automatic_proportions-recording-steps-3 = Twist upper body right, then bend left.
onboarding-automatic_proportions-recording-steps-4 = Wiggle around until timer ends.
onboarding-automatic_proportions-recording-processing = Processing the result
# $time (Number) - Seconds left for the automatic calibration recording to finish (max 15)
onboarding-automatic_proportions-recording-timer = { $time ->
    [one] 1 second left
    *[other] { $time } seconds left
}
onboarding-automatic_proportions-verify_results-title = Verify results
onboarding-automatic_proportions-verify_results-description = Check the results below, do they look correct?
onboarding-automatic_proportions-verify_results-results = Recording results
onboarding-automatic_proportions-verify_results-processing = Processing the result
onboarding-automatic_proportions-verify_results-redo = Redo recording
onboarding-automatic_proportions-verify_results-confirm = They're correct
onboarding-automatic_proportions-done-title = Body measured and saved.
onboarding-automatic_proportions-done-description = Your body proportions calibration is complete!

## Home
home-no_trackers = No trackers detected or assigned
