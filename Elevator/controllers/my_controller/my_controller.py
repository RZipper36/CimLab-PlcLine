import sys
import time
import argparse
try:
    from controller import Robot, Keyboard, Supervisor,TouchSensor
except ImportError:
    print("Didn't import webots controller libraries")
try:
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    print("INFO: use ctrl+c for exit")
except ImportError:
    GPIO = None  # GPIO is not available if not running on a Raspberry Pi
    print("INFO: use 'q' for exit")

DEFAULT_TIMESTEP = 64  # milliseconds


#############################################################################################################
# The Environment class is responsible for setting up and managing the environment in which the
# elevator operates. It supports both physical and simulation modes.

# Attributes:
#     simulation_mode (str): Specifies the mode of operation for the elevator system. Acceptable
#                            values are 'phys' for physical mode and "simu" for simulation mode.
#     elevator1 (ElevatorController or PhysicalElevatorController): The elevator controller object
#                            that manages the elevator's operations based on the specified simulation mode.

# Methods:
#     __init__(self, simulation_mode): Initializes the Environment with the specified simulation mode,
#                                      setting up the appropriate type of elevator controller.
#     run(self): Starts the operation of the elevator within the environment. Handles any exceptions
#                that occur during operation and ensures cleanup of resources.
#############################################################################################################       
class Environment:
    DEFAULT_POSITION_VECTOR = [0, 0, 1.59] # Don't change me
    def __init__(self,simulation_mode):
        if simulation_mode == 'phys':
            # Instantiate a PhysicalElevatorController for operating in a physical environment
            self.elevator1 = PhysicalElevatorController(
            elevator_id=1,
            pos_vector=self.DEFAULT_POSITION_VECTOR, # Initial position vector of the elevator
            levels= {
                1: 0,    # Ground level
                2: 1,    # Second level
                3: 2     # Third level
            },
            simulation_mode =simulation_mode
            )
        #simulation_mode is simu
        else:
            self.elevator1 = ElevatorController(
            elevator_id=1,
            pos_vector=self.DEFAULT_POSITION_VECTOR, # Initial position vector of the elevator
            levels= {
                1: 0,    # Ground level
                2: 1,    # Second level
                3: 2     # Third level
            },
            simulation_mode = simulation_mode
            )
        
    def run(self):
        try:
            self.elevator1.run()
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            None
            
##############################################################################################
    
   # ElevatorController is the FSM handles the operations and management of an elevator in either a simulation environment or a physical setup. 
   # It controls the elevator's movement between floors, the opening and closing of doors, and the initialization of the elevator system.

   # Main attributes:
   #     pos_vector (list): The initial position vector of the elevator in the simulation environment.
   #     levels (dict): A mapping of floor levels to their respective height positions.
   #     simulation_mode (str): The mode of operation, either 'simulation' for virtual environments or 'physical' for PlcLine-Raspberry Pi.
   #     state (str): The current state of the elevator ('idle', 'in_level', 'door_open').
   #     timestep (int): The simulation timestep in milliseconds.
   #     robot (Supervisor or None): An instance of the Supervisor class when in simulation mode, used to interact with the virtual environment.
   #     keyboard (Keyboard): An instance of the Keyboard class for receiving user input.
   #     elev (Device): The elevator motor device.
   #     door (Device): The door motor device.
   #     touch_sensor, touch_sensor2, touch_sensor3 (TouchSensor): Sensors for detecting the elevator's current floor.
   #     Tsensors_dict (dict): A dictionary mapping floor numbers to their respective touch sensors.
   #     door_touch_sensor (TouchSensor): A sensor to detect if the elevator door has made contact with an object or is fully opened.
   #     door_close_touch_sensor (TouchSensor): A sensor to detect if the elevator door is fully closed.
   
   # Methods:
   #     __init__(self, elevator_id, pos_vector, levels, simulation_mode): Initializes a new instance of the ElevatorController.
   #     run(self): The FSM operation.
   #     move_to_floor(self, floor): Moves the elevator to the specified floor.
   #     open_the_door(self): Opens the elevator door.
   #     close_the_door(self): Closes the elevator door.
    
##############################################################################################
class ElevatorController:
    def __init__(self, elevator_id, pos_vector, levels,simulation_mode):
        self.simulation_mode = simulation_mode
        self.elevator_id = elevator_id
        self.pos_vector = pos_vector
        self.levels = levels
        
        if GPIO and self.simulation_mode == "simu":
            print("Error: you are using Raspberry Pi with simulation mode. use --help for instructions.")
            sys.exit(0)
        self.timestep = DEFAULT_TIMESTEP 
        print(f"running in {self.simulation_mode} mode")
        # set robot for webots simulation
        if "Supervisor" in globals():
            self.robot = Supervisor()
        else:
            self.robot = None
            
        self.state = "idle"
        self.green_color = 0x28EE1F
        self.white_color = 0xFFFFFF
        self.red_color = 0xEE1F1F  
        self.door_open_time = 0 
        self.door_stay_open = 2  # seconds
        self.target_height = 0
        self.led_light = None
        self.current_level = self.levels[next(iter(self.levels))]
        self.led_state = None
        self.behavior = None
        self.door_steps = 0.02
        self.elev_step = 0.03
        if self.robot:     
            struct = self.robot.getFromDef('robot{}'.format(self.elevator_id))
            struct_pos = struct.getField('translation')
            struct_pos.setSFVec3f(self.pos_vector)

            self.keyboard = Keyboard()
            self.keyboard.enable(self.timestep)  # Enable the keyboard with a time step
            
            # Get the elevator motor and position sensor
            self.elev = self.robot.getDevice('linear{}'.format(self.elevator_id))
            self.elev.setPosition(self.levels[next(iter(self.levels))])  # Initialize elevator to the ground floor
            
            # Door position definitions
            self.door_open_pos = 0.08
            self.door_close_pos = -0.14
            self.door = self.robot.getDevice('door{}'.format(self.elevator_id))
            self.door.setPosition(self.door_close_pos)  # Initialize door to the closed position       

            # Initialize display
            self.change_monitor(text_color=0x28EE1F,text="elev: "+str(self.elevator_id),font_size=8,init=True)
            
            # Initialize led
            self.led_light = self.robot.getDevice("led{}".format(self.elevator_id))
            self.led_light.set(1)   # green in the first position red in the second
            
            # Initialize touch sensor
            self.touch_sensor = self.robot.getDevice('touch_sensor{}'.format(self.elevator_id))
            self.touch_sensor.enable(self.timestep)
            self.touch_sensor2 = self.robot.getDevice('touch_sensor{}'.format(2))
            self.touch_sensor2.enable(self.timestep)
            self.touch_sensor3 = self.robot.getDevice('touch_sensor{}'.format(3))
            self.touch_sensor3.enable(self.timestep)
            self.Tsensors_dict={0:self.touch_sensor,1:self.touch_sensor2,2:self.touch_sensor3}
            
            # Initialize door touch sensor
            self.door_touch_sensor = self.robot.getDevice('door_touch_sensor{}'.format(self.elevator_id))
            self.door_touch_sensor.enable(self.timestep)
            self.door_close_touch_sensor = self.robot.getDevice('close_door_touch_sensor{}'.format(self.elevator_id))
            self.door_close_touch_sensor.enable(self.timestep)    
        
        
        
    ########################################################################################################################################        
    # run:
    # Continuously operates the elevator based on the FSM.

    # This method represents the core loop of the elevator's operation. It continuously checks for input commands
    # (e.g., floor selections from a user) and manages the elevator's movements and door operations according to its current state.

    # The elevator operation is divided into several states:
    # - 'idle': The elevator is waiting for input commands. If a valid floor selection is made, the elevator prepares to move to the selected floor.
    # - 'in_level': The elevator has arrived at a floor. It proceeds to open the door to allow passengers to enter or exit.
    # - 'door_open': After the door has been opened, this state ensures that the door remains open for a set duration before closing.
    #               This duration is measured from the `door_open_time` attribute, allowing for a delay to accommodate passengers.

    # The loop runs indefinitely, constantly checking for inputs and adjusting the elevator's state as necessary. During each iteration of the loop,
    # the method checks for a key press, translates it into a floor request if applicable, and adjusts the elevator's state based on the current state
    # and the received input. This might involve moving the elevator to a new floor, opening the door upon arrival, or closing the door after a set period.

    # In case of an exception, the method will catch it, allowing for error handling and cleanup. This ensures that the elevator can recover from unexpected issues
    # and that resources, such as GPIO pins in a physical implementation, are properly released upon termination.
    ########################################################################################################################################        

    def run(self):
        try:            
            # Check the simulation mode and adjust the loop condition accordingly
            if self.simulation_mode == 'simu':
                loop_condition = lambda: self.robot.step(self.timestep) != -1
            else:  # physical mode
                loop_condition = lambda: True
                
            key_floor,key = 0,0 # init the keys
            while loop_condition():
                # get key from user
                if self.state == 'idle' and key_floor==0:
                    key = self.keyboard.getKey()
                    if self.led_light and key:
                        self.led_state = False
                        self.behavior = 1                   
                        self.led_light.set(1)                        
                if not key:
                   key = 0
                key_floor = self.get_key_floor(mode=self.simulation_mode ,key=key)
                
                # idle -> in_level state if getting valid floor.    
                if self.state == 'idle':
                    if (key_floor in self.levels):
                        self.init_move_to_floor(key_floor=key_floor) 
                # in_level -> door_open state  
                elif self.state == "in_level":
                    self.init_open_the_door()
                # door_open -> idle   
                elif self.state == "door_open":
                    if (time.time() - self.door_open_time) >= self.door_stay_open:
                        self.close_the_door()  
                        key=0
                if self.simulation_mode == "phys":
                    time.sleep(0.2)  # A short delay to prevent physical button Debouncing  
        except Exception as e:
            print(f"An error occurred: {e}")                    
        finally:
            if GPIO:
                self.cleanup()
                
    # control the monotor display
    def change_monitor(self, text_color,text,font_size=30,init=False):
        if init:
            self.display = self.robot.getDevice("display{}".format(self.elevator_id)) 
            self.width = self.display.getWidth()
            self.height = self.display.getHeight()
            self.display.fillRectangle(0, 0, self.width, self.height)
            self.display.setColor(self.green_color)  
        else:
            self.display.setColor(self.white_color) 
            self.display.fillRectangle(0, 0, self.width, self.height)
            self.display.setColor(text_color)

        self.display.setFont("Arial", font_size, True)
        self.display.drawText(text, self.width / 4, self.height / 4)
    
    def init_move_to_floor(self,key_floor):
        self.target_height = self.levels[key_floor]
        if self.simulation_mode == "phys": # use the required led 
            self.led_light = self.led_floor_dict[self.target_height]
        self.led_state = True
        self.behavior = 2 
        self.led_light.set(self.behavior) 
        self.change_monitor(text_color=self.red_color,text=str(int(self.target_height + 1)))
        #choose the diraction of movment and the destination touch sensor
        direction = 1 if self.target_height > self.current_level else -1
        sensor =self.Tsensors_dict[self.target_height]
        self.move_to_floor(direction,sensor)
        
    def move_to_floor(self,direction,sensor):
        while self.robot.step(self.timestep)!=-1:
            if sensor.getValue() == 0:
                self.elev.setPosition(self.current_level+(self.elev_step*direction))
                self.current_level= self.current_level+(self.elev_step*direction) # save the current location all the time            
            if sensor.getValue() > 0:
                self.state = "in_level"
                break 
                
    def init_open_the_door(self):
        self.open_the_door()
        self.door_open_time = time.time()
        self.change_monitor(text_color=self.green_color,text=str(int(self.target_height + 1))) 
        self.led_state = True
        self.behavior = 1 
        self.led_light.set(self.behavior)
        self.state = "door_open"
        
    def open_the_door(self):      
        curr_pos = self.door_close_pos
        while self.robot.step(self.timestep)!=-1:
            if(self.door_touch_sensor.getValue() == 0):
                self.door.setPosition(curr_pos+self.door_steps)
                curr_pos = curr_pos+self.door_steps
            else:
                break
        
    def close_the_door(self):
        if self.simulation_mode == "simu":
            curr_pos = self.door_open_pos
            while self.robot.step(self.timestep)!=-1:
                if(self.door_close_touch_sensor.getValue() == 0):
                    self.door.setPosition(curr_pos-self.door_steps)
                    curr_pos = curr_pos-self.door_steps
                else:
                    break
        else:
            GPIO.output(self.door, GPIO.LOW) # voltage down when the door is closing 
                              
        self.state = "idle"
    ##########################################################################################################################    
    # get_key_floor:
    # Converts keyboard inputs into elevator floor requests or commands.

    # Parameters:
    #     key (int): The ASCII value of the pressed key, representing a user's input.
    #     mode (str): Indicates the operation mode of the elevator system. It can be either "simu" for simulation or "phys" for physical operation.

    # The function checks the operation mode:
    # - In "phys" mode, it directly returns the key as it is.
    # - In "simu" mode, it interprets the key presses to convert them into floor selections or commands:
    #     - If the 'Q' key is pressed (ASCII value 81), the function triggers a cleanup process by calling `self.cleanup()` and then exits the program. This provides a way to safely terminate the simulation.
    #     - Numeric keys (0-9), represented by ASCII values 48 to 57, are converted into corresponding floor numbers by subtracting 48 from the ASCII value. This allows users to select floors directly.
    #     - Uppercase letter keys (A-Z), represented by ASCII values 65 to 90, are converted into lowercase letters. This feature could be used for special commands or accessing floors beyond the numeric range.

    # Returns:
    #     key_floor: The interpreted floor number or command based on the user's input. This could be a numerical value for floor selection or a lowercase letter for special commands. If the input does not match any of the expected patterns, `None` is returned, indicating no action.    
    ##########################################################################################################################
    def get_key_floor(self,key=0,mode = "simu"):
        if mode == "phys":
            return key
        else:
            if key == ord('Q'):  # 'Q' key to quit the simulation
               self.cleanup()
               sys.exit(0)
            key_floor = 0
            if key >= 48 and key <= 57:
                key_floor = key - 48  # Numeric input: ASCII to floor
            elif key >= 65 and key <= 90:
                 key_floor = chr(key).lower()  # Character input
            return key_floor
    
    ###############################################
    # set outputs GPIO as low at the end of the run
    ###############################################
    def cleanup(self):
        print("\nCleaning up raspberry pi GPIO")
        if GPIO:
            for gpio in self.list_of_gpio_to_set_as_low:
               GPIO.output(gpio, False) 
                   
            
####################################################################################################
#
# This class extends the ElevatorController to implement control logic for a physical elevator system, 
# facilitated by a Raspberry Pi connected to a PLC (Programmable Logic Controller) board. The PlcLine board
# simulates the elevator's mechanics.
#
# The Raspberry Pi interacts with the PLC to manage the elevator's operations based on the
# physical inputs and the state of the elevator system.
#
# Attributes:
#     - LED_PIN_FLOOR_X (int): GPIO pin numbers assigned to LED indicators for each floor on the Raspberry Pi.
#     - GPIO_PIN_UP/DOWN (int): GPIO pin numbers for controlling the elevator's upward and downward movement signals sent to the PLC.
#     - touch_sensorX (int): GPIO pin numbers assigned to touch sensors for detecting the elevator's current floor, interfaced through the PLC.
#     - door_touch_sensor (int): GPIO pin for the sensor to detect if the elevator door is open.
#     - door (int): GPIO pin for controlling the elevator door motor signal sent to the PLC.
#     - Tsensors_dict (dict): Mapping of floor numbers to their respective touch sensor GPIO pins.
#     - keyboard (ModifiedKeyboard): A custom interface for input, potentially used for manual control.
#
# Methods:
#     __init__(self, elevator_id, pos_vector, levels, simulation_mode): Initializes the controller with the specified attributes,
#      sets up GPIO pins for interfacing with the PLC and peripherals, and configures the initial state of the elevator system.
#
#     change_monitor(self, text_color, text, font_size=30, init=False): This method is a overwrite the simu method.
#
#     run(self): The main operation loop for the elevator system. It handles the logic for floor selection,
#      moving the elevator to the selected floor, opening and closing the door, and responding to sensor inputs
#       to determine the elevator's current floor. This method orchestrates the communication between the
#        Raspberry Pi and the PlcLine to simulate a fully functional elevator system.   
       
####################################################################################################

class PhysicalElevatorController(ElevatorController):
    def __init__(self, elevator_id, pos_vector, levels,simulation_mode):
        # inheritance of ElevatorController methods and attributes
        super().__init__(elevator_id, pos_vector, levels, simulation_mode='phys')
        
        # GPIO initialization for Raspberry Pi
        if GPIO:
            GPIO.setmode(GPIO.BCM)
            ########### floors leds GPIO #####################
            self.LED_PIN_FLOOR_1 = 21  # GPIO pin for LED corresponding to floor 1
            self.LED_PIN_FLOOR_2 = 22  # GPIO pin for LED corresponding to floor 2
            self.LED_PIN_FLOOR_3 = 23  # GPIO pin for LED corresponding to floor 3
            # Set up GPIO pins for LEDs
            self.led_floor_1 = LedLight(self.LED_PIN_FLOOR_1,self)
            self.led_floor_2 = LedLight(self.LED_PIN_FLOOR_2,self)
            self.led_floor_3 = LedLight(self.LED_PIN_FLOOR_3,self)
            self.led_floor_dict = {0:self.led_floor_1, 1:self.led_floor_2, 2:self.led_floor_3}
            # Initialize LEDs to OFF state
            GPIO.output(self.LED_PIN_FLOOR_1, GPIO.LOW)
            GPIO.output(self.LED_PIN_FLOOR_2, GPIO.LOW)
            GPIO.output(self.LED_PIN_FLOOR_3, GPIO.LOW)
            
            ########### DC motor GPIO #####################
            self.GPIO_PIN_UP = 16  # GPIO pin for going up
            self.GPIO_PIN_DOWN = 18  # GPIO pin for going down
            GPIO.setup(self.GPIO_PIN_UP, GPIO.OUT)
            GPIO.setup(self.GPIO_PIN_DOWN, GPIO.OUT)
            GPIO.output(self.GPIO_PIN_UP, False)
            GPIO.output(self.GPIO_PIN_DOWN, False)
            
            ########### floor touch sensors GPIO###########
            self.touch_sensor = 24 # first floor touch sensor
            self.touch_sensor2 = 4 # second floor touch sensor
            self.touch_sensor3 = 26 # third floor touch sensor
            GPIO.setup(self.touch_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.touch_sensor2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.touch_sensor3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            
            ##### door touch sensors GPIO ###########
            self.door_touch_sensor = 3
            GPIO.setup(self.door_touch_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            
            ######### open door motor GPIO ##########
            self.door = 17
            GPIO.setup(self.door, GPIO.OUT)
            GPIO.output(self.door, GPIO.LOW)
        
        
        self.list_of_gpio_to_set_as_low =[21,22,23,16,18,17]
        self.Tsensors_dict={0:self.touch_sensor, 1:self.touch_sensor2, 2:self.touch_sensor3}        
        self.keyboard = ModifiedKeyboard() # overwrite the keyboard class of webots to get info from PlcLine
        print("INFO: The elevator will start the opartion from ground floor")
        self.move_to_floor(-1,self.Tsensors_dict[0]) # init first positation of the elevator to ground floor
        self.state = "idle"
        
    ##################################################################
    # overwrite change_monitor function from simu due to the lack of physical screen
    ##################################################################
    def change_monitor(self, text_color,text,font_size=30,init=False):
        return None
        
    ##################################################################
    # overwrite move_to_floor function from simu due to difreences in physicals actuators movments 
    ##################################################################
    def move_to_floor(self,direction,sensor):
        while not GPIO.input(sensor):  # elevator move till it reache the touch sensor in the destination floor
            if direction == 1:  #up
                GPIO.output(self.GPIO_PIN_UP, True)
                GPIO.output(self.GPIO_PIN_DOWN, False)
            elif direction == -1:  #down
                GPIO.output(self.GPIO_PIN_UP, False)
                GPIO.output(self.GPIO_PIN_DOWN, True)
            time.sleep(0.05)
            
        #stop the motor
        GPIO.output(self.GPIO_PIN_UP, GPIO.LOW)
        GPIO.output(self.GPIO_PIN_DOWN, GPIO.LOW)
        self.current_level = self.target_height
        self.state = "in_level"       
   
    def open_the_door(self):      
        while True:
            if not GPIO.input(self.door_touch_sensor):
                GPIO.output(self.door, GPIO.HIGH)
                time.sleep(self.door_steps)  
                GPIO.output(self.door, GPIO.LOW)
                return
            else:
                GPIO.output(self.door, GPIO.HIGH) # leave open
            return
                

            
######################################################
# This class replace the LED node object at WEBOTS
# It allow us to use diffrent behavaros of the leds on the PlcLine

# Methods:
#   __init__(self, pin): Initializes the LED for the spesific floor depend on the GPIO pin
#   set(self): activated the LED by required behavior(1 for on, 2 for blomking) and led_state (1 is on 2 is off)
#   start_blinking(self): LED start blinking 

######################################################
class LedLight():
    def __init__(self,pin,elevator):
        self.pin = pin
        self.elevator = elevator
        if GPIO:
            GPIO.setup(self.pin, GPIO.OUT)
            GPIO.output(self.pin, GPIO.LOW)  # Initialize LED to OFF state

    def set(self,behavior=None):
        if GPIO and self.elevator.behavior == 1:
            GPIO.output(self.pin, self.elevator.led_state)
        elif GPIO and self.elevator.behavior == 2:
            if self.elevator.led_state == True:
                self.start_blinking()
            else:
                GPIO.output(self.pin, GPIO.LOW)
    def start_blinking(self):
        for i in range (4):
            GPIO.output(self.pin, GPIO.HIGH)
            time.sleep(0.2)  # LED ON for 1 second
            GPIO.output(self.pin, GPIO.LOW)
            time.sleep(0.2)  # LED OFF for 1 second
            

 ######################################################
# This class replace the keyboard from the controller package at WEBOTS
# It allow us to use get input from the PlcLine by the user

# Methods:
#   __init__(self, pin): Initializes the GPIO pins that get the floor signal from the PlcLine
#   getKey(self): equivalent to the getKey finction from the controller package, used as a wrapper to get_floor_from_gpio function 
#   get_floor_from_gpio(self): check the state of each level button and return the required floor. 

######################################################           
  
class ModifiedKeyboard():
    def __init__(self):
        
        # GPIO initialization for Raspberry Pi
        if GPIO:
            self.GPIO_PIN_FLOOR_1 = 11  # GPIO pin for floor 1 button
            self.GPIO_PIN_FLOOR_2 = 12  # GPIO pin for floor 2 button
            self.GPIO_PIN_FLOOR_3 = 13  # GPIO pin for floor 3 button

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.GPIO_PIN_FLOOR_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.GPIO_PIN_FLOOR_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.GPIO_PIN_FLOOR_3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def getKey(self):
        floor_from_gpio = self.get_floor_from_gpio()
        if floor_from_gpio is not None: 
            return floor_from_gpio
        return 0
        

    def get_floor_from_gpio(self):
        # Check the status of each floor button
        floor_1_pressed = GPIO.input(self.GPIO_PIN_FLOOR_1)
        floor_2_pressed = GPIO.input(self.GPIO_PIN_FLOOR_2)
        floor_3_pressed = GPIO.input(self.GPIO_PIN_FLOOR_3)

        if floor_1_pressed == GPIO.HIGH:
            return 1
        elif floor_2_pressed == GPIO.HIGH:
            return 2
        elif floor_3_pressed == GPIO.HIGH:
            return 3
        return 0
        

###########################################################################################################
#This script allows the user to run an elevator simulation in either a physical mode or a simulation mode.
#The mode is specified through the command-line arguments.

#Usage:
#    To run the simulation in simulation mode (default):
#    python elevator_simulation.py --mode simu /or/ run from WEBOTS app

#    To run the simulation in physical mode:
#    python elevator_simulation.py --mode phys

#Arguments:
#    --mode: Specifies the mode of the elevator simulation. The available modes are:
#        - 'phys': Represents the physical mode where the elevator operates on the PlcLine.
#        - 'simu': Represents the simulation mode where the elevator operation is WEBOTS simulation.
###########################################################################################################        
if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser(description="Elevator Simulation\n  | Exit for UNIX/physical: CTRL+c | Exit for simulation: q")
        parser.add_argument("--mode", choices=["phys", "simu"], default="simu",
                            help="Specify the mode of the elevator simulation (physical/simulation)")
        args = parser.parse_args()

        # Instantiate and run the Environment with the specified simulation mode
        environment = Environment(simulation_mode=args.mode)
        environment.run()
    except KeyboardInterrupt:
        if GPIO: #free GPIO resources
            GPIO.output(16, False)
            GPIO.output(17, False)
            GPIO.output(18, False)
            GPIO.output(21, False)
            GPIO.output(22, False)
            GPIO.output(23, False)           
        print("\nThank you, goodbye!!!")    
        sys.exit(0)
