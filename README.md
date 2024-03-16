# CimLab-PlcLine
The code for CIM lab - PlcLine elevator project 

**Prerequisites**
  - Webots R2023a 
  - Python 3.7 or newer

**Installing**
1. Save the repository to your local machine.
2. Download the required Webots version.
3. Open Webots.

**Running the Simulation**
1. Open your Webots world (File -> Open World).
2. Select the simulation world you wish to run from the `worlds` directory - 'elevator'.
3. Press the play '>' button to start the simulation.
4. Reset the simulation using the '|<<' button.
5. Stop the simulation using the '||' button.
6. (Optional) lock/unlock the viewoint (View -> Scene Interactions -> Lock Viewpoint)
7. Activeted the elevator simulation using "1" "2" "3" keys. 
8. Use "q" to exit from controller. the simlation will not stop - use step 5 to stop it. 

**Raspberry Pi use**
1. Copט the controller code to your Raspberry Pi using a USB flash drive.
2. Open a UNIX terminls from the Raspberry Pi.
3. Use the "cd" command to change the directory to where the code is. Use the "ls -ltr" command to list the directory contents.
4. Check the help menu by executing "python <code_name>.py --help".
5. Run the elevator simulation.
6. Control the elevator behavior using the PlcLine control panel.
7. Use "ctrl+c" to exit from the activation.

**Acknowledgments**
Gad Halevy - CIM Lab - TAU
