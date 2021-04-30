# Instructions
Instructions for running this assignment on a DE2-115 board:

- Program the Nios II DE2-115 board using Quartus
    - Navigate to Quartus II 13.0sp1
    - Programmer > add a new file (freq_relay_controller.sof file) 
    - Make sure that the Hardware Setup connected is the ‘USB Blaster’ hardware > Start
    - Progress should say 100% to indicate programming was successful
- Run the project on Nios II Eclipse
    - Navigate to Nios II 13.0sp1 Software Build Tools for Eclipse 
    - Navigate to the workspace directory you want to work in
    - Right click in the 'Project Explorer' space > Import > General > Existing Projects into Workspace > Next > Select the top-level directory that contains the `a1` folder and the `a1_bsp` folder as the 'root directory' 
    - Ensure that both folders `a1` and `a1_bsp` folders are ticked > Finish
    - Build the project by right clicking on the main project folder, `a1`
    - Navigate to Run > Run Configurations > Nios II Hardware > Create a New Configuration
        - In the Project tab: 
            - Ensure that the ‘Project Name’ is what you set it
            - Ensure that the `ELF` file of the build that you just ran has been selected as the 'Project ELF file name'
        - In the Target Connection tab:
            - Ensure that ‘USB-Blaster on localhost [USB-0]’ is the Connection Cables for Processors and Byte Stream Devices. If not, then refresh the connections till it is
            - Apply these changes > Run
