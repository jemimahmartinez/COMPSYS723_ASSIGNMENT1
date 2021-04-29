# Instructions
Instructions for running this assignment on a DE2-115 board:

- Program the Nios II DE2-115 board using Quartus
    - Navigate to Quartus (what's the version number) > create a new project 
    - Programmer > add a new file (freq_relay_controller.sof file) 
    - Make sure that the Hardware Setup connected is the ‘USB Blaster’ hardware > Start
    - Progress should say 100% to indicate programming was successful
- Run the project on Nios II Eclipse
    - Navigate to Nios II 13.0sp1Software Build Tools for Eclipse 
    - Create an empty Nios II project by clicking on File > New > Nios II Application and BSP from Template > Create a Blank Project 
    - Import `main.c` into the main project folder
    - Build the project by right clicking on the project folder 
    - Navigate to Run > Run Configurations > Nios II Hardware > Create a New Configuration
        - Ensure that the ‘Project Name’ is what you set it
        - Navigate to Target Connection > Ensure that ‘USB-Blaster on localhost [USB-0]’ is the Connection Cables for Processors and Byte Stream Devices > Apply these changes > Run
