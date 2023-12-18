# Edurob-Teaching-Units
Collection of teaching units and teaching materials for Edurob

# Pathplanning Edurob
In this teaching unit, the Edurob should be used to explore and compare various kinematic models in the context of driving along a defined path. The defined paths for Edurob are specified through a CSV file, containing pose (x, y, theta) and world velocities (x, y, theta).

## Project Overview

Our objective is to investigate different kinematic models, utilizing pre-defined paths encoded in a CSV file. By using the same predefined path with different kinematics it is possible to compare and contrast the performance of various kinematic approaches
![](.\Pathplanning_edurob\pathplaner\img\robot_path.png)

## Getting Started
[Homepage](https://www.imsl.fh-dortmund.de/mobile-roboter/edurob/)

## Initial Setup

- Install Visual Studio Code: https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/#1
- Install PlatformIO extension: https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/#2
- Clone the Repository and open the "edurob" folder with PlatformIO
- Depending on your hardware configuration (Mecanum, Differential drive, 3-Wheel Omni or 4-Wheel Omni) you have to select the corresponding software configuration by uncommenting **only one** of the following statements within the "parameter.h"

    `#define MECANUM`
    
    `#define DIFF`

    `#define OMNI4`

    `#define OMNI3`

- Plan a Path with the JupyterNotebook provided within the "pathplaner" folder. The Jupyter Notebook shows one specific scenario that can be used as a guideline on how to create your path
- After the CSV is generated it has to be flashed to the ESP on the Edurob: https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/
- To read the path from the CSV uncomment the `#define read_file_mode` after flashing the CSV, so the Edurob can follow the predesigned path.

