
# Applied Medical Robotics - Group 8

This repository contains the project files for **Applied Medical Robotics Group 8**, focusing on developing innovative solutions in medical robotics.

## Table of Contents

- [Project Overview](#project-overview)
- [Folder Structure](#folder-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Project Overview

The goal of this project is to design and implement a medical robotics system that enhances efficiency, accuracy, and safety in healthcare applications.

## Folder Structure

- **1_PID/**: Files for PID controller implementation.
- **V1 CAD/**, **V2 CAD/**, **V3 CAD/**: Different versions of CAD designs.
- **genadata_PID2/**: Data files for PID version 2.
- **src/**: Contains scripts for forward and inverse kinematics (`forward_kinematics.m`, `inverse_kinematics.m`).
- **scripts/**: MATLAB GUI files (`official_gui.m`, `official_gui_draw.m`).
- **data/**: Workspace and figure files (`workspace.m`, `workspace figure.fig`).
- **docs/**: Circuit diagrams and other documentation (`Circuit diagram.png`).
- **tests/**: Test cases and validation files.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Lucas-G-Xu/Applied-Medical-Robotics-Group-8.git
   ```

2. Navigate to the project folder:
   ```bash
   cd Applied-Medical-Robotics-Group-8
   ```

3. Install dependencies:
   - **MATLAB**: Required for running `.m` files.
   - **Arduino IDE**: Needed for uploading `.ino` files to the microcontroller.

## Usage

1. Open the MATLAB GUI:
   ```matlab
   open official_gui.m
   ```

2. Upload Arduino sketches:
   - Use `gendata.ino` or `gendata_PID.ino` in the Arduino IDE and upload to your microcontroller.

3. Run kinematics scripts:
   ```matlab
   run forward_kinematics.m
   ```

4. Use test files to validate functionalities:
   ```bash
   pytest tests/
   ```

## Contributing

We welcome contributions! Please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Commit your changes:
   ```bash
   git commit -m "Add your feature"
   ```
4. Push to your branch:
   ```bash
   git push origin feature/your-feature-name
   ```
5. Open a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
