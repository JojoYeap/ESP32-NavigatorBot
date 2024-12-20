# ESP32-NavigatorBot: ESP32 GPS Bluetooth Robot Car

<img src="https://github.com/JojoYeap/ESP32-NavigatorBot/blob/main/images/Car_Main.png" alt="ESP32-NavigatorBot" width="300">

This repository contains the code for an **ESP32-based robot car** that integrates GPS and Bluetooth capabilities. The robot receives GPS coordinates via Bluetooth from a mobile app and autonomously navigates to the specified location.

## Features
- **Bluetooth Communication**: Receive GPS coordinates from a mobile app.
- **GPS Integration**: Navigate to the provided GPS coordinates.
- **Real-Time Processing**: Calculate direction and control motors dynamically.
- **OLED Display**: Show status updates on an SSD1306 display.
- **Magnetometer Support**: Use a compass for orientation adjustments.

## Hardware Requirements
- **ESP32 microcontroller**
- **NEO-6M GPS module**
- **Bluetooth-enabled app/device**
- **HMC5883L magnetometer**
- **OLED display (SSD1306)**
- **Motor drivers**
- **Motors and chassis**

## Software Requirements
- **Arduino IDE** (for code compilation and uploading)
- **Libraries**:
  - `Adafruit_GFX`
  - `Adafruit_SSD1306`
  - `TinyGPS++`
  - `Adafruit_HMC5883_U`
  - `BluetoothSerial`

## Setup Instructions
1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/JojoYeap/ESP32-NavigatorBot.git
2. Open the .ino file in the Arduino IDE.
3. Install the required libraries via the Arduino Library Manager.
4. Upload the code to the ESP32 board.
5. Connect the GPS module, magnetometer, OLED display, and motors as per the pin configurations in the code.
6. Download the mobile app file CAR_APP.aia, and open it with MIT App Inventor to compile and install it on your mobile device.

## Usage
1. Power the robot and ensure the Bluetooth module is enabled.
2. Connect your mobile app to the robot via Bluetooth.
3. Send GPS coordinates in the format [latitude, longitude].
4. Watch as the robot navigates autonomously towards the target location!

## Repository Structure
- `ArduinoCode.ino`: The main Arduino sketch for the robot car.
- `CAR_APP.aia`: The MIT App Inventor project file for the mobile app.
  
## Future Enhancements
- Add obstacle detection and avoidance.
- Improve route optimization algorithms.
- Implement web-based control for enhanced usability.

## License
This project is licensed under the MIT License.
