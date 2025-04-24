# GPS tracking
 Location tracking using GPS modulle, mpu6050 and Kalman filter
This project enables real-time GPS location tracking using an ESP32 microcontroller, GPS module, and MPU6050 sensor. It publishes the live coordinates to an MQTT broker, which are then visualized on a world map using Node-RED. The system uses sensor fusion techniques to improve the accuracy of location data through a Kalman filter.

🧠 Features
📡 Live GPS Tracking
Reads real-time GPS data using UART and processes it on the ESP32.

🎯 Sensor Fusion with MPU6050
Uses accelerometer and gyroscope data with a Kalman filter to smooth GPS position updates.

🌐 WiFi + MQTT Communication
Publishes fused location data to an MQTT topic over WiFi.

🗺️ Node-RED Integration
Visualizes the live location on an interactive map using the node-red-contrib-web-worldmap node.
