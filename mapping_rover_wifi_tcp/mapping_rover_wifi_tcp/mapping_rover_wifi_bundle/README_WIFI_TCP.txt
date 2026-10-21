MAPPING ROVER — WI-FI TCP BUNDLE (NO APPLICATION ACK)
====================================================

FILES
-----
wifi_connect_tcp.ino
  ESP32 DOIT DEVKIT V1 bridge. Receives newline-delimited STM32 UART2 data
  on GPIO16 (RX2) and serves it to the laptop using TCP port 5000.

main_wifi_no_ack.c
  STM32F401 main.c matching the DATA record format. Use this only if you want
  the included no-ACK scan/movement firmware. Keep your existing Keil project
  platform, VL53L0X, and servo source/header files.

live_lidar_plot_wifi_tcp.py
  Python TCP client with live 3D point cloud and distance heatmap. It understands
  DATA,sequence,distance_mm,angle_raw,z_mm records and sends no ACKs.

requirements.txt
  Python package list.

DATA PROTOCOL
-------------
STM32 -> ESP32 UART2 -> TCP -> Python:
  DATA,<sequence>,<distance_mm>,<angle_raw>,<z_mm>
Examples:
  DATA,1400,77,1980,140
  SCAN_START
  Z_MOVE_START
  Z_MOVE_END

Status strings are informational. Invalid distance readings (e.g. -1) are not
plotted. TCP provides ordered stream transport while connected, but this bundle
does not implement application-level acknowledgements. The STM32 continues its
scan/movement sequence regardless of whether the laptop is connected.

WIRING
------
STM32 PA2 (USART2 TX) ---> ESP32 GPIO16 (UART2 RX)
STM32 GND                --- ESP32 GND
ESP32 GPIO17 / STM32 PA3 are not required for this no-ACK data path.
UART baud rate: 115200, 8-N-1.

SETUP
-----
1. Open wifi_connect_tcp.ino in Arduino IDE. Select the ESP32 DOIT DEVKIT V1.
2. In the sketch, edit WIFI_SSID and WIFI_PASSWORD to the exact Wi-Fi network
   credentials. The included values ("Wifi" / "12345678") are carried over from
   the previous Wi-Fi sketch; replace them if they are not your actual network.
3. Upload to ESP32 and open Serial Monitor at 115200 baud.
4. Wait for "Wi-Fi connected" and note the "ESP32 IP address" printed there.
   If it does not connect, check the credentials and use a 2.4 GHz Wi-Fi network.
5. Make sure the laptop is connected to the same Wi-Fi network. If using a phone
   hotspot, connect BOTH the laptop and ESP32 to that hotspot.
6. Edit ESP32_IP near the top of live_lidar_plot_wifi_tcp.py to the IP printed
   by the ESP32. Leave ESP32_PORT = 5000.
7. Install the Python packages in a terminal:
      py -m pip install -r requirements.txt
   Or:
      py -m pip install numpy matplotlib
8. Flash main_wifi_no_ack.c as main.c if using this included STM32 firmware.
   If your currently working STM32 already emits DATA,sequence,distance,angle_raw,z
   at 115200 baud on PA2, no STM32 firmware change is needed.
9. Run the Python plotter:
      py live_lidar_plot_wifi_tcp.py
10. The ESP32 Serial Monitor should print "Laptop TCP client connected from ..."
    and continue printing "STM32: DATA,..." lines. Python will print that TCP
    connected and then plot valid LiDAR measurements.

TROUBLESHOOTING
---------------
- The ESP32 IP can change after reboot/rejoining Wi-Fi; update ESP32_IP in Python.
- Allow Python through Windows Defender Firewall on private networks, or allow
  incoming TCP port 5000 for Python if the connection times out.
- Some institutional/guest Wi-Fi networks block communication between clients.
  If laptop and ESP32 cannot communicate on the same network, try a phone hotspot
  or another private 2.4 GHz network.
- Do not run the Python plotter before the ESP32 joins Wi-Fi and reports its IP.
- USB Serial Monitor (ESP32 COM port) is only for diagnostics; Python uses TCP,
  not COM6, COM7, or COM8.
