# NAU7802 driver
A ROS2 wrapper for the Cedargrove/Adafruit NAU7802 ADC sensor driver and library

## Install
(from your venv)
```
pip install -r requirements.txt
colcon build
source install/setup.zsh
```

## Run
```
ros2 run nau7802_adc nau7802_node

ros2 topic echo /nau7802/load
```

## Dependencies
This depends on the following python packages:

- [`cedargrove-nau7802`](https://github.com/adafruit/CircuitPython_NAU7802/) 
A CircuitPython driver class for the NAU7802 24-bit ADC 
- [`empy`](https://pypi.org/project/empy/) version 3.3.4 
(required to workaround [this issue](https://github.com/colcon/colcon-core/issues/602)  until `colcon` supports empy v4)