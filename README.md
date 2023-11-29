<br/>
<p align="center">
  <h3 align="center">Stiebel Eltron Can Sniffer</h3>

  <p align="center">
    Make Your Heat Pump Smarter
    <br/>
    <br/>
  </p>
</p>

<img align="center" width="200" src="/images/renderings/case_encoder_oled.PNG">

## About The Project

<div style="text-align: justify">
This project involves enhancing the capabilities of Stiebel Eltron heat pumps through the development of a custom circuit board. The board interfaces with the heat pump's CAN bus, enabling the extraction and modification of parameters. The primary goal is to optimize the cooling function for smart homes. By altering the communication between the heat pump and the FEK, this project facilitates the utilization of a more efficient cooling function. The designed interface allows data retrieval from the system through serial communication and Modbus, offering flexibility. Additionally, it enables the modification of values sent from the FEK to the heat pump, such as room temperature and humidity. 

A key feature is the potential to relocate the FEK closer to the heat pump, eliminating the necessity for it to be in an excessively cool room. Looking ahead, the interface has the capacity to eventually replace the entire communication system of the FEK. Powered by an ESP32, the board provides a versatile platform with numerous expansion possibilities.
</div>

## Table Of Contents

* [Getting Started](#getting-started)
  * [Hardware](#hardware)
  * [Software](#software)
* [Installation](#installation)
* [Contributing](#contributing)
* [Acknowledgements](#acknowledgements)
* [License](#license)

## Getting Started

<div style="text-align: justify">
The board can be used in two different ways. Simple or interactive configuration. Both versions offer the same possibilities, with the interactive version the board can be shut down more easily, the functionality can be monitored and parameters can be set more comfortably.
</div>

### Hardware

TODO

### Software

TODO 

## Installation

TODO 

## Contributing

Help with expanding the software options is always welcome. The bus dump and the board should allow a complete emulation of the FEK/FET. However, I don't have the time to work this out in detail. A dump of the bus communication can be found [here](utils/bus_dump).

## Acknowledgements

* [juerg5524](https://juerg5524.ch/list_data.php)
* [bullitt186](https://github.com/bullitt186/ha-stiebel-control)
* [mkaiser](https://github.com/mkaiser/ESP32-CAN)

## License

[GPLv3] (https://www.gnu.org/licenses/gpl-3.0.en.html)