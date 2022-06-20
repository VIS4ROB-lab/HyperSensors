[contributors-shield]: https://img.shields.io/github/contributors/VIS4ROB-lab/HyperSensors.svg?style=for-the-badge
[contributors-url]: https://github.com/VIS4ROB-lab/HyperSensors/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/VIS4ROB-lab/HyperSensors.svg?style=for-the-badge
[forks-url]: https://github.com/VIS4ROB-lab/HyperSensors/network/members
[stars-shield]: https://img.shields.io/github/stars/VIS4ROB-lab/HyperSensors.svg?style=for-the-badge
[stars-url]: https://github.com/VIS4ROB-lab/HyperSensors/stargazers
[issues-shield]: https://img.shields.io/github/issues/VIS4ROB-lab/HyperSensors.svg?style=for-the-badge
[issues-url]: https://github.com/VIS4ROB-lab/HyperSensors/issues
[license-shield]: https://img.shields.io/github/license/VIS4ROB-lab/HyperSensors.svg?style=for-the-badge
[license-url]: https://github.com/VIS4ROB-lab/HyperSensors/blob/main/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/davidhug

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![BSD-3-Clause License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

<br />
<br />
<div align="center">
  <a href="https://github.com/VIS4ROB-lab/HyperSensors">
    <img src="https://drive.google.com/uc?export=view&id=1UAFr3tepqKwdnTomhKaeI2eIag3HOISY" alt="" style="width: 150px;">
  </a>

<h2><em>Hyper</em>Sensors</h2>
  <p>
    Implementations of common sensing modalities for computer vision. 
    <br />
    <a href="https://github.com/VIS4ROB-lab/HyperSensors/issues">Report Issues or Request Features</a>
  </p>
</div>
<br />

## About

[*Hyper*Sensors](https://github.com/VIS4ROB-lab/HyperState) are part of the
[*Hyper*SLAM](https://github.com/VIS4ROB-lab/HyperSLAM) framework and implement, based on
the [*Hyper*Variables](https://github.com/VIS4ROB-lab/HyperVariables) repository, the physical sensor models (i.e.
absolute sensor, pinhole camera and inertial measurement unit models) employed
in *Hyper*SLAM. If you use this repository, please cite it as below.

```
@article{RAL2022Hug,
    author={Hug, David and B\"anninger, Philipp and Alzugaray, Ignacio and Chli, Margarita},
    journal={IEEE Robotics and Automation Letters},
    title={Continuous-Time Stereo-Inertial Odometry},
    year={2022},
    volume={7},
    number={3},
    pages={6455-6462},
    doi={10.1109/LRA.2022.3173705}
}
```

## Installation

[*Hyper*Sensors](https://github.com/VIS4ROB-lab/HyperSensors) depends on the
[*Hyper*Variables](https://github.com/VIS4ROB-lab/HyperVariables) and
[*Hyper*State](https://github.com/VIS4ROB-lab/HyperState) libraries and uses features from the
[C++20](https://en.cppreference.com/w/cpp/20) standard (see
[link](https://askubuntu.com/questions/26498/how-to-choose-the-default-gcc-and-g-version) to update gcc and g++
alternatives). All dependencies aside from the ones contained in the `setup.sh` script are automatically fetched
during compilation. The compilation process itself (without additional compile flags) is as follows:

```
# Clone repository.
git clone https://github.com/VIS4ROB-lab/HyperSensors.git && cd HyperSensors/

# Run setup.
chmod +x setup.sh
sudo setup.sh

# Build repository.
mkdir build && cd build
cmake ..
make
```

## Literature

1. [Continuous-Time Stereo-Inertial Odometry, Hug et al. (2022)](https://ieeexplore.ieee.org/document/9772323)
2. [HyperSLAM: A Generic and Modular Approach to Sensor Fusion and Simultaneous<br /> Localization And Mapping in Continuous-Time, Hug and Chli (2020)](https://ieeexplore.ieee.org/document/9320417)
3. [Extending kalibr: Calibrating the Extrinsics of Multiple<br /> IMUs and of Individual Axes, Rehder et al. (2016)](https://ieeexplore.ieee.org/document/7487628)

### Updates

25.07.22 Initial release of *Hyper*Sensors.

### Contact

Admin - [David Hug](mailto:dhug@ethz.ch), Leonhardstrasse 21, 8092 Zürich, ETH Zürich, Switzerland  
Maintainer - [Philipp Bänninger](mailto:baephili@ethz.ch), Leonhardstrasse 21, 8092 Zürich, ETH Zürich, Switzerland  
Maintainer - [Ignacio Alzugaray](mailto:aignacio@ethz.ch), Leonhardstrasse 21, 8092 Zürich, ETH Zürich, Switzerland

### License

*Hyper*Sensors are distributed under the [BSD-3-Clause License](LICENSE).
