# DEPRECATED!!! This repository is not maintained anymore!

# ctrl_lib - Controller Library for Whole-Body Control

[Code API](http://arc-opt.github.io/orogen-ctrl_lib/)  | [Full Documentation](https://arc-opt.github.io/Documentation/)

This task library provides a collection of task and joint space controllers for implementing tasks in a Whole-Body Control framework.

ctrl_lib was initiated and is currently developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

<img src="doc/images/DFKI_Logo_e_schrift.jpg" alt="drawing" width="300"/>

## Motivation

[WBC](https://github.com/ARC-OPT/wbc) is a framework for optimization-based control of redundant robots. This task library provides thin wrappers for the controllers implemented in the WBC library, to be used within the Rock framework. Examples of implemented controllers are

  * CartesianPositionController: Position/velocity controller with acceleration feed forward in Cartesian space
  * JointPositionController: Position/velocity controller with acceleration feed forward in joint space 
  * JointLimitAvoidance: Joint position limit avoidance based on potential fields 
  * CartesianRadialPotentialFields: Collision avoidance in Cartesian space

## Getting Started

* Please check out the tutorials section in the [documentation](https://arc-opt.github.io/Documentation/) for examples of usage.

## Requirements / Dependencies

Currently supported OS: Ubuntu18.04, Ubuntu20.04

This task library is for sole use within the [Rock framework](https://www.rock-robotics.org/). I.e., you require a full Rock installation to use it. In addition, it has the following 1st order dependencies:

* [WBC library](https://github.com/ARC-OPT/wbc)

## Installation

* New Bootstrap: See [here](https://arc-opt.github.io/Documentation/installation/installation_rock.html)
* Existing Rock Installation: Add the wbc package set to your autoproj/manifest file: 
    ```
    package_sets:
    - github: ARC-OPT/package_set
    ```    
  followed by `aup control/orogen/ctrl_lib` and then `amake control/orogen/ctrl_lib`

## Testing

Please check the unit tests [here](https://github.com/ARC-OPT/orogen-ctrl_lib/tree/master/test), as well as the [tutorials](https://github.com/ARC-OPT/orogen-wbc/tree/master/tutorials) in the WBC task library. 

## Contributing

Please use the [issue tracker](https://github.com/ARC-OPT/orogen-ctrl_lib/issues) to submit bug reports and feature requests.

Please use merge requests as described [here](https://github.com/ARC-OPT/orogen-ctrl_lib/blob/master/CONTRIBUTING.md) to add/adapt functionality. 

## License

orogen-ctrl_lib is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).

## Acknowledge WBC

If you use Ctrl_lib within your scientific work, please cite the following publication:

```
@INPROCEEDINGS{mronga2022,
author = "D. Mronga and S.Kumar and F.Kirchner",
title = "Whole-Body Control of Series-Parallel Hybrid Robots",
year = "2022",
note = "{2022 IEEE International Conference on Robotics and Automation (ICRA)}, Accepted for publication",
}
```

## Funding

Ctrl_lib has been developed in the research projects [TransFit](https://robotik.dfki-bremen.de/en/research/projects/transfit/) (Grant number 50RA1701) and [BesMan](https://robotik.dfki-bremen.de/en/research/projects/besman.html) (Grant number 50RA1216) funded by the German Aerospace Center (DLR) with funds from the German	Federal Ministry for Economic Affairs and Climate Action (BMWK). It is further developed in the [M-Rock](https://robotik.dfki-bremen.de/en/research/projects/m-rock/) (Grant number 01IW21002) and [VeryHuman](https://robotik.dfki-bremen.de/en/research/projects/veryhuman/) (Grant number  01IW20004) projects funded by the German Aerospace Center (DLR) with federal funds from the German Federal Ministry of Education and Research (BMBF).

## Maintainer / Authors / Contributers

Dennis Mronga, dennis.mronga@dfki.de

Copyright 2017, DFKI GmbH / Robotics Innovation Center

