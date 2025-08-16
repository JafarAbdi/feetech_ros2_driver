^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package feetech_ros2_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2025-08-16)
------------------
* 🛠️ Bump actions/checkout from 4 to 5 (`#15 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/15>`_)
  Bumps [actions/checkout](https://github.com/actions/checkout) from 4 to 5.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Changelog](https://github.com/actions/checkout/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/checkout/compare/v4...v5)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: '5'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* Add option to enable warnings as errors & enable by default in CI (`#16 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/16>`_)
* Fix error for colcon build (`#13 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/13>`_)
  * fix error for compile
  * add #include <fmt/ranges.h> to demo.cpp
  * use fmt::join(keys, ", ")
* Add on_deactivate function to disable torque when exit (`#12 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/12>`_)
* Disable holding torque for joints without command interfaces (`#10 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/10>`_)
* Write commands only if joint has any command interfaces defined (`#9 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/9>`_)
* Move ROS 2 unrelated code to a core standalone cmake package (`#7 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/7>`_)
* Contributors: Bence Magyar, Jafar Uruç, Tsogoo, dependabot[bot], yadunund

0.1.0 (2024-11-13)
------------------
* Add feetech ros2 driver
* Contributors: Jafar Uruç
