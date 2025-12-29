^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package feetech_ros2_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2025-12-29)
------------------
* Fix unused result warning (`#24 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/24>`_)
* 🛠️ Bump actions/cache from 4 to 5 (`#23 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/23>`_)
  Bumps [actions/cache](https://github.com/actions/cache) from 4 to 5.
  - [Release notes](https://github.com/actions/cache/releases)
  - [Changelog](https://github.com/actions/cache/blob/main/RELEASES.md)
  - [Commits](https://github.com/actions/cache/compare/v4...v5)
  ---
  updated-dependencies:
  - dependency-name: actions/cache
  dependency-version: '5'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* Fixes errors about missing braces (`#21 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/21>`_)
* Fix deprecated hardware_interface API (`#18 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/18>`_)
  * Fix deprecated hardware_interface API
  * Support both humble and jazzy
  ---------
  Co-authored-by: JafarAbdi <jafar.uruc@gmail.com>
* 🛠️ Bump actions/setup-python from 5 to 6 (`#17 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/17>`_)
  Bumps [actions/setup-python](https://github.com/actions/setup-python) from 5 to 6.
  - [Release notes](https://github.com/actions/setup-python/releases)
  - [Commits](https://github.com/actions/setup-python/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/setup-python
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
  Co-authored-by: Jafar Uruç <jafar.uruc@gmail.com>
* 🛠️ Bump actions/checkout from 5 to 6 (`#20 <https://github.com/JafarAbdi/feetech_ros2_driver/issues/20>`_)
  Bumps [actions/checkout](https://github.com/actions/checkout) from 5 to 6.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Changelog](https://github.com/actions/checkout/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/checkout/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* Contributors: Christoph Fröhlich, Louis LE LAY, Sebastian Castro, dependabot[bot]

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
