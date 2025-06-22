^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package septentrio_gnss_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.4 (2025-06-06)
------------------
* Fixes
    * Readme on PVT messages for INS
    * Deprecation of ament_target_dependencies in Rolling
* Contributors:  Thomas Emter, Tibor Dome, septentrio-users

1.4.3 (2025-05-09)
------------------
* Fixes
    * Resolve issues with removed/renamed functionality in boost 1.87 (thanks to @oysstu)
    * VSM data not being sent to the rx if configure_rx is false
    * ROS 2 Rolling regression (thanks to @kevshin2002)
* Improvements
    * IMU orientation sync
* Contributors:  @oysstu, @kevshin2002, Thomas Emter, Tibor Dome, septentrio-users

1.4.2 (2025-02-22)
------------------
* Fixes
    * Add export of compiler directives (thanks to @oysstu)
    * ROS 1 rebuild (thanks to @peci1)
* Improvements
    * Rework TCP connection/reconnection
* Changes
    * In case INS is not aligned yet but has GNSS heading, a valid orientation with roll and pitch = 0.0 will be published.
* Contributors: Martin Pecka, Thomas Emter, @oysstu, Tibor Dome, septentrio-users

1.4.1 (2024-08-04)
------------------
* Fixes
   * Lever arm calculation from tf
   * NavSatStatus and GPSFixStatus
   * Orientation in pose topic of GNSS
* Contributors: Thomas Emter, Tibor Dome, septentrio-users

1.4.0 (2024-05-21)
------------------
* New features
   * Send custom commands via ASCII file on startup
   * Save config to boot after setup
   * NTP and PTP server options (BREAKING: NTP is not setup automatically for `use_gnss_time: true` anymore)
   * Receiver status on `/diagnostics`
   * Option to publish only valid SBF block messages
   * Option to auto publish available messages for `configure_rx: false`
* Changes
   * Change floating point do-not-use-values to NaN (BREAKING in case these values are used for validity checks downstream)
   * VSM now uses separate TCP device specified IP server
* Improvements
   * Rework some sections of the README
   * Combine ROS 1 and ROS 2 in one branch
   * Change GPSFix publishing policy to allow for high update rates
* Contributors: Thomas Emter, Tibor Dome, septentrio-users

1.3.2 (2023-11-19)
------------------
* Fixes
    * IMU units
    * Topics namespace
    * Units of imu angular rates
* Contributors: Thomas Emter, Tibor Dome, septentrio-users

1.3.1 (2023-07-06)
------------------
* New Features
   * Recovery from connection interruption
   * Add option to bypass configuration of Rx
   * Add tests
   * OSNMA
   * Latency compensation for ROS timestamps
   * Output of SBf block VelCovCartesian
   * Support for UDP and TCP via IP server
   * New VSM handling allows for unknown variances (INS firmware >= 1.4.1)
   * Add heading angle to GPSFix msg (by diverting dip field, cf. readme)
* Improvements
   * Rework IO core and message handling
      * Unified stream processing 
      * Internal data queue
      * Prevent message loss in file reading
   * Add some explanatory warnings for parameter mismatches
   * Add units to message definitions
* Fixes
   * navsatfix for INS
   * Empty headers
   * Single antenna receiver setup
* Preliminary Features
   * Output of localization and tf in ECEF frame, testing and feedback welcome
* Contributors: Thomas Emter, Tibor Dome

1.2.3 (2022-11-09)
------------------
* New Features
   * Twist output option
   * Example config files for GNSS and INS
   * Get leap seconds from receiver
   * Firmware check
   * VSM from odometry or twist ROS messages
   * Add receiver type in case INS is used in GNSS mode
   * Add publishing of base vector topics
* Improvements
   * Rework RTK corrections parameters and improve flexibility
* Fixes
   * /tf not being published without /localization
   * Twist covariance matrix of localization
   * Support 5 ms period for IMU explicitly

1.2.2 (2022-06-22)
------------------
* Fixes
   * Memory corruption under adverse conditions
* Contributors: Thomas Emter, Tibor Dome

1.2.1 (2022-05-16)
------------------
* New Features
   * Add login credentials
   * Activate NTP server if use_gnss_time is set to true
* Improvements
   * Add NED option to localization
* Fixes
   * IMU orientation for ROS axis convention
* Contributors: Daisuke Nishimatsu, Thomas Emter, Tibor Dome

1.2.0 (2022-04-27)
------------------
* New Features
   * Add option to use ROS axis orientations according to REP103
   * Add frame_id parameters
   * Add option to get frames from tf
   * Publishing of cartesian localization in UTM (topic and/or tf) for INS
   * Publishing of IMU topic for INS
   * Publishing of MeasEpoch
   * ROS2 branch
* Improvements
   * Add multi antenna option
   * Increase number of SBF streams
   * Add option to set polling_period to "on change"
   * Increased buffer size from 8192 to 131072 bytes
   * Add endianess aware parsers
   * Only publish topics set to true
   * Add parameter to switch DEBUG logging on and off
   * Change GPxxx messages to ROS built-in types
   * Remove duplicate INS msg types
* Fixes
   * Setting of antenna type
   * Publishing rate interconnections of gpsfix and velcovgeodetic
   * Missing quotes for antenna type
   * Broken attitude parsing pose and gpsfix from INS
   * IMU orientation was not sent to Rx
   * Graceful shutdown of threads
* Contributors: Thomas Emter, Tibor Dome, tibordome

1.0.8 (2021-10-23)
------------------
* Added INS Support

1.0.7 (2021-05-18)
------------------
* Clang formatting, publishing from SBF log, play-back of PCAP files

1.0.6 (2020-10-16)
------------------
* ROSaic binary installation now available on Melodic & Noetic

1.0.5 (2020-10-15)
------------------
* changed repo name
* v1.0.4
* 1.0.3
* Merge pull request `#22 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/22>`_ from septentrio-gnss/local_tibor
  New changelog
* New changelog
* Merge pull request `#21 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/21>`_ from septentrio-gnss/local_tibor
  Added rosdoc.yaml file
* Merge pull request `#20 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/20>`_ from septentrio-gnss/local_tibor
  Improved doxygen annotations
* Merge pull request `#19 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/19>`_ from septentrio-gnss/local_tibor
  Improved doxygen annotations
* Update README.md
* Merge pull request `#18 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/18>`_ from septentrio-gnss/local_tibor
  Adopted ROS and C++ conventions, added ROS diagnostics msg,
* Update README.md
* Update README.md
* Update README.md
* Contributors: septentrio-users, tibordome

1.0.4 (2020-10-11)
------------------
* Added rosdoc.yaml file
* Improved doxygen annotations
* Improved doxygen annotations
* Adopted ROS and C++ conventions, added ROS diagnostics msg, removed ROS garbage value bug, added auto-detection of SBF arrival order for composite ROS msgs
* Merge branch 'master' of https://github.com/septentrio-gnss/rosaic
* NTRIP with Datalink, circular buffer, reading connection descriptor, new messages
* Update README.md
* Contributors: septentrio-users, tibordome

1.0.3 (2020-09-30)
------------------
* Add new config/rover.yaml file
* Add config/rover.yaml to .gitignore
* Merge pull request `#17 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/17>`_ from septentrio-gnss/local_tibor
  NTRIP with Datalink, circular buffer, reading connection descriptor..
* Merge branch 'local_tibor'
* NTRIP with Datalink, circular buffer, reading connection descriptor, new messages
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#16 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/16>`_ from septentrio-gnss/local_tibor
  NTRIP parameters added, reconnect_delay_s implemented,
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#15 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/15>`_ from tibordome/local_tibor
  GPSFix completed, datum as new parameter
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#14 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/14>`_ from tibordome/local_tibor
  GPSFix completed, datum as new parameter
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#13 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/13>`_ from tibordome/local_tibor
  Added AttCovEuler.msg and AttEuler.msg
* Merge pull request `#12 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/12>`_ from tibordome/local_tibor
  Fixed service field of NavSatStatus
* Contributors: Tibor Dome, septentrio-users, tibordome

1.0.2 (2020-09-25)
------------------
* NTRIP parameters added, reconnect_delay_s implemented, package.xml updated, ROSaic now detects connection descriptor automatically, mosaic serial port parameter added
* GPSFix completed, datum as new parameter, ANT type and marker-to-arp distances as new parameters, BlockLength() method corrected, sending multiple commands to Rx corrected by means of mutex
* Contributors: tibordome

1.0.1 (2020-09-22)
------------------
* GPSFix completed, datum as new parameter, ANT type and marker-to-arp distances as new parameters, BlockLength() method corrected, sending multiple commands to Rx corrected by means of mutex
* Added AttCovEuler.msg and AttEuler.msg
* Fixed service field of NavSatStatus, fixed ROS header's seq field of each published ROS message, added write method for sending commands to Rx, successfully tested, added AttEuler, added AttCovEuler
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#11 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/11>`_ from tibordome/local_tibor
  rosconsole_backend_interface dependency not needed
* rosconsole_backend_interface dependency not needed
* Merge pull request `#10 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/10>`_ from tibordome/local_tibor
  rosconsole_log4cxx dep not needed
* rosconsole_log4cxx dep not needed
* Merge pull request `#9 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/9>`_ from tibordome/local_tibor
  rosconsole_log4cxx dep not needed
* rosconsole_log4cxx dep not needed
* Merge pull request `#8 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/8>`_ from tibordome/local_tibor
  Local tibor
* Update README.md
* Merge pull request `#7 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/7>`_ from tibordome/local_tibor
  Ready for First Release
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#6 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/6>`_ from tibordome/local_tibor
  Local tibor
* Merge pull request `#5 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/5>`_ from tibordome/local_tibor
  TCP seems to work
* Contributors: Tibor Dome, tibordome

1.0.0 (2020-09-11)
------------------
* Ready for first release
* Added Gpgga.msg and PosCovGeodetic.msg files
* Ready for First Release
* Ready for first release
* Ready for first release
* Ready for first release
* TCP bug removed
* TCP bug removed
* TCP seems to work
* Merge pull request `#4 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/4>`_ from tibordome/v0.2
  V0.2
* PVTCartesian and PVTGeodetic publishing works on serial
* PVTCartesian and PVTGeodetic publishing works on serial
* Merge pull request `#3 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/3>`_ from tibordome/v0.2
  Add doxygen_out and Doxyfile 2nd trial
* Add doxygen_out and Doxyfile 2nd trial
* Merge pull request `#2 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/2>`_ from tibordome/v0.1
  Add doxygen_out and Doxyfile
* Add doxygen_out and Doxyfile
* Update README.md
* Create README.md
* Update LICENSE
* Merge pull request `#1 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/1>`_ from tibordome/add-license-1
  Create LICENSE
* Create LICENSE
* Create LICENSE
* Commit
* Successfully tested publishing to /gpgga topic via serial
* To make sure master branch exists
* Contributors: Tibor Dome, tibordome
