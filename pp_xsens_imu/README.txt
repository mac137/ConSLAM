=[ Xsens MTi driver for ROS ]============================================================

Documentation:
    You can find the full documentation in "<your MT SDK directory>/doc/xsensdeviceapi/doc/html/index.html" under "ROS MTi driver" section.

Prerequisites:
    - ROS Kinetic or Melodic
    - C/C++ Compiler: GCC 5.4.0 or MSVC 14.0
    - C++11

Building:
    - Copy xsens_ros_mti_driver folder from your MT SDK directory into your catkin workspace 'src' folder.
        Make sure the permissions are set to o+rw on your files and directories.

    - Build xspublic from your catkin workspace:
        $ pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd

    - Build Xsens MTi driver package:
        $ catkin_make

    - Source workspace:
        $ source devel/setup.bash

Running:
    - Configure your MTi device to output desired data (e.g. for display example - orientation output)

    - Launch the Xsens MTi driver from your catkin workspace:
            $ roslaunch xsens_mti_driver xsens_mti_node.launch

        After the device has been detected, you can communicate with it from another process / terminal window.
        For example:
            $ rostopic echo /filter/quaternion
        This will result in a continuous stream of data output:
            ---
            header: 
              seq: 1386351
              stamp: 
                secs: 1545223809
                nsecs: 197252179
              frame_id: "imu_link"
            quaternion: 
              x: 0.00276306713931
              y: 0.00036825647112
              z: -0.89693570137
              w: -0.442152231932
            ---

    - There is also an example that shows a 3D visualization of the device (orientation data should be enabled in the device):
            $ roslaunch xsens_mti_driver display.launch


Notes:
    - ROS timestamps
        The data messages from devices are time stamped on arrival in the ROS driver.
        When collecting data at higher rates, eg 100 Hz, the times between reads can differ from the configured output rate in the device.
        This is caused by possible buffering in the USB/FTDI driver.

        For instance:
        10 us, 10 us, 10 us, 45 ms, 10 us, 10 us, 10 us, 39 ms, 10 us, etc.
        instead of 
        10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, etc.

        Work-around: for accurate and stable time stamp information, users can make use of the sensor reported time stamp (/imu/time_ref) instead.

-[ Troubleshooting ]------------------------------------------------------------

    - The Mti1 (Motion Tracker Development Board) is not recognized.

        Support for the Development Board is present in recent kernels. (Since June 12, 2015).
        If your kernel does not support the Board, you can add this manually

        $ sudo /sbin/modprobe ftdi_sio
        $ echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id


    - The device is recognized, but I cannot ever access the device -

        Make sure you are in the correct group (often dialout or uucp) in order to
        access the device. You can test this with

            $ ls -l /dev/ttyUSB0
            crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
            $ groups
            dialout audio video usb users plugdev

        If you aren't in the correct group, you can fix this in two ways.

        1. Add yourself to the correct group
            You can add yourself to it by using your distributions user management
            tool, or call

                $ sudo usermod -G dialout -a $USER

            Be sure to replace dialout with the actual group name if it is
            different. After adding yourself to the group, either relogin to your
            user, or call

                $ newgrp dialout

            to add the current terminal session to the group.

        2. Use udev rules
            Alternatively, put the following rule into /etc/udev/rules.d/99-custom.rules

                SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="$GROUP", MODE="0660"

            Change $GROUP into your desired group (e.g. adm, plugdev, or usb).


    - The device is inaccessible for a while after plugging it in -

        When having problems with the device being busy the first 20 seconds after
        plugin, purge the modemmanager application.

    - RViz doesn't show an MTi model.

        It is a known issue with urdfdom in ROS Melodic. A workaround is to unset/modify the LC_NUMERIC environment variable:

        $ LC_NUMERIC="en_US.UTF-8"
