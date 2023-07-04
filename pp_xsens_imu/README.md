# pp_xsens_imu

# install

in ros_ws run these (based on http://wiki.ros.org/xsens_mti_driver)

install MTI package first. Go on their webstie and follow the intruction. Particulary after downlaoding, unpack and `bash ./mtsdk_linux-x64_2021.4.sh`

then copy from the unpacked folder the ros driver or just have this one from this repo

`pushd src/pp_xsens_imu/lib/xspublic && make && popd`

`catkin_make`

`source devel/setup.bash`

`roslaunch xsens_mti_driver display.launch`

# troubleshooting

- be sure that this command `ls -l /dev/ttyUSB0` actually works and displays ```crw-rw---- 1 root dialout 188, 0 May 6 16:21 /dev/ttyUSB0```

- then make sure that `maciej` is in `dialout`. How? type `groups` and if maciej is not there then add it via `sudo usermod -G dialout -a $USER` or the other coomand in the link (re-log most likely neccessary)

- if you cant see maciej and dialout there then the IMU wont work.

- i noticed that you should first plug in the adapter with imu and then start ubuntu to to make `ls -l /dev/ttyUSB0` work. UPDATE: actually i noticed with mbp14 that you should boot ubuntu and then when you r in ubuntu then plug the usb and select ubuntu.
