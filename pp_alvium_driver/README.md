# Drivers for Alvium USB cameras

## How to run
### 1 camera
To run **just 1 camera** simply run in a terminal `roslaunch pp_alvium_driver rgb_only_python.launch` or `nir_only_python.launch`.

### 2 cameras simultaneously
To run **2 cameras simultaneously**:
1. run `roslaunch pp_alvium_driver rgb_only_python.launch` in a terminal
2. then open another terminal, and run `conda activate <<virtual env>>`
3. in the activated conda's `<virtual env>>` run `roslaunch pp_alvium_driver nir_only_python.launch`

The whole point for the virtual environemnt is that the way above gives you maximum frame rate from both cameras whereas using multithreading gives you way lower Hz and things start breaking up when my second camera wants to achieve fps > 20 Hz. I presume this is due to the fact that Python uses only 1 core for both cameras to operate. 

### How to install vimba in Conda's virtual environemnt?
Just install vimba to a default system's interpreter. And then literally copy manually `vimba` and `VimbaPython-1.0.1.dist-info` folders from the python base where they have just been installed (e.g. in `/usr/local/lib/python3.8`) to your venv folder, for example `/home/maciej/miniconda3/envs/4ros1/lib/python3.8/site-packages`.


## Remarks
1. Install `Vimba_4_2` for intel processors or `Vimba 5.1` for arm to a desired python directory as instructed by `VimbaPython/Install.sh`
2. Make sure that `GENICAM_GENTL64_PATH` points to `path/2/vimba/Vimba_4_2/VimbaUSBTL/CTI/x86_64bit` or for vimba 5.1 accordingly
3. This code has been tested on python 3.8
