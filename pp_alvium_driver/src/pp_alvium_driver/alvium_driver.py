"""BSD 2-Clause License

Copyright (c) 2019, Allied Vision Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import threading
import sys
from typing import Optional
from vimba import *


def print_usage():
    print('Usage:')
    print('    python asynchronous_grab_opencv.py [camera_id]')
    print('    python asynchronous_grab_opencv.py [/h] [-h]')
    print()
    print('Parameters:')
    print('    camera_id   ID of the camera to use (using first camera if not specified)')
    print()

def abort(reason: str, return_code: int = 1, usage: bool = False):
    print(reason + '\n')

    if usage:
        print_usage()

    sys.exit(return_code)

def get_camera(camera_id: Optional[str]) -> Camera:
    with Vimba.get_instance() as vimba:
        if camera_id:
            try:
                return vimba.get_camera_by_id(camera_id)

            except VimbaCameraError:
                abort('Failed to access Camera \'{}\'. Abort.'.format(camera_id))

        else:
            cams = vimba.get_all_cameras()
            if not cams:
                abort('No Cameras accessible. Abort.')

            return cams[0]


def setup_camera(cam: Camera, exposure_time: int):
    with cam:
        # Enable auto exposure time setting if camera supports it
        try:
            if cam.get_id() == "DEV_1AB22C00A470":
                cam.ExposureAuto.set('Continuous')
                # cam.ExposureAutoMax.set(48233)
                cam.ExposureAutoMax.set(exposure_time)
            elif cam.get_id() == "DEV_1AB22C00C28B":
                cam.ExposureAuto.set('Continuous')
                # cam.ExposureAutoMax.set(48233)
                cam.ExposureAutoMax.set(exposure_time)
        except (AttributeError, VimbaFeatureError):
            pass

        # Enable white balancing if camera supports it
        try:
            cam.BalanceWhiteAuto.set('Continuous')

        except (AttributeError, VimbaFeatureError):
            pass

        # if cam.get_id() == "DEV_1AB22C00C28B" or cam.get_id() == "DEV_1AB22C00A470":
        # Gain only for the NIR camera. IDK why but this way both cams work best
        try:
            cam.GainAuto.set('Continuous')

        except (AttributeError, VimbaFeatureError):
            pass

        # Query available, open_cv compatible pixel formats
        # prefer color formats over monochrome formats
        cv_fmts = intersect_pixel_formats(cam.get_pixel_formats(), OPENCV_PIXEL_FORMATS)
        color_fmts = intersect_pixel_formats(cv_fmts, COLOR_PIXEL_FORMATS)
        #
        # for format in cam.get_pixel_formats():
        #     print(format)
        #
        # if color_fmts:
        #     cam.set_pixel_format(color_fmts[0])
        #
        # else:
        #     mono_fmts = intersect_pixel_formats(cv_fmts, MONO_PIXEL_FORMATS)
        #
        #     if mono_fmts:
        #         cam.set_pixel_format(mono_fmts[0])
        #
        #     else:
        #         abort('Camera does not support a OpenCV compatible format natively. Abort.')

        try:
            if cam.get_id() == "DEV_1AB22C00A470":
                # for format in cam.get_pixel_formats():
                #     print(format)
                # Mono8
                # Mono10
                # Mono10p
                # Mono12
                # Mono12p
                # BayerRG8
                # BayerRG10
                # BayerRG12
                # BayerRG10p
                # BayerRG12p
                # Rgb8
                # Bgr8
                # YCbCr411_8_CbYYCrYY
                # YCbCr422_8_CbYCrY
                # YCbCr8_CbYCr
                cam.set_pixel_format(cam.get_pixel_formats()[5])  # that's BayerRG8
            else:
                mono_fmts = intersect_pixel_formats(cv_fmts, MONO_PIXEL_FORMATS)
                cam.set_pixel_format(mono_fmts[0])

        except (AttributeError, VimbaFeatureError):
            pass


def main():
    pass


if __name__ == '__main__':
    main()
