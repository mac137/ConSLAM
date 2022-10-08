"""
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or  (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import os
import sys, getopt
import numpy as np
from datetime import datetime
from scipy.spatial.transform import Rotation as R


def generate_evo_data_from_ppix_matrices(PATH):
    """
    The function takes poses in PointPix format and converts them into a raw evo formats.

    Parameters
    ----------
    PATH : str, obligatory
        valid path to the folder with PointPix matrices

    Returns
    ------
    list
        sorted timestamps
    list
        poses in raw kitti format (sorted with respect to the timestamps)
    list
         poses in raw tum format and NOT sanitized (sorted with respect to the timestamps)       
    """
    
    timestamps = []
    pos_list = []
    pos_list_TUM = []
    
    files = os.listdir(PATH)
    for file in files:
        if file.endswith('.txt'):
            file_path = os.path.join(PATH, file)
            pos_name = os.path.basename(file).split('.')
    
            matx = np.loadtxt(file_path)
        
            v = np.zeros(8)
            r = R.from_matrix(matx[0:3,0:3])
            qr = r.as_quat()
        
            valtime = 0
        
            if len(pos_name[0]) == 16:
                valtime = int(pos_name[0]) / 1000000.
            elif len(pos_name[0]) == 17:
                valtime = int(pos_name[0]) / 10000000.
            else:
                raise Exception("timestamps len is not standard")
            v[0] = valtime
       
            v[1:4] = np.transpose(matx[0:3,3:4])
            v[4:8] = qr
        

        timestamps.append(valtime)
        pos_list.append(list(matx[0:3,].flatten()))
        pos_list_TUM.append(v)

    timestamps, pos_list, pos_list_TUM = zip(*sorted(zip(timestamps, pos_list, pos_list_TUM)))
    
    return timestamps, pos_list, pos_list_TUM

def export_kitti_evo_format(pos_list, path_evo_kitti):
    """
    The function saves poses in the kitti format.

    Parameters
    ----------
    pos_list : list, obligatory
        list of poses in the raw, not sanitized KITTI format    
    path_evo_kitti : str, obligatory
        valid path to the output file
  
    """
    
    print('WARNING: the data are not sanitized, i.e., duplicates are possible to occur.')
    file_evo = open(path_evo_kitti, "w") 
    for pos in pos_list:
        file_evo.write(" ".join(str(item) for item in pos))
        file_evo.write("\n")  
    file_evo.close()

def export_tum_evo_format(pos_list_TUM, path_evo_tum):
    """
    The function saves poses in the kitti format.

    Parameters
    ----------
    pos_list_TUM : list, obligatory
        list of poses in the raw, not sanitized TUM format
    path_evo_tum : str, obligatory
        valid path to the output file
  
    """
    
    file_tum = open(path_evo_tum, "w")

    prev = 0.0
    for pos in pos_list_TUM:
        # remove duplicated data, it can happen but evo does not like that
        if str(pos[0]) == str(prev):
            continue
        prev = pos[0]
        file_tum.write(" ".join(str(item) for item in pos))
        file_tum.write("\n")
        
    file_tum.close()


def main(argv):

    PATH = ' '
    tum_file = ' '
    kitti_file = ' ' 

    try:
        opts, args = getopt.getopt(argv,"hw:t:k:",["wdir=", "tumfile=", "kittifile="])
    except getopt.GetoptError:
        print('pp2evo.py -w <dir_with_ppix_poses> -t <output_tum_file.csv> -k <output_kitti_file.csv>')
        sys.exit(2)
    for opt, arg in opts:
      if opt == '-h':
        print('pp2evo.py -w <dir_with_ppix_poses> -t <output_tum_file.csv> -k <output_kitti_file.csv>')
        sys.exit()
      elif opt in ("-w", "--wdir"):
        PATH = arg
        if not os.path.exists(PATH):
            raise Exception("Sorry, the work directory does not exist.")
      elif opt in ("-t", "--tumfile"):
        tum_file = arg
      elif opt in ("-k", "--kittifile"):
        kitti_file = arg        
    
    if (tum_file == ' ' or len(tum_file) == 0) and (kitti_file == ' ' or len(kitti_file) == 0):
        raise Exception("Sorry, but you need to provide an output file.")
    
    if tum_file == kitti_file:
        raise Exception("Sorry, kitti and tum files cannot be the same.")

    timestamps, pos_list, pos_list_TUM = generate_evo_data_from_ppix_matrices(PATH)

    if kitti_file != ' ' and len(kitti_file) != 0:    
        export_kitti_evo_format(pos_list, kitti_file)

    if tum_file != ' ' and len(tum_file) != 0:    
        export_tum_evo_format(pos_list_TUM, tum_file)   
    
if __name__ == "__main__":
   main(sys.argv[1:])