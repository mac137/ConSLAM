# General

The script can convert PointPix poses into two file formats that EVO uses. These file formats are TUM and KITTI.

To run the code type: 

```python.exe .\pp2evo.py -w <path_to_dir_ppix_matrices> -t <output_tum_file_format> -k <output_kitti_file_format>```


The tool can output a TUM, a KITTI file, or both if necessary.


# Using EVO

## Plotting trajectories

To plot trajectories against a reference, it is sufficient to call:

```evo_traj tum 00_ORB.txt 00_SPTAM.txt --ref=00_gt.txt -p --plot_mode=xy```

in the above example, TUM file format is used (see the first argument). The KITTI file format is quite limited, and it should not be used.

To compute Absolute Pose Error we can simply call

```evo_ape tum 00_gt.txt 00_ORB.txt -v --plot --plot_mode xy --save_results results/ORB.zip```

In this example, the automatic alignment of trajectories is switched off.

Note that the zip file can be used for further plotting in using `evo_res`, e.g.,

```evo_res results/*.zip -p --save_table results/table.csv```


## Generating high-quality polts

TODO....