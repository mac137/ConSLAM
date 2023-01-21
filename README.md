# ConSLAM

# Download the dataset
A link to the dataset:
```
https://drive.google.com/drive/folders/1qOZGXjlLQNu15xakG7NqE5WEv3Iu9akr?usp=sharing
```
*The bag file in sequence 1 is faulty. Do no use it for your SLAM algorithms*

# Evaluate your odometry or SLAM trajectory
We have extended a popular trajectory evaluation package [evo](https://github.com/MichaelGrupp/evo) for odometry and SLAM trajectory evaluation.

Save your poses you would like to evaluate as individual `.txt` files containing 4x4 pose matrices. Our script in `tools/` can then convert the poses into two file formats that EVO uses. These file formats are TUM and KITTI.

To run the code type: 

```python ./tools/pp2evo.py -w <path_to_dir_ppix_matrices> -t <output_tum_file_format> -k <output_kitti_file_format>```


The tool can output a TUM, a KITTI file, or both if necessary.

## Using EVO
For general information about Evo, please, see the official project web page: https://github.com/MichaelGrupp/evo

## Plotting trajectories by EVO

To plot trajectories against a reference, it is sufficient to call:

```evo_traj tum 00_ORB.txt 00_SPTAM.txt --ref=00_gt.txt -p --plot_mode=xy```

in the above example, TUM file format is used (see the first argument). The KITTI file format is quite limited, and it should not be used.

To compute Absolute Pose Error we can simply call

```evo_ape tum 00_gt.txt 00_ORB.txt -v --plot --plot_mode xy --save_results results/ORB.zip```

In this example, the automatic alignment of trajectories is switched off.

Note that the zip file can be used for further plotting in using `evo_res`, e.g.,

```evo_res results/*.zip -p --save_table results/table.csv```


## Generating high-quality plots

To generate plots ready for a paper, set up the configuration of Evo as you desire with `evo config`, for example:

``` sh
evo_config set plot_seaborn_style whitegrid \
               plot_fontfamily serif plot_fontscale 1.2 \
               plot_linewidth 1.0 \
               plot_figsize 5 4.5 \
               ros_map_unknown_cell_value 128 \
               plot_usetex               
 ```

Before and after:

  <a href="https://github.com/MichaelGrupp/evo/wiki/images/plot_style_default.png" target="_blank">
    <img src="https://github.com/MichaelGrupp/evo/wiki/images/plot_style_default.png" alt="evo" height="200" border="5" />
  </a>
  
  <a href="https://github.com/MichaelGrupp/evo/wiki/images/plot_style_changed.png" target="_blank">
    <img src="https://github.com/MichaelGrupp/evo/wiki/images/plot_style_changed.png" alt="evo" height="200" border="5" />
  </a>

You can find more information about this configuration [here](https://github.com/MichaelGrupp/evo/wiki/Plotting).

# TODO
- [ ] Replace evo images with ConSLAM trajectories.

# How to cite
```bibtex
@inproceedings{trzeciak2022conslam,
  title={ConSLAM: Periodically Collected Real-World Construction Dataset for SLAM and Progress Monitoring},
  author={Trzeciak, Maciej and Pluta, Kacper and Fathy, Yasmin and Alcalde, Lucio and Chee, Stanley and Bromley, Antony and Brilakis, Ioannis and Alliez, Pierre},
  year={2022},
  booktitle={European Conference on Computer Vision Workshops 2022}
}
```
