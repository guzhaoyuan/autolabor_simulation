# IMR localization

## Required package
To visualize the ground truth (total station data), a open source package `xlnt` is used to load data from .xlsx file.

To install `xlnt`, download the package from [the github repo](https://github.com/tfussell/xlnt).
```shell script
git clone https://github.com/tfussell/xlnt.git
cd xlnt
mkdir build
cd build
cmake ..
make
sudo make install
```
`xlnt` does not have a good cmake support. So you need to add a .cmake file to allow cmake to find the package.
Find the `Findxlnt.cmake` in [this issue](https://github.com/tfussell/xlnt/issues/412#issuecomment-531352571). Then put `Findxlnt.cmake` under `/usr/local/lib/cmake/xlnt`.

## TODO

- [x] Figure out the tranformation between base_link and camera.
- [X] Add rviz_visual_tools to visualize UWB in global frame.
- [X] Add UWB to global EKF.
- [x] The left and right UWB are reverse.