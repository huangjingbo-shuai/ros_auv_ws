## 启动
1. 假设已经安装了anaconda3或者miniconda3，进入虚拟环境`conda activate yolov11`
2. pip install pyyamal
3. pip install rospkg
4. pip install opencv-python
5. pip install ultralytics
6. `catkin_make`
7. `source ./devel/setup.bash`

## 注意事项
1. 启动usb发布节点的时候报错`[ERROR] [1741959553.339980]: 发生错误: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0`，解决方案：`conda install libffi==3.3`
2. 安装报错多半是网络原因，不断的尝试`proxy`,`unproxy`
3. `catkin_make`如果报错：`Invoking "make cmake_check_build_system" failed`，解决方案：`pip install empy`，之前师兄碰到这个问题要安装指定的某一个版本才可以`pip install empy==3.3.4`，但我这里不指定也可以，留一个心眼注意，以防某一天出现这个问题解决不了，节省开发时间。