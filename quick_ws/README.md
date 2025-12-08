# install

```bash

sudo apt remove libgoogle-glog-dev libgoogle-glog0v5
wget https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz
tar -xzf v0.6.0.tar.gz
cd glog-0.6.0
mkdir build && cd build
cmake -DWITH_GFLAGS=OFF -DBUILD_TESTING=OFF ..
make -j$(nproc)
sudo make install
```

```
  <depend>autoware_lanelet2_extension</depend>
  <depend>autoware_lanelet2_utils</depend>
```

pugixml-dev 이거 autoware_lanelet2_extension 에서 필요