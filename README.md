# Strelka Quadruped SDK
# Build using CMake
```
mkdir ~/strelka/build
cd ~/strelka
cmake -S . -B build -DCMAKE_BUILD_TYPE=[Debug|Release]
cmake --build build
```
# Install
```
cd ~/strelka/build
sudo make install
sudo ldconfig
```
# Uninstall
```
sudo ~/strelka/uninstall.sh
```
# Run tests
```
cd build && ctest --output-on-failure
```
# Profile LCM messages
```
lcm-mon -t ~/strelka/strelka_messages/lcm
```