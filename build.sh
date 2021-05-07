echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCUDA_INCLUDE_DIRS=/usr/local/cuda/include -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCUDA_INCLUDE_DIRS=/usr/local/cuda/include -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCUDA_INCLUDE_DIRS=/usr/local/cuda/include -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64
make -j
