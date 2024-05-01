echo "Configuring and building Thirdparty/DBoW2 ..."
#编译DBoW2
cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
#指定并行运行任务的数量，如果提供了多个-j，最后一个是有效的。如果-j没有给定具体的数量，make命令将不限制并行任务的数量
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
#编译g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."
#解压字典
#-xf，其中x表示解压。f表示需要解压的文件名称。
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."
#编译orb源代码
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
