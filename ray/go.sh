rm -rf raycheck.out/*
cd build
make -j 8
cd ..
# raycheck.py --scenes temp --ref ../linux_binary/ray
raycheck.py --scenes ../Part1/scenes/ --ref ../linux_binary/ray