#！/bin/bash

rmDirFunc(){
if [ -d $1 ];then
rm -r $1
fi
}

rmFileFunc(){
if [ -f $1 ];then
rm -f $1
fi
}

cd ../
cd modules/
find . -name "*.pb.cc" -type f -print -exec rm -rf {} \;
find . -name "*.pb.h" -type f -print -exec rm -rf {} \;

dirlist="bin/ build/ lib/ log/"
filelist="CMakeLists.txt.user"

cd ../
for d in $dirlist;do
rmDirFunc $d
done

rm -f CMakeLists.txt.user

mkdir log





