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

dirlist="bin/ build/"
filelist="CMakeLists.txt.user Display* Planning* Perception* CreateRoadMap*"

cd ../
for d in $dirlist;do
rmDirFunc $d
done
cd components/
find . -name "*.pb.cc" -type f -print -exec rm -rf {} \;
find . -name "*.pb.h" -type f -print -exec rm -rf {} \;
for f in $filelist;do
rmFileFunc $f
done
cd ../log/
for f in $filelist;do
rmFileFunc $f
done

cd ../tools/
for d in $dirlist;do
rmDirFunc $d
done
for f in $filelist;do
rmFileFunc $f
done



