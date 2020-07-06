#!/usr/bin/env sh

generateProto(){
 protoc -I=../ --cpp_out=../ ../$1/proto/*.proto
}

pb_dir_list="modules/common modules/perception modules/canbus modules/planning modules/prediction"


for d in $pb_dir_list;do
 if [ ! -d ../$d/proto ];then
  echo $1" proto dir not exit!"
 else
  generateProto $d
 fi
done
