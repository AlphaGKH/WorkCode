#!/usr/bin/env sh

generateProto(){

if [ ! -d $2$1 ];then
echo $1" proto file not exit!"
else
protoc -I $2 --cpp_out=$2 $2$1/*.proto
fi
}

PB_DIR=../components


dit_dir_list="/common/adapters/proto /common/proto /perception/proto /planning/proto /map/proto /display/proto /control/proto"


for d in $dit_dir_list;do
generateProto $d ${PB_DIR}
done
