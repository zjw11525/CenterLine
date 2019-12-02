2019集成电路EDA设计精英挑战赛二等奖作品 华大九天反求图形中心线赛题

1.工程目录结构
其中src目录存放源代码，include目录存放头文件，case目录存放测试样例

2.编译
直接运行CenterLine目录下的脚本build.sh编译项目，或者在CenterLine目录下依次执行如下命令
mkdir build && cd build

cmake ..

make

3.运行
运行build目录下的lineExt
使用方法 ./lineExt ../case/boundary_*.txt

运行后将在当前目录下生成中心线文件centerline_*.txt
