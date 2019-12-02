------------------------------------------------------------------------->
1.工程目录结构
CenterLine
├── build.sh
├── CMakeLists.txt
├── include
│   ├── Calculate.h
│   ├── FileAccess.h
│   ├── Optimizer.h
│   └── Point.h
└── src
    ├── Calculate.cpp
    ├── FileAccess.cpp
    ├── main.cpp
    └── Optimizer.cpp
其中src目录存放源代码，include目录存放头文件
------------------------------------------------------------------------->
2.编译
直接运行CenterLine目录下的脚本build.sh编译项目，或者在CenterLine目录下依次执行如下命令
mkdir build && cd build

cmake ..

make
注：编译前请先source命令执行routing_test目录下的setup.bash
如果已经存在build目录，则直接进入build目录下执行make命令进行编译
------------------------------------------------------------------------->
3.运行
运行build目录下的lineExt
使用方法 ./lineExt ../case/boundary_*.txt

运行后将在当前目录下生成中心线文件centerline_*.txt
------------------------------------------------------------------------->
