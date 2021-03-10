改版的recast软件，主要支持两个功能：  
1.去除封闭空间行走面  
2.支持空中寻路功能  

工程生成：
========
1.cd RecastDemo 所在目录，后执行 premake5 vs2017  
2.vs 工程对应为 Release Win64 或 Debug Win64  

RecastDemo 现在只是支持win下生成对应数据（如寻路，体素等二进制数据），Apple太贵了。。  
OurRecast 为单独导出跨平台嵌入其他语言子工程，用于生成静态库 现支持Java(jni),C# 等。  
OurRecast要嵌入Linux，Andriod下，可直接使用CMakeLists(需修改CMakeLists中Java所在Jni路径)  

![f033f0edf207e8621aa44a00f41355b](https://user-images.githubusercontent.com/35165340/110571933-67685b00-8193-11eb-9d2d-a75df596cade.png)  

后直接  
mkdir Build  
cd Build  
cmake ..  
make  

生成 libOurRecast.so 即可
生成数据：
========
sample 选择为 Only Mesh  
点击屏幕内一点作为寻路开始坐标，后点击Build  
Save为保存寻路二进制数据,路径为:RecastDemo/bin navmesh_***.bytes  
Save3D为保存三维寻路体素所用文件:RecastDemo/bin navmesh_***3D.bytes  

三维寻路  
========
1.右侧 build -> Save3D -> load
2.左侧 Tool -> Test Navmesh -> Pathfind Fly 
