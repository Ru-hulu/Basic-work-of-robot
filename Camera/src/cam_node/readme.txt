这里记录一些关于动态连接库的debug
有的时候引用了外部的动态连接库，可能在编译build的时候不会出现问题，但是在运行的时候会出现
找不到文件或者目录的情况。
可以把头文件，动态连接库所在的include, lib文件夹复制到包中
然后CMakeList中这样写
include_directories(include)
link_directories(lib)
这两句话是找到头文件和动态连接库

install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY lib/  DESTINATION lib)
这两句话是将头文件和动态连接库安装到工作空间中的install文件夹下
这样可以解决问题
玄学。