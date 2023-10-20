 #include <iostream>
 #include <addition.h>
 #include"cuda_runtime.h"
 #include"device_launch_parameters.h"
 #include "driver_types.h"
 #include "cuda.h"

//  #include "helper_cuda.h"
using namespace std;
 int main(int argc, char** argv)
 {
     int a=1,b=2,c;
 
     if(addition(a,b,&c))
         std::cout<<"c="<<c<<std::endl;
     else
         std::cout<<"Addition failed!"<<std::endl;
 
    int dev = 0;
    cudaDeviceProp deviceProp;
    cudaGetDeviceProperties(&deviceProp, dev);
    std::cout << "使用GPU device " << dev << ": " << deviceProp.name << std::endl;
    // std::cout << "SM的数量：" << devProp.multiProcessorCount << std::endl;
    // std::cout << "每个线程块的共享内存大小：" << devProp.sharedMemPerBlock / 1024.0 << " KB" << std::endl;
    // std::cout << "每个线程块的最大线程数：" << devProp.maxThreadsPerBlock << std::endl;
    // std::cout << "每个EM的最大线程数：" << devProp.maxThreadsPerMultiProcessor << std::endl;
    // std::cout << "每个EM的最大线程束数：" << devProp.maxThreadsPerMultiProcessor / 32 << std::endl;

        cout << "设备计算能力:" << deviceProp.major << "." << deviceProp.minor << endl;
        cout << "显卡时钟频率:" << deviceProp.clockRate * 1e-6f << " GHz" << endl;
        cout << "内存时钟频率:" << deviceProp.memoryClockRate * 1e-3f << " MHz" << endl;
        cout << "内存总线带宽:" << deviceProp.memoryBusWidth << " bit" << endl;
        cout << "总显存大小:" << deviceProp.totalGlobalMem / (1024.0 * 1024.0) << " MB" << endl;
        cout << "总常量内存大小:" << deviceProp.totalConstMem / 1024.0 << " KB" << endl;
        cout << "SM数量:" << deviceProp.multiProcessorCount << endl;
        cout << "每个SM最大线程数:" << deviceProp.maxThreadsPerMultiProcessor << endl;
        cout << "每个线程块(block)共享内存大小:" << deviceProp.sharedMemPerBlock / 1024.0 << " KB" << endl;
        cout << "每个线程块(block)的最大线程数:" << deviceProp.maxThreadsPerBlock << endl;
        cout << "每个线程块(block)的最大可用寄存器数:" << deviceProp.regsPerBlock << endl;
        cout << "线程束(wrap)尺寸:" << deviceProp.warpSize << endl;
        cout << "每个线程块(block)各个维度最大尺寸:" << deviceProp.maxThreadsDim[0] << " x " << deviceProp.maxThreadsDim[1] << " x " << deviceProp.maxThreadsDim[2] << endl;
        cout << "每个线程格(grid)各个维度最大尺寸:" << deviceProp.maxGridSize[0] << " x " << deviceProp.maxGridSize[1] << " x " << deviceProp.maxGridSize[2] << endl;
        cout << "最大存储间距:" << deviceProp.memPitch / (1024.0 * 1024.0) << " MB" << endl;

     return 0;

// 使用GPU device 0: NVIDIA GeForce RTX 2060
// 设备计算能力:7.5
// 显卡时钟频率:1.56 GHz
// 内存时钟频率:5501 MHz
// 内存总线带宽:192 bit
// 总显存大小:5904.44 MB
// 总常量内存大小:64 KB
// SM数量:30
// 每个SM最大线程数:1024
// 每个线程块(block)共享内存大小:48 KB
// 每个线程块(block)的最大线程数:1024
// 每个线程块(block)的最大可用寄存器数:65536
// 线程束(wrap)尺寸:32
// 每个线程块(block)各个维度最大尺寸:1024 x 1024 x 64
// 每个线程格(grid)各个维度最大尺寸:2147483647 x 65535 x 65535
// 最大存储间距:2048 MB

 }
