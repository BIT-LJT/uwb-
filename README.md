# uwb-
依赖四个基站对标签进行定位
Solve_the_location包括了2d和3d解算，但是都设定了是四个基站，且基站位置应当在代码中写入
calibration中是对基站和标签得到的距离数据进行标定，可以看到里面包含一个图片，图片上拟合的线性方程就是用来修正测量距离得到真实距离的函数
一帧数据提共的是8个基站数据，具体的数据结构可以在Solve_the_location中对数据进行unpack的时候看到
