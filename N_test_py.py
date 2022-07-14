import numpy as np
import time

count = 1e6  #定义向量的个数1e6=1000000个数据
a = np.random.rand(int(count))  #Numpy中的random.rand()主要用于返回一个或一组0到1之间的随机数或随机数组
b = np.random.rand(int(count))  #a可能为此 数组/向量 “[0.24553644 0.17621858 0.07568902 ... 0.53442229 0.74213432 0.05280213]”

tic = time.time() #记录起始时间
#tic可能为此数据 1648569193.4429026
#tic*1000可能为此数据 1648569236296.0916
c = np.dot(a,b)  #x是m × n 矩阵 ，y是n×m矩阵，则x.dot(y) 得到m×m矩阵。如果处理的是一维数组，则得到的是两数组的点积。
toc = time.time()  #记录结束时间
time_elapse_1 = toc - tic
print('向量计算结果为：' + str(c))
print('所用时间为：' + str(1000*time_elapse_1) + 'ms')


c = 0
tic = time.time()
for i in range(int(count)):  #使用for循环计算向量a b 的点积
    c += a[i] * b[i]
toc = time.time()
time_elapse_2  = toc - tic
print('向量计算结果为：' + str(c))
print('所用时间为：' + str(1000*time_elapse_2) + 'ms')

print('计算长度为1e6向量的点积时，for循环法所消耗的时间是向量运算法消耗时间的' + str(time_elapse_2/time_elapse_1) + '倍！')

