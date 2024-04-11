![《opencv算法精解》全书笔记](https://picx.zhimg.com/70/v2-b48669c57de9e5fb054f638e2d553726_1440w.image?source=172ae18b&biz_tag=Post)

![](E:\typora\Project\OpenCV4.assets\2.jpg)

# 二、图像数字化

## 2.1 认识OpenCV中的Mat类

### 2.1.1 初识Mat

构造Mat对象相当于构造了一个矩阵（数组），需要四个基本要素：行数、列数、通道数及其数据类型

```c++
Mat(int rows,int cols,int type)
```

- rows 矩阵的行数
- cols 矩阵的列数
- type 类型（包括通道数及数据类型）

​		可以设置为CV_8UC(n)、CV_8SC(n)、CV_16SC(n)、CV_16UC(n)、CV_32SC(n)、CV_32FC(n)、CV_64FC(n)

​			其中数字代表一个数值代表的bit数，一字节=8Bit。

​			所以32F是4字节的float类型，64F是8字节的double类型、32S是占4字节的int类型。

​			C(n)代表通道数，当n=1时，即构造单通道矩阵或者称二维矩阵；n>1时，构造多通道矩阵即三维矩阵。

于是构造Mat也可以采取以下形式

```c
Mat(Size(int cols,int rows),int type)
```

### 2.1.2 构造单通道Mat对象

直接构造

```C
Mat m = Mat(2,3,CV_32FC1);
```

借助Size构造

```c
Mat m;
m.create(2,3,CV_32FC1);
```

构造全是1的矩阵

```c
Mat o=Mat::ones(2,3,CV_32FC1);
```

构造全是0的矩阵

```
Mat o=Mat::zeros(2,3,CV_32FC1);
```

### 2.1.3 获取单通道Mat的基本信息

使用rows和cols获取矩阵的行数和列数

```c++
Mat m=(Mat_<int>(3,2)<<11,12,33,43,51,16);
cout<<"行数"<<m.rows;
cout<<"列数"<<m.cols;
```

使用size()获取矩阵的尺寸

```c
Size size=m.size()
cout<<"尺寸："<<size<<endl;
```

使用channels()获取矩阵的通道数

```c
cout<<"通道数"<<m.channels()<<end;
```

成员函数total() 面积=行数乘以列数

```c++
cout<<"面积"<<m.total()<<endl;
```

成员函数dims代表矩阵的维数

```c
cout<<"维数"<<m.dims<<endl;
```



## 2.4 彩色图像数字化

#### cv::split

常用于图像通道的分离，另外，cv::merge()用于实现图像通道的合并，是split的逆向操作。

```c++
void split
(
    InputArray;
    OutputArrayOfArrays mv;
);
```

- m 是输入的需要分离通道的图像。
- mv 是输出的vector容器，装载不同通道的图像信息。

```c++
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace cv;

int main(int argc, char* argv[])
{
	//输入图像矩阵
	//注意OpenCV4中为IMREAD_GRAYSCALE
	std::string path = "F:\\Visuo studio\\Project\\Opencv\\Photo\\2.jpg";
	cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (img.empty())
	{
		return -1;
	}
	//显示彩色图像
	imshow("BGR", img);
	//分离通道
	std::vector<Mat> planes;
	cv:split(img, planes);
	//显示B通道
	imshow("B", planes[0]);
	//显示G通道
	imshow("G", planes[1]);
	//显示R通道
	imshow("R", planes[2]);
	waitKey(0);

}
```



# 三、几何变换

## 1.计算仿射变换

### 1.1 仿射法

```c++
//原位置坐标
Point2f src[]={Point2f(0,0),Point2f(200,0),Point2f(0,200)};
//经过某仿射变换后的坐标
Point2f dst[]={Point2f(0,0),Point2f(200,0),Point2f(0,200)};
//计算仿射矩阵
Mat A = getAffineTransform(src,dst);
```

### 1.2矩阵法

```c++
Mat s=(Mat_<float>(3,3)<<0.5,0,0,0,0.5,0,0,0,2);//缩放矩阵
Mat t=(Mat_<float>(3,3)<<1,0,100,0,1,200,0,0,1);//缩放矩阵
Mat A;
gemm(s,t,1.0,Mat(),0,A,0);//矩阵相乘
```

# 四、对比度增强

## 5.全局直方图均衡化

`CV_Assert()`是OpneCV中的带参数宏定义，它的作用是：若括号中的表达式值为false，则返回一个错误信息，并终止程序执行。

`CV_8UC1`的定义：

bit_depth—比特数—代表8bite,16bites,32bites,64bites                                                                                                         举个例子吧–比如说,你现在创建了一个存储–灰度图片的Mat对象,这个图像的大小为宽100,高100,那么,现在这张灰度图片中有10000个像素点，它每一个像素点在内存空间所占的空间大小是8bite,8位–所以它对应的就是CV_8

S|U|F–S--代表—signed int—有符号整形
U：代表–unsigned int–无符号整形
F：代表–float---------单精度浮点型

C<number_of_channels>----代表一张图片的通道数,比如:
1：灰度图片–grayImg—是–单通道图像
2：RGB彩色图像---------是–3通道图像
3：带Alph通道的RGB图像–是--4通道图像



# 五、图像平滑



# 六、阈值分割

​		把图像分割成若干个特定的、具有独特性质的区域，每一个区域代表一个像素的集合，每一个集合又代表一个物体，而完成该过程的技术通常被称为图像分割。

​		图像分割的方式有以下四种：基于阈值的分割方法、基于区域的分割方法、基于边缘的分割方法、基于特定理论的分割方法。以下重点介绍基于阈值的分割方法。

​		基于阈值的分割方法：

​		阈值分割是一种基于区域的、简单的通过灰度值信息提取形状的技术。往往阈值分割后的输出图像只有两种灰度值：255和0。所以阈值分割又常称为图像的二值化处理。

## 6.1 方法概述

### 6.1.1	全局阈值分割

​		全局阈值分割指的是将灰度值大于thresh(阈值)的像素设为白色，小于或者等于阈值的像素设为黑色；或者反过来将大于阈值的像素设为黑色，小于或者等于阈值的像素设为白色。

​		分割步骤：设定阈值-->图像处理

​		OpenCV中提供了相对应的函数`threshold`进行全局阈值分割。

### 6.1.2   阈值函数threshold

```c
threshold(InputArray src,OutputArray dst,double thresh,double maxval,int type)
```

​	**thresh:**为设定的阈值，取值范围即为灰度值的范围0~255，数据类型为浮点型（输入可以为整型）；

​    **result:**为进行阈值分割后的结果图像，数据类型为整数矩阵；

​    **src:**为被进行分割的源图像，一般为单通道的灰度图，但三通道的RGB图像也可以进行处理（但可能只根据第一个通道的大小进行处理（所谓枪打出头鸟））

​     **maxval:**为最大值，为分割后的图像所取到的灰度最大值

​     **type:**为阈值分割的类型，常用的有THRESH_BINARY、THRESH_BINARY_INV、THRESH_TOZERO、THRESH_TOZERO_INV。具体的说明可见下表所示：

| THRESH_BINARY_INV | 灰度值不超过阈值的像素设置为最大灰度值，超过的设置为0     |
| ----------------- | --------------------------------------------------------- |
| THRESH_TOZERO     | 灰度值低于阀值的像素设为0灰度值                           |
| THRESH_TOZERO_INV | 灰度值高于阀值的像素设为0灰度值                           |
| THRESH_TRUNC      | 灰度值超过阈值的像素设为阈值的灰度值                      |
| THRESH_MASK       | 掩码                                                      |
| THRESH_OTSU       | 标记，使用大津算法来选择最佳阈值，只支持8位单通道图像     |
| THRESH_TRIANGLE   | 标记，使用TRIANGLE算法来算则最佳阈值，只支持8位单通道图像 |
| THRESH_BINARY     | 灰度值超过阈值的像素设置为最大灰度值，不超过的设置为0     |

```c
void GlobalTreshold()
{
	//输入矩阵为5 行 3列
	Mat src = (Mat_<uchar>(5, 3) << 123, 234, 68, 33, 51, 17, 48, 98, 234, 129, 89, 27, 45, 167, 134);
	//第一种情况：手动设置阈值
	double the = 150;
	Mat dst;
	threshold(src, dst, the, 255, THRESH_BINARY);
	//第二种情况：Otsu算法
	double otsuThe = 0;
	Mat dst_Otsu;
	otsuThe = threshold(src, dst_Otsu, otsuThe, 255, THRESH_OTSU + THRESH_BINARY);
	cout << "计算的OTSU阈值" << otsuThe<< endl;
	//第三种情况：TRIANGLE 算法
	double triThe = 0;
	Mat dst_tri;
	triThe = threshold(src, dst_tri, 0, 255, THRESH_TRIANGLE + THRESH_BINARY);
	cout << "计算的TRIANGLE阈值" << triThe << endl;

}
```

### 6.1.3 局部阈值分割

​		在很多情况下，如受光照不均等因素影响，需要进行局部阈值分割。它不像全局分割一样，对整个矩阵只有一个阈值，它针对输入矩阵的每一个位置的值都有相对应的阈值。

以下详细介绍自动计算全局阈值的常用方法。

## 6.2	直方图技术法

一幅含有一个与背景呈现明显对比的物体的图像具有包含双峰的直方图，两个峰值对应于物体内部和外部较多数目的点，但由于直方图在直方图的随机波动，两个波峰（局部最大值）和波谷（局部最小值）不好确定

<img src="https://img-blog.csdnimg.cn/20210109172316594.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L20wXzM4MDA3Njk1,size_16,color_FFFFFF,t_70#pic_center" alt="在这里插入图片描述" style="zoom: 33%;" />

### 6.2.1 原理详解

第一步：找到灰度直方图的第一个峰值，并找到其对应的灰度值。显然，灰度直方图的最大值就是第一个峰值且对应的灰度值用firstPeak表示。

第二步：找到灰度直方图的第二个峰值，并找到其对应的灰度值。第二个峰值不一定是直方图的第二大值，因为它很可能出现在第一个峰值的附近。

第三步：找到这两个峰值之间的波谷，如果出现两个或者多个波谷，则取左侧的波谷即可，其对应的灰度值即为阈值。

### 6.2.2 c++实现

```c
Mat calcGrayHist(const Mat & image)
{
    Mat histogram = Mat::zeros(Size(256, 1), CV_32SC1);
    int rows = image.rows;
    int cols = image.cols;
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            int index = int(image.at<uchar>(r, c));
            histogram.at<int>(0, index) += 1;
        }
    }
    return histogram;
}

int threshTwoPeaks(const Mat& image, Mat& thresh_out)
{
  // 计算灰度直方图
  Mat histogram = calcGrayHist(image);
  // 找到灰度直方图最大峰值对应的灰度值
  Point firstPeakLoc;
  minMaxLoc(histogram, NULL, NULL, NULL, &firstPeakLoc);
  int firstPeak = firstPeakLoc.x;
  //寻找灰度直方图的第二个峰值对应的灰度值
  Mat measureDists = Mat::zeros(Size(256,1), CV_32FC1);
  for(int k=0;k<256;k++)
  {
    int hist_k = histogram.at<int>(0,k);
    measureDists.at<float>(0,k) = pow(float(k-firstPeak), 2) * hist_k;
  }
  Point secondPeakLoc;
  minMaxLoc(measureDists, NULL, NULL, NULL, &secondPeakLoc);
  int secondPeak = secondPeakLoc.x;
  //找到两个峰值之间的最小值对应的灰度值，作为阈值
  Point threshLoc;
  int thresh = 0;
  if(firstPeak < secondPeak){
    minMaxLoc(histogram.colRange(firstPeak, secondPeak), NULL, NULL, &threshLoc);
    thresh = firstPeak + threshLoc.x + 1;
  }else{
    minMaxLoc(histogram.colRange(secondPeak, firstPeak), NULL, NULL, &threshLoc);
    thresh = secondPeak + threshLoc.x + 1;
  }
  //阈值分割
  threshold(image, thresh_out, thresh, 255, THRESH_BINARY);
  return thresh;
}

```

## 6.3 熵算法

### 6.3.1 原理详解

信息熵的概念来源于信息论，假设信源符号u有N种取值，记为
$$
p_1,p_2,...,p_N
$$
那么该信息源的信息熵，记为
$$
entropy(u)=-∑Np_ilogp_i
$$
​		图像也可以看作一种信源，把信息熵的概念带入图像就是，图像的信息熵越大（信息量大），所包含的细节越多，图像就越清晰。假设输入图像为 *I*，normHist_I 代表归一化的图像灰度直方图，那么对于 8 位图可以看成由 256 个灰度符号，且每一个符号出现的概率为 normHist I ( k ) 组成的信源。

​		利用熵计算阈值的步骤如下：

​		第一步：计算I的累加概率直方图，又称为零阶累积矩，记为
$$
cumuHist(k)=-∑p_ilogp_i,k∈[0,255]
$$
​		第二步：计算各个灰度级的熵，记为：
$$
entropy(t)=-∑normHist_I(k)log(normHist_I(k)),0≤t≤255
$$
​		第三步

![image-20231216145020811](E:\typora\Project\OpenCV4.assets\image-20231216145020811.png)



## 6.4 OTSU阈值分割

### 6.4.1 原理详解

 	  这个处理是为了克服我们手动设置的阈值x对当前图像来说不合理的情况。比如当前图像的像素值都大于125，然后我们设置一个阈值为125，用二值化处理，则图像的像素值都会变为255，其图像是一个白板，这样进行阈值处理是不可取的，那么我们可以通过Otsu进行处理，可以根据当前图像中的所有像素值，设定一个最优的阈值，它会综合考虑所有可能的阈值。

### 6.4.2 代码详解

```c++
//Otsu 阈值处理：得到阈值处理后的二值图 OtsuThreshImage，并返回分割阈值
int otsu(const Mat& image, Mat& OtsuThreshImage)
{
	//计算灰度直方图
	Mat histogram = calcGrayHist(image);
	//归一化灰度直方图
	Mat normHist;
	histogram.convertTo(normHist, CV_32FC1, 1.0 / (image.rows * image.cols), 0.0);
	//计算累加直方图(零阶累加矩)和一阶累加矩
	Mat zeroCumuMoment = Mat::zeros(Size(256, 1), CV_32FC1);
	Mat oneCumuMoment = Mat::zeros(Size(256, 1), CV_32FC1);
	for (int i = 0; i < 256; i++)
	{
		if (i == 0)
		{
			zeroCumuMoment.at<float>(0, i) = normHist.at<float>(0, i);
			oneCumuMoment.at<float>(0, i) = i * normHist.at<float>(0, i);
		}
		else
		{
			zeroCumuMoment.at<float>(0, i) = zeroCumuMoment.at<float>(0, i - 1) + normHist.at<float>(0, i);
			oneCumuMoment.at<float>(0, i) = oneCumuMoment.at<float>(0, i - 1) + i * normHist.at<float>(0, i);
		}
	}
	//计算类间方差
	Mat variance = Mat::zeros(Size(256, 1), CV_32FC1);
	//总平均值
	float mean = oneCumuMoment.at<float>(0, 255);
	for (int i = 0; i < 255; i++)
	{
		if (zeroCumuMoment.at<float>(0, i) == 0 || zeroCumuMoment.at<float>(0, i) == 1)
			variance.at<float>(0, i) = 0;
		else
		{
			float cofficient = zeroCumuMoment.at<float>(0, i) * (1.0 - zeroCumuMoment.at<float>(0, i));
			variance.at<float>(0, i) = pow(mean * zeroCumuMoment.at<float>(0, i) - oneCumuMoment.at<float>(0, i), 2.0) / cofficient;
		}
	}
	//找到阈值
	Point maxLoc;
	minMaxLoc(variance, NULL, NULL, NULL, &maxLoc);
	int otsuThresh = maxLoc.x;
	//阈值处理
	threshold(image, OtsuThreshImage, otsuThresh, 255, THRESH_BINARY);
	return otsuThresh;
}


```

## 6.5 自适应阈值

### 6.5.1 原理详解

​		我们在进行阈值处理时，如果说我们的图像色彩不均衡的话，我们通过一个阈值x，无法进行有效的分割，这时我们就可以使用自适应阈值处理，它就像一个掩膜，关于掩膜概念不懂得话，可以看一下我的图像处理笔记03，这种阈值的处理是根据当前像素点的周围邻域内的像素值的加权平均，获得一个阈值，获得阈值之后的操作就和前面的一样了。

### 6.5.2 代码详解

```c
Mat adaptiveThreshold_S(const Mat& image, Size blockSize, float ratio)
{
	//图像的宽高
	int rows = image.rows;
	int cols = image.cols;
	//阈值处理后的二值图
	Mat threshImage = Mat::zeros(image.size(), CV_8UC1);
	//
	int h = (blockSize.height - 1) / 2;
	int w = (blockSize.width - 1) / 2;
	//图像的积分
	Mat imageIntegral;
	integral(image, imageIntegral, CV_32FC1);
	imageIntegral = imageIntegral(Rect(1, 1, cols, rows)).clone();

for (int r = 0; r < rows; r++)
{
	for (int c = 0; c < cols; c++)
	{
		int tl_r = r - h > 0 ? r - h : 0;
		int tl_c = c - w > 0 ? c - w : 0;
		int br_r = r + h < rows ? r + h : rows - 1;
		int br_c = c + w < cols ? c + w : cols - 1;
		//计算区域和
		float regionSum = imageIntegral.at<float>(br_r, br_c) +
			imageIntegral.at<float>(tl_r, tl_c) -
			imageIntegral.at<float>(tl_r, br_c) -
			imageIntegral.at<float>(br_r, tl_c);
		int count = (br_r - tl_r + 1) * (br_c - tl_c + 1);
		int pixel = int(image.at<uchar>(r, c));
		if (pixel * count < (1 - ratio) * regionSum)
			threshImage.at<uchar>(r, c) = 0;
		else
			threshImage.at<uchar>(r, c) = 255;
	}
}
return threshImage;

}
//自适应阈值分割
Mat adaptiveThresh(Mat I, int radius, float ratio, METHOD method)
{
	//第一步：图像矩阵的平滑处理
	Mat I_smooth;
	switch (method)
	{
	case MEAN:
		boxFilter(I, I_smooth, CV_32FC1, Size(2 * radius + 1, 2 * radius + 1));//均值平滑
		break;
	case GAUSS:
		GaussianBlur(I, I_smooth, Size(2 * radius + 1, 2 * radius + 1), 0, 0);//高斯平滑
		I_smooth.convertTo(I_smooth, CV_32FC1);
		break;
	case MEDIAN:
		medianBlur(I, I_smooth, 2 * radius + 1);//中值平滑
		I_smooth.convertTo(I_smooth, CV_32FC1);
		break;
	default:
		break;
	}
	//第二步：平滑结果乘以比例系数，然后图像与其做差
	I.convertTo(I, CV_32FC1);
	Mat diff = I - (1.0 - ratio) * I_smooth;
	//第三步：阈值处理：大于等于 0 的，输出 255 ，反之输出 0
	Mat out = Mat::zeros(diff.size(), CV_8UC1);
	for (int r = 0; r < out.rows; r++)
	{
		for (int c = 0; c < out.cols; c++)
		{
			if (diff.at<float>(r, c) >= 0)
				out.at<uchar>(r, c) = 255;
		}
	}
	return out;
}
```



## 6.6 二值图的逻辑运算

### 6.6.1 “与”和“或”运算

​		对于阈值阈值处理后的二值图，还可以利用二值图之间的逻辑运算：“与”运算和“或”运算，以便得到想要的结果。

​		`bitwis_and`和`bitwise_or`分别实现了与运算和或运算。

### 6.6.2 代码详解

```c
Mat src1 = Mat::zeros(Size(100, 100), CV_8UC1);
	cv::rectangle(src1, Rect(25, 25, 50, 50), Scalar(255), CV_FILLED);
	imshow("src1", src1);
	Mat src2 = Mat::zeros(Size(100, 100), CV_8UC1);
	cv::circle(src2, Point(75, 50), 25, Scalar(255), CV_FILLED);
	imshow("src2", src2);
	//与运算
	Mat dst_and;
	bitwise_and(src1, src2, dst_and);
	imshow("与运算", dst_and);
	//或运算
	Mat dst_or;
	bitwise_or(src1, src2, dst_or);
	imshow("或运算", dst_or);
	//非运算
	Mat dst_not;
	bitwise_not(src1, dst_not);
	imshow("非运算", dst_not);
	waitKey(0);
	return 0;
```



# 七、形态学处理

常用的形态学处理方法包括：腐蚀、膨胀、开运算、闭运算、顶帽运算、底帽运算。

其中腐蚀与膨胀是最基础的方法，其他方法是两者相互组合而产生的。

## 7.1 腐蚀

### 7.1.1 原理详解

结构元：与平滑操作类似，在平滑操作中使用的是矩形邻域，而在形态学处理中邻域可以是矩形结构，也可以是椭圆形、十字交叉形结构。同样也需要指定一个锚点。

​		在腐蚀操作中，是取结构元中的最小值作为锚点的值。可以对灰度图或二值图做腐蚀操作。

![img](https://img-blog.csdnimg.cn/20210116153011132.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L20wXzM4MDA3Njk1,size_16,color_FFFFFF,t_70#pic_center)

​		上方三个图中的邻域的最小值分别为 11、21、21，这些最小值输出到图像中的锚点位置，其他位置通过移动结构元以此类推，即可得到完整的输出图像。

​		由上图可知，因为取每个位置邻域内的最小值，所以腐蚀后输出图像的总体亮度的平均值比起原图会有所降低，图像中比较亮的区域的面积会变小甚至消失，而比较暗的区域的面积会增大。

​		因为对图像进行腐蚀操作后缩小了亮度区域的面积，所以针对阈值分割后前景时白色的二值图，可以通过原图与腐蚀后的图像相减得到前景的边界。

### 7.1.2 代码详解

```c++
CV_EXPORTS_W void erode( InputArray src, OutputArray dst, InputArray kernel,
                         Point anchor = Point(-1,-1), int iterations = 1,
                         int borderType = BORDER_CONSTANT,
                         const Scalar& borderValue = morphologyDefaultBorderValue() );
```

此函数实现了图像的腐蚀

- src					输入矩阵，灰度图或二值图
- dst	               输出矩阵
- kernel	          结构元，可以通过 getStructuringElement 函数获取
- anchor        	 锚点的位置；默认值（-1，-1）表示锚点在结构元中心。
- iterations	     腐蚀的次数
- borderType     边界扩充类型（可不在使用的时候声明）
- borderValue	边界扩充值（可不在使用的时候声明）

```c
Mat cv::getStructuringElement(int 	shape,
							Size 	ksize,
							Point 	anchor = Point(-1,-1) 
							)		
```

此函数返回设定的结构元

|  int shape   | 结构元形状 |
| :----------: | :--------: |
|  Size ksize  | 结构元大小 |
| Point anchor | 结构元锚点 |

总体代码实现如下：

```c
//输入图像
	Mat I = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	if (!I.data)
		return -1;
	//创建矩形结构元
	Mat s = cv::getStructuringElement(cv::MORPH_RECT, Size(3, 3));
	//腐蚀图像
	Mat E;
	cv::erode(I, E, s,Point(-1,-1),2);
	//显示图像
	imshow("I", I);
	imshow("erode", E);
	waitKey(0);
	return 0;


```





## 7.2 膨胀

### 7.2.1 原理详解

在膨胀操作中，是取结构元中的最大值作为锚点的值。可以对灰度图或二值图做膨胀操作。膨胀后输出图像的总体亮度的平均值比起原图会有所上升，图像中比较亮的区域的面积会变大，而比较暗的区域的面积会减小。

### 7.2.2 代码详解

```c++
// 膨胀
// C++
void cv::dilate(InputArray 	  src,
				OutputArray   dst,
				InputArray 	  kernel,
				Point 	      anchor = Point(-1,-1),
				int 	      iterations = 1,
				int 	      borderType = BORDER_CONSTANT,
				const Scalar & 	borderValue = morphologyDefaultBorderValue() 
				)		
```

- src					输入矩阵，灰度图或二值图
- dst	               输出矩阵
- kernel	          结构元，可以通过 getStructuringElement 函数获取
- anchor        	 锚点的位置；默认值（-1，-1）表示锚点在结构元中心。
- iterations	     腐蚀的次数
- borderType     边界扩充类型（可不在使用的时候声明）
- borderValue	边界扩充值（可不在使用的时候声明）

## 7.3 开运算与闭运算

### 7.3.1 原理详解

​		腐蚀与膨胀是开运算与闭运算的基础

​		开运算是先腐蚀后膨胀的过程，它具有消除高度较高的细小区域、在纤细点处分离物体，对于较大物体，可以在不明显改变其面积的情况下平滑其边界等作用。

​		闭运算是先膨胀后腐蚀，它具有填充白色物体内细小黑色空洞的区域、连接临近物体，用一个结构元、多次迭代处理，可以在不明显改变其面积的情况下平滑其边界等作用。

### 7.3.2 代码详解

```c++
void cv::morphologyEx(InputArray 	src,
					OutputArray 	dst,
					int 	op,
					InputArray 	kernel,
					Point 	anchor = Point(-1,-1),
					int 	iterations = 1,
					int 	borderType = BORDER_CONSTANT,
					const Scalar & 	borderValue = morphologyDefaultBorderValue() 
					)；
```

|     src     |                     输出的图片                     |
| :---------: | :------------------------------------------------: |
|     dst     |                     输出的图片                     |
|     op      |                  形态学操作的类型                  |
|   kernel    |  结构元，可以通过 getStructuringElement 函数获取   |
|   anchor    | 锚点的位置；默认值（-1，-1）表示锚点在结构元中心。 |
| iterations  |                      迭代次数                      |
| borderType  |                边界扩充类型，见腐蚀                |
| borderValue |                 边界扩充值，见腐蚀                 |

op 形态学操作的类型

- `MORPH_ERODE`  腐蚀 erode
- `MORPH_DILATE`  膨胀 dilate
- `MORPH_OPEN`  开运算（先腐蚀后膨胀）
- `MORPH_CLOSE`  闭运算（先膨胀后腐蚀）
- `MORPH_GRADIENT`  形态学梯度
- `MORPH_TOPHAT`  顶帽运算
- `MORPH_BLACKHAT`  底帽运算



## 7.4 顶帽变换和底帽变换

### 7.4.1 原理详解

顶帽变换（Top-hat)与底帽变换（Botton-hat)是分别以开运算与闭运算为基础的。

**顶帽变换：**图像减去开运算结果，因为开运算可以消除暗背景下的较亮区域，那么如果用原图减去开运算，结果就可以得到原图中灰度较亮的区域。

**底帽变换：**图像减去闭运算结果，因为闭运算可以消除较高背景下的较暗区域，那么用原图减去闭运算结果就可以得到原图中灰度较暗的区域。

### 7.4.2 代码详解

如7.3.2 所示。

# 八 、边缘检测

​		图像的边缘指的是灰度值发生急剧变化的位置，有些时候，只需考虑图像的边缘就可以理解图像的内容。

​		边缘检测的目的是制作一个线图，在不会损害理解图像内容的情况下，同时又大大减少图像的数据量，提供了对图像数据的合适概述。

最简单的算子：Roberts

自带平滑的两方向算子：Prewitt,Sobel ,Scharr

各个方向的算子：Kirsch，Robinson

## 8.1 Roberts 算子

### 8.1.1 原理详解

Roberts 边缘检测是图像矩阵与以下两个卷积核：
$$
Roberts_{135}=\left[\begin{matrix}
1&0\\
0&-1
\end{matrix}\right]\\\\
Roberts_{45}=\left[\begin{matrix}
0&1\\
-1&0
\end{matrix}\right]
$$
​		Roberts算子**利用对角线方向**相邻两像素之差近似梯度幅值来检测边缘，检测垂直边缘的效果要优于其他方向边缘，定位精度高，但对噪声的抑制能力比较若，边缘检测算子检查每个像素的领域并对灰度变化了进行**量化**，同时也包含**方向的确定**。



### 8.1.2 代码详解

```c++
void conv2D(InputArray _src, InputArray _kernel, OutputArray _dst, int ddepth, Point anchor = Point(-1, -1), int borderType = BORDER_DEFAULT);
```

​		conv2D为卷积函数，当x≠0，y=0时，计算的是图像矩阵与Roberts135核的卷积，当x=0，y=0时，计算的是图像矩阵与Roberts45核的卷积。

```c
/*roberts 卷积*/
void roberts(InputArray src, OutputArray dst, int ddepth, int x, int y = 0, int borderType = BORDER_DEFAULT)
{
	CV_Assert(!(x == 0 && y == 0));
	Mat roberts_1 = (Mat_<float>(2, 2) << 1, 0, 0, -1);
	Mat roberts_2 = (Mat_<float>(2, 2) << 0, 1, -1, 0);
	//当 x 不等于零时，src 和 roberts_1 卷积
	if (x != 0)
	{
		conv2D(src, roberts_1, dst, ddepth, Point(0, 0), borderType);
	}
	//当 y 不等于零时，src 和 roberts_2 卷积
	if (y != 0)
	{
		conv2D(src, roberts_2, dst, ddepth, Point(0, 0), borderType);
	}
}
```



## 8.2 Prewitt边缘检测

### 8.2.1 Prewitt算子及分离性

​		标准的Prewitt边缘检测算子由以下两个卷积核：
$$
Prewitt_{x}=\left[\begin{matrix}
1&0&-1\\
1&0&-1\\
1&0&-1
\end{matrix}\right]\\\\
Prewitt_{y}=\left[\begin{matrix}
1&1&1\\
0&0&0\\
-1&-1&-1
\end{matrix}\right]
$$
​		图像与Prewitt_x卷积后可以反映图像垂直方向上的边缘，图像与Prewitt_y卷积后可以反映图像水平方向上的边缘，而且这两种边缘是可以分离的。

​		由于对图像进行了平滑处理，所以对噪声较多的图像进行Prewitt边缘检测得到的边缘比Roberts要好，可以对标准的Prewitt算子进行改进，比如：
$$
Prewitt_{135}=\left[\begin{matrix}
1&1&0\\
1&0&-1\\
0&-1&-1
\end{matrix}\right]\\\\
Prewitt_{45}=\left[\begin{matrix}
0&1&1\\
-1&0&1\\
-1&-1&0
\end{matrix}\right]
$$
​		反映的是在45°和135°方向上的边缘。

### 8.2.2 代码详解



## 8.3 Sobel边缘检测

### 8.3.1 Sobel算子及分离性

​		对于二项式展开式的系数，可以作为非归一化的高斯平滑算子，利用n=2时展开式的系数，把Prewitt算子中的非归一化的均值平滑算子换成该系数，把Prewitt算子中的非归一化的鄂均值平滑算子换成该系数，即可得到3阶的Sobel边缘检测算子：
$$
sobel_x=\left[\begin{matrix}
1&0&-1\\
2&0&-2\\
1&0&-1
\end{matrix}\right]\\\\
sobel_y=\left[\begin{matrix}
1&2&1\\
0&0&0\\
1&-2&-1
\end{matrix}\right]\\\\
$$
​		也可以利用二项式展开式的系数构建窗口更大的Sobel算子，如5×5、7×7。

​		优点：Sobel算子在进行边缘检测时**效率较高**，当对精度要求不是很高时，是一种较为常用的边缘检测方法；缺点：Sobel算子**对沿x轴和y轴的排列表示得较好**，但是对于其他角度得表示却不够精确

### 8.3.2 代码详解

```c++
Mat sobel(Mat image, int x_flag, int y_flag, int winSize, int borderType)
{
	// sobel 卷积核的窗口大小为大于 3 的奇数 
	CV_Assert(winSize >= 3 && winSize % 2 == 1);
	//平滑系数
	Mat pascalSmooth = getPascalSmooth(winSize);
	//差分系数
	Mat pascalDiff = getPascalDiff(winSize);
	Mat image_con_sobel;
	/* 当 x_falg != 0 时，返回图像与水平方向的 Sobel 核的卷积*/
	if (x_flag != 0)
	{
		//根据可分离卷积核的性质
		//先进行一维垂直方向上的平滑，再进行一维水平方向的差分
		sepConv2D_Y_X(image, image_con_sobel, CV_32FC1, pascalSmooth.t(), pascalDiff, Point(-1, -1), borderType);
	}
	/* 当 x_falg == 0 且 y_flag != 0 时，返回图像与垂直 方向的 Sobel 核的卷积*/
	if (x_flag == 0 && y_flag != 0)
	{
		//根据可分离卷积核的性质
		//先进行一维水平方向上的平滑，再进行一维垂直方向的差分
		sepConv2D_X_Y(image, image_con_sobel, CV_32FC1, pascalSmooth, pascalDiff.t(), Point(-1, -1), borderType);
	}
	return image_con_sobel;
}
```



### 8.3.3 处理结果

​																					垂直方向上：

![image-20231218141101462](E:\typora\Project\OpenCV4.assets\image-20231218141101462.png)

​																				   水平方向上：

![image-20231218141200119](E:\typora\Project\OpenCV4.assets\image-20231218141200119.png)

​																						边缘图：

![image-20231218141237393](E:\typora\Project\OpenCV4.assets\image-20231218141237393.png)

## 8.4 Scharr算子

### 8.4.1 原理详解

​		标准的Scharr边缘检测算子与Prewitt边缘检测算子和3阶的Sobel边缘检测算子类似，由以下两个卷积核：
$$
scharr_x=\left[\begin{matrix}
3&0&-3\\
10&0&-10\\
3&0&-3
\end{matrix}\right]\\\\
scharr_y=\left[\begin{matrix}
3&10&3\\
3&0&-3\\
0&-3&-10
\end{matrix}\right]\\\\
$$
​		这两个卷积核都不能分离。图像与水平方向上的scharr_x卷积结果反映的是垂直方向上的边缘强度，与垂直方向上的scharr_y卷积结果反映的是水平方向上的边缘强度。同样Scharr边缘检测算子也可以拓展到其他方向，比如：
$$
scharr_{45}=\left[\begin{matrix}
0&3&10\\
-3&0&3\\
-10&-3&0
\end{matrix}\right]\\\\
scharr_{135}=\left[\begin{matrix}
10&3&0\\
3&0&-3\\
0&-3&-10
\end{matrix}\right]\\\\
$$

### 8.4.2 代码详解

### 8.4.3 处理结果

垂直方向上的边缘：

![image-20231218143700328](E:\typora\Project\OpenCV4.assets\image-20231218143700328.png)

水平方向上的边缘：

![image-20231218143716137](E:\typora\Project\OpenCV4.assets\image-20231218143716137.png)

边缘强度：

![image-20231218143738586](E:\typora\Project\OpenCV4.assets\image-20231218143738586.png)

## 8.5 kirsch和Robinson算子

### 8.5.1原理详解

​		1.Kirsh算子由以下8个卷积核组成
$$
k_1=\left[\begin{matrix}
5&5&5\\
-3&0&-3\\
-3&-3&-3
\end{matrix}\right]\ 
k_2=\left[\begin{matrix}
-3&-3&-3\\
-3&0&-3\\
5&5&5
\end{matrix}\right]\
k_3=\left[\begin{matrix}
-3&5&5\\
-3&0&5\\
-3&-3&-3
\end{matrix}\right]\
k_4=\left[\begin{matrix}
-3&-3&-3\\
5&0&-3\\
5&5&-3
\end{matrix}\right]\\\\
k_5=\left[\begin{matrix}
-3&-3&5\\
-3&0&5\\
-3&-3&5
\end{matrix}\right]\
k_6=\left[\begin{matrix}
5&-3&-3\\
5&0&-3\\
5&-3&-3
\end{matrix}\right]\  
k_7=\left[\begin{matrix}
-3&-3&-3\\
-3&0&5\\
-3&5&5
\end{matrix}\right]\
k_8=\left[\begin{matrix}
5&5&-3\\
5&0&-3\\
-3&-3&-3
\end{matrix}\right]\\\\
$$
​		图像与每一个核进行卷积，然后取绝对值作为对应方向上的边缘强度的量化。对8个卷积结果取绝对值，然后再对应位置值取最大值作为最后输出的边缘强度。

​		2.Robinson边缘算子

​		暂略

### 8.5.2 代码详解

```c++
/* Krisch 边缘检测算法*/
Mat krisch(InputArray src, int borderType = BORDER_DEFAULT)
{
	//存储八个卷积结果
	vector<Mat> eightEdge;
	eightEdge.clear();
	/*第1步：图像矩阵与8 个 卷积核卷积*/
	/*Krisch 的 8 个卷积核均不是可分离的*/
	//图像矩阵与 k1 卷积
	Mat k1 = (Mat_<float>(3, 3) << 5, 5, 5, -3, 0, -3, -3, -3, -3);
	Mat src_k1;
	conv2D(src, k1, src_k1, CV_32FC1,Point(-1,-1), borderType);
	convertScaleAbs(src_k1, src_k1);
	eightEdge.push_back(src_k1);
	//图像矩阵与 k2 卷积
	Mat k2 = (Mat_<float>(3, 3) << -3, -3, -3, -3, 0, -3, 5, 5, 5);
	Mat src_k2;
	conv2D(src, k2, src_k2, CV_32FC1, Point(-1, -1), borderType);
	convertScaleAbs(src_k2, src_k2);
	eightEdge.push_back(src_k2);
	//图像矩阵与 k3 卷积
	Mat k3 = (Mat_<float>(3, 3) << -3, 5, 5, -3, 0, 5, -3, -3, -3);
	Mat src_k3;
	conv2D(src, k3, src_k3, CV_32FC1, Point(-1, -1), borderType);
	convertScaleAbs(src_k3, src_k3);
	eightEdge.push_back(src_k3);
	//图像矩阵与 k4 卷积
	Mat k4 = (Mat_<float>(3, 3) << -3, -3, -3, 5, 0, -3, 5, 5, -3);
	Mat src_k4;
	conv2D(src, k4, src_k4, CV_32FC1, Point(-1, -1), borderType);
	convertScaleAbs(src_k4, src_k4);
	eightEdge.push_back(src_k4);
	//图像矩阵与 k5 卷积
	Mat k5 = (Mat_<float>(3, 3) << -3, -3, 5, -3, 0, 5, -3, -3, 5);
	Mat src_k5;
	conv2D(src, k5, src_k5, CV_32FC1, Point(-1, -1), borderType);
	convertScaleAbs(src_k5, src_k5);
	eightEdge.push_back(src_k5);
	//图像矩阵与 k6 卷积
	Mat k6 = (Mat_<float>(3, 3) << 5, -3, -3, 5, 0, -3, 5, -3, -3);
	Mat src_k6;
	conv2D(src, k6, src_k6, CV_32FC1, Point(-1, -1), borderType);
	convertScaleAbs(src_k6, src_k6);
	eightEdge.push_back(src_k6);
	//图像矩阵与 k7 卷积
	Mat k7 = (Mat_<float>(3, 3) << -3, -3, -3, -3, 0, 5, -3, 5, 5);
	Mat src_k7;
	conv2D(src, k7, src_k7, CV_32FC1, Point(-1, -1), borderType);
	convertScaleAbs(src_k7, src_k7);
	eightEdge.push_back(src_k7);
	//图像矩阵与 k8 卷积
	Mat k8 = (Mat_<float>(3, 3) << 5, 5, -3, 5, 0, -3, -3, -3, -3);
	Mat src_k8;
	conv2D(src, k8, src_k8, CV_32FC1, Point(-1, -1), borderType);
	convertScaleAbs(src_k8, src_k8);
	eightEdge.push_back(src_k8);
	/*第二步：将求得的八个卷积结果,取对应位置的最大值，作为最后的边缘输出*/
	Mat krischEdge = eightEdge[0].clone();
	for (int i = 0; i < 8; i++)
	{
		max(krischEdge, eightEdge[i], krischEdge);
	}
	return krischEdge;
}
```

### 8.5.3 处理结果

![image-20231218160034325](E:\typora\Project\OpenCV4.assets\image-20231218160034325.png)

## 8.6 Canny边缘检测

### 8.6.1原理详解

基于卷积运算的边缘检测算法，比如Sobel、Prewitt等，有如下几个缺点：

(1)没有充分利用边缘的梯度方向。

(2)最后输出的边缘二值图，只是简单地利用阈值进行维护，显然如果阈值过大，则会损失很多边缘信息；如果阈值过小，则会有很多噪音。

而Canny边缘检测基于两点做了改进，提出了：

(1)基于边缘梯度方向的非极大值抑制。

(2)双阈值的滞后阈值处理。



#### **Canny边缘检测的近似算法的步骤如下：**

1. 图像矩阵分别与水平方向上的卷积核sobel_x和垂直方向上的卷积核sobel_y卷积得到dx和dy,然后利用平方和的开方得到边缘强度。这一步和Sobel边缘检测一样。

2. 利用第一步计算出的dx和dy,计算出梯度方向angle=arctan2(dy,dx)，即对每一个位置（r,c),用梯度表示，一般用角度的形式表示梯度。

3. 对每一个位置进行**非极大值抑制**的处理，它能找到局部极大值，并筛除（抑制）邻域内其余的值，非极大值抑制操作返回的仍然是一个矩阵。

   在获得梯度的方向和大小之后，应该对整幅图像做一个扫描，去除那些非边界上的点。对每一个像素进行检查，看这个点的梯度是不是周围具有相同梯度方向的点中最大的。如下所示：

   ![img](https://img-blog.csdn.net/20161118130215988)

​		上图中的数字代表了像素点的梯度强度，箭头方向代表了梯度方向。以第二排第三个像素点为例，由于梯度方向向上，则		将这一点的强度[7]与其上下两个像素点的强度[5和4]比较，由于这一点强度最大，则保留。

​		对于非极大值抑制的实现，有两种方法：

​		第一种方法：将梯度方向一般离散化为四种情况。0、90、45、135四个梯度方向上，将近似的方向近似。但这样会舍弃		部分信息，使得准确性降低。

​		第二种方法：用插值法拟合梯度方向上的边缘强度，这样会更加准确地衡量梯度方向上的边缘强度。

<img src="https://img-blog.csdnimg.cn/100afad86575453c93ce854df3c7958e.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBA5pqW5LuU5Lya6aOe,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom: 47%;" />

​		4.双阈值的滞后阈值处理。经过第三步非极大值抑制处理后的边缘强度图，一般需要阈值化处理，常用的方法是全局阈值		分割和局部自适应阈值分割。此处用另一种方法，滞后阈值处理，它使用两个阈值—高阈值(upperThresh)和低阈值		   		(lowerThresh),按照以下三个规则进行边缘化的阈值化处理。

​		(1)边缘强度大于高阈值的那些点作为确定边缘点。

​		(2)边缘强度比低阈值小的那些点立刻剔除。

​		(3)那些在高阈值与低阈值之间的点，只有那些点能够按照一定路径与确定边缘点相连时，才能够作为边缘点被接受。组成		这一路径的所有点的边缘强度都比低阈值要大。



### 8.6.2 代码详解

### 8.6.3 处理结果

​											边缘强度的灰度级显示           	                        非极大值抑制

<center class="half">    <img src="E:\typora\Project\OpenCV4.assets\image-20231218173919975.png" width="300"/>    <img src="E:\typora\Project\OpenCV4.assets\image-20231218173950041.png" width="300"/> </center>

​												Canny边缘                                               低阈值处理后的图像

<center class="half">    <img src="E:\typora\Project\OpenCV4.assets\image-20231218174019459.png" width="300"/>    <img src="E:\typora\Project\OpenCV4.assets\image-20231218174237515.png" width="300"/> </center>																	

​																			高阈值处理后的图像

![image-20231218174252400](E:\typora\Project\OpenCV4.assets\image-20231218174252400.png)

## 8.7 Laplacian 算子

### 8.7.1 原理详解

Laplace算子的优点：只有一个卷积核，所//以器计算成本比其他算子要低。

Laplace算子的缺点：它不像Sobel和Prewitt算子那样对图像进行了平滑处理，所以它会对噪声产生较大的响应，误将噪声当作边缘，并且得不到有方向的边缘。

![一些常用形式](https://img-blog.csdnimg.cn/20200626225440359.png)

Laplace核内的所有值的和必须为0。

### 8.7.2 代码详解

```c++
void laplacian(InputArray src, OutputArray dst, int ddepth, int borderType)
{
	Mat lapKernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 4, -1, 0, -1, 0);
	conv2D(src, lapKernel, dst, ddepth, Point(-1, -1), borderType);
}
```



### 8.7.3 处理结果

![image-20231218180732090](E:\typora\Project\OpenCV4.assets\image-20231218180732090.png)

## 8.8 高斯拉普拉斯（LoG）边缘检测

### 8.8.1 原理详解

​		在运用Lapalce核进行边缘检测的时候，首先要对图像进行高斯平滑处理，然后再与Laplace核进行卷积运算。这里使用了两次卷积运算。如果使用二维高斯函数，就可以只使用一次卷积运算。

​		因为这两个卷积核都是对图片二阶导数的近似估计，它们对于图片中的噪声均很敏感。因此，为了解决这一问题，我们一般会在进行拉普拉斯操作之前先对图像进行高斯平滑滤波处理。 ,先利用高斯平滑滤波进行处理，可以降低图片中的高频噪声，方便后续的拉普拉斯操作。事实上由于卷积操作具有结合律，因此我们先将高斯平滑滤波器与拉普拉斯滤波器进行卷积，然后利用得到的混合滤波器去对图片进行卷积以得到所需的结果。采用这个做法主要有以下两个优点：

- 由于高斯和拉普拉斯核通常都比图像小得多，所以这种方法通常只需要很少的算术运算。
- LoG (' Laplacian of Gaussian')内核的参数可以预先计算，因此在运行时只需要对图像执行一遍的卷积即可。

### 8.8.2 代码详解

```c++
// LoG 卷积
Mat LoG(InputArray image, float sigma, int win)
{
	Mat kernelX, kernelY;
	//得到两个分离核
	getSepLoGKernel(sigma, win, kernelX, kernelY);
	//先水平卷积再垂直卷积
	Mat covXY;
	sepConv2D_X_Y(image, covXY, CV_32FC1, kernelX, kernelY,Point(-1,-1),BORDER_DEFAULT);
	//卷积核转置
	Mat kernelX_T = kernelX.t();
	Mat kernelY_T = kernelY.t();
	//先垂直卷积再水平卷积
	Mat covYX;
	sepConv2D_Y_X(image, covYX, CV_32FC1, kernelX_T, kernelY_T,Point(-1,-1), BORDER_DEFAULT);
	//计算两个卷积结果的和，得到 LoG 卷积
	Mat LoGCov;
	add(covXY, covYX, LoGCov);
	return LoGCov;
}
```

### 8.8.3 运行结果

![image-20231218192252983](E:\typora\Project\OpenCV4.assets\image-20231218192252983.png)

## 8.9 高斯差分（DoG)边缘检测

### 8.9.1 高斯拉普拉斯与高斯差分的关系



# 九、几何形状的检测和拟合

通过阈值分割可以提取图像中的目标物体，通过边缘检测可以提取目标物体的轮廓，使用这两种方法可以基本确定物体的边缘或者前景。接下来是要拟合这些边缘和前景，如确定边缘是否满足某种几何形状。

## 9.1 点集的最小外包

​		点集是指坐标点的集。已经笛卡尔坐标系中有很多坐标点，需要找到能够包围这些坐标点的最小外包四边形或者圆，他们的面积就是最小面积。

### 9.1.1 最小外包旋转矩形

OpenCV提供了旋转矩形的类：

```c++
RotatedRect minAreaRect(InputArray points);
```

points：接收三种点集形式

 第一种：N×2的Mat类型，每一行代表一个点的坐标且数据类型只能是 CV_32S 或者 CV_32F；

 第二种：vector<Point>或者vector<Point2f>，即多个点组成的向量；

 第三种：N×1的双通道Mat类型

返回：输入点集points的最小外包旋转矩形（角度 中心 尺寸）

```C
//第一种
Mat points = (Mat_<double>(5, 2) << 1, 1, 5, 1, 1, 10, 5, 10, 2, 5);
//第二种
vector<Point2f> points;
points.push_back(Point2f(1, 1));
points.push_back(Point2f(5, 1));
points.push_back(Point2f(1, 10));
points.push_back(Point2f(5, 10));
points.push_back(Point2f(2, 5));
//第三种
Mat points = (Mat_<Vec2f>(5, 1) << Vec2f(1, 1), Vec2f(5, 1), Vec2f(1, 10), Vec2f(5, 10), Vec2f(2,5));
//计算点集的最小外包旋转矩形
RotatedRect rRect = minAreaRect(points);
Rect rect = boundingRect(points);
//打印旋转矩形的信息:
cout << "旋转矩形的角度:" << rRect.angle << endl;
cout << "旋转矩形的中心:" << rRect.center << endl;
cout << "旋转矩形的尺寸:" << rRect.size << endl;
cout << rect << endl;
```

​																输出结果为：

![image-20231220101454298](E:\typora\Project\OpenCV4.assets\image-20231220101454298.png)



### 9.1.2 旋转矩阵的4个顶点

`RotatedRect`提供的返回值能够间接计算出旋转矩阵的四个顶点，接下来介绍一个可以直接获取四个顶点的函数。

```c
void boxPoints(RotatedRect box,OutputArray points)
```

- box:构造的RotatedRect类矩形
- points 存放四个顶点的Points集

```c++
RotatedRect rRect(Point2f(200, 200), Point2f(90, 150), -60);

	Mat vertices;
	boxPoints(rRect, vertices);

	cout << "顶点坐标：" << endl;
	for (int i = 0; i < vertices.rows; i++)
	{
		Point2f point(vertices.at<float>(i, 0), vertices.at<float>(i, 1));
		cout << point << endl;
	}

	Mat img = Mat::zeros(Size(400, 400), CV_8UC1);
	for (int i = 0; i < 4; i++)
	{
		Point2f p1(vertices.at<float>(i, 0), vertices.at<float>(i, 1));
		int j = (i + 1) % 4;
		Point2f p2(vertices.at<float>(j, 0), vertices.at<float>(j, 1));
		line(img, p1, p2, Scalar(255), 3);
	}

	imshow("旋转矩形", img);
```

输出结果为：

<img src="E:\typora\Project\OpenCV4.assets\image-20231220104557767.png" alt="image-20231220104557767" style="zoom:50%;" />

### 9.1.3 最小外包圆

OpenCV为实现最小外包圆，提供了以下函数：

```c
void cv::minEnclosingCircle(cv::InputArray points, cv::Point2f &center, float &radius)

```

- points：点集，Mat 或者 vector 类型，和 minAreaRect 一样
- center：最小外包圆的圆心（输出）
- radius：最小外包圆的半径（输出）

```c
//点集
Mat points = (Mat_<float>(5, 2) << 1, 1, 5, 1, 1, 10, 5, 10, 2, 5);
//计算点集的最小外包圆
Point2f center;	//圆心
float radius;	//半径
minEnclosingCircle(points, center, radius);
//打印最小外包圆的信息:
cout << "圆心:" << center << endl;	//[3,5.5]
cout << "半径:" << radius << endl;	//5.07216
```

输出结果

![image-20231220105938408](E:\typora\Project\OpenCV4.assets\image-20231220105938408.png)

### 9.1.4 最小外包直立矩形

### 9.1.5 最小凸包

给定二维平面上的点集，凸包就是将最外层的点连接起来构成的凸多边形，他能包含点集中的所有点

<img src="https://img-blog.csdnimg.cn/img_convert/34565da8517e5720655ef7309e1a124b.png" alt="image-20211020213356743" style="zoom: 50%;" />



OpenCV为求点集的最小凸包，定义了如下函数：

```c++
void cv::convexHull(cv::InputArray points, cv::OutputArray hull, bool clockwise = false, bool returnPoints = true)
```

- points：输入点集是 vector 或者 Mat 类型

- hull：构成凸包的点，类型为vector<Point>、vector<Point2f>

- clockwise：hull 中的点是按顺时针还是逆时针排列的

- returnPoints：值为 true 时，hull 中存储的是坐标点；值为 false 时，存储的是这些坐标点在点集中的索引

```c
//5行2列的单通道Mat
Mat points = (Mat_<float>(5, 2) << 1, 1, 5, 1, 1, 10, 5, 10, 2, 5);
//求点集的凸包
vector<Point2f> hull;
convexHull(points, hull);
//打印得到最外侧的点（凸包）
for (int i = 0; i < hull.size(); i++)
{
	cout << hull[i] << ",";
}
```

### 9.1.6 最小外包三角形

```c
double cv::minEnclosingTriangle(InputArray points, CV_OUT OutputArray triangle)
```

- points 只支持两种点集形式：𝑁 × 1 的双通道 Mat 或者vector<Point>、vector<Point2f>，不支持 𝑁 × 2 的单通道 Mat

- triangle：是计算出的三角形的三个顶点，存储在 vector 向量中

- 返回的 double 值是最小外包三角形的面积。

```c++
//5行2列的单通道Mat
Mat points = (Mat_<int>(5, 2) << 1, 1, 5, 1, 1, 10, 5, 10, 2, 5);
//转换为5行1列的双通道Mat
points = points.reshape(2, 5);
//存储三角形的三个顶点
vector<Point> triangle;
//点集的最小外包三角形
double area = minEnclosingTriangle(points, triangle);
cout << "三角形的三个顶点:";
for (int i = 0; i < 3; i++)
{
	cout << triangle[i] << ",";
}
cout << "最小外包三角形的面积：" << area << endl;
//三角形的三个顶点: [9,1],[1,1],[1,19],
//最小外包三角形的面积：72
```



## 9.2 霍夫直线检测

### 9.2.1 原理详解

​		直线方程可由以下方式表示
$$
ρ=xcosθ+ysinθ
$$
​		如果知道平面内的一条直线，那么可计算出唯一的ρ 和θ 。换句话讲，xoy平面内的任意一条直线对应参数空间（也称霍夫空间）θoρ中的一点( ρ , θ ) 。

​		ρ是原点到直线的代数距离，θ是ON与x 轴的正方向（一二象限）或负方向（三四象限）的夹角。
​							<img src="https://img-blog.csdnimg.cn/img_convert/fa599de090d8dac24b3b48c901e4704f.png" alt="image-20211021190841904" style="zoom: 50%;" />

​		如果已知xoy平面内的一个点 ( x_0 , y_0) (x_0,y_0)(x_0 ,y_0)，则对应霍夫空间中的一条曲线ρ = x_0 cos_⁡θ + y_0 sin ⁡θ + y_0 =x_0cosθ+y0sinθ。据此，在xoy平面内的多个点在霍夫空间内对应多条曲线。如果这几条曲线相交于同一点，那么这几个点在xoy 平面内就是共线的。

​		**这就是霍夫线变换要做的：**它追踪图像中每个点对应曲线间的交点. 如果交于一点的曲线的数量超过了阈值, 那么可以认为这个交点所代表的参数表示的直线，在原图像中为一条直线。

### 9.2.2 代码详解

​		OpenCV提供的霍夫直线检测函数如下：

```c
void cv::HoughLines(cv::InputArray image, cv::OutputArray lines, double rho, double theta, int threshold, double srn = (0.0), double stn = (0.0), double min_theta = (0.0), double max_theta = (3.141592653589793116))
```

- image：输入边缘图片，必须是单通道的Mat格式的二值图像

- lines：vector<Vec2f>格式，包含ρ \rhoρ和θ \thetaθ信息，不包含直线上的点

- rho：以像素为单位的距离 r 的精度。一般情况下，使用的精度是 1

- theta：角度 θ 的精度。一般情况下，使用的精度是CV_PI / 180，表示要搜索所有可能的角度

- threshold：阈值。阈值越大 -> 检测越精准 + 速度越快 + 直线越少

标准的霍夫直线检测内存消耗比较大，执行时间比较长，基于这一点，提出了概率霍夫直线检测，它随机地从边缘二值图中选择前景像素点，确定检测直线的两个参数，其本质上还是标准的霍夫直线检测。

```c++
void cv::HoughLinesP(InputArray 	image,
                    OutputArray 	lines,
                    double 			rho,
                    double 			theta,
                    int 			threshold,
                    double 			minLineLength = 0,
                    double 			maxLineGap = 0 
                    )		
```

- image：输入边缘图片，必须是单通道的Mat格式的二值图像。
- lines：vector<Vec2f>格式，包含ρ \rhoρ和θ \thetaθ信息，不包含直线上的点。
- rho：以像素为单位的距离 r 的精度。一般情况下，使用的精度是 1。
- theta：角度 θ 的精度。一般情况下，使用的精度是CV_PI / 180，表示要搜索所有可能的角度。
- threshold：阈值。阈值越大 -> 检测越精准 + 速度越快 + 直线越少。
- minLineLength：最小线长。 短于此长度的将被拒绝。
- maxLineGap：连接同一条线上的点之间的最大允许间隙。

```c++
void on_HoughLines(int, void*)
{
    // 0、生成灰度图
    cvtColor(g_midImage, g_srcGrayImage, COLOR_GRAY2BGR);//转化边缘检测后的图为灰度图
    // 1、定义一个矢量结构lines用于存放得到的线段矢量集合
    // 2、进行霍夫线变换
    // 3、依次在图中绘制出每条线段
    vector<Vec4i> lines;
    HoughLinesP(g_midImage, lines, 1, CV_PI / 180, g_HoughLinesThreshold, g_minLineLength, 10);
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(g_srcGrayImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
    }
    //显示效果图
    imshow("HoughLines", g_srcGrayImage);
}
```

```c++
 // 1、载入原图
	g_srcImage = imread("F:\\Visuo studio\\Project\\Opencv\\Photo\\3.png");
	if (!g_srcImage.data) { printf("Oh，no，读取srcImage错误~！ \n"); return false; }

	//显示原始图
	imshow("【原始图】", g_srcImage);

	// 2、进行边缘检测和转化为灰度图
	Canny(g_srcImage, g_midImage, 50, 200, 3);//进行一此canny边缘检测
	imshow("【边缘检测后的图】", g_midImage);

	// 3、创建trackbar
	namedWindow("HoughLines", WINDOW_AUTOSIZE);
	createTrackbar("累加阈值：", "HoughLines", &g_HoughLinesThreshold, 300, on_HoughLines);
	createTrackbar("最小直线距离：", "HoughLines", &g_minLineLength, 200, on_HoughLines);

	// 4、调用回调函数
	on_HoughLines(0, 0);

	//轮询获取按键信息，若按下Q，程序退出
	while ((char(waitKey(1)) != 'q')) {}

	waitKey(0);
	return 0;
```



### 9.2.3 输出结果



![image-20231220132334373](E:\typora\Project\OpenCV4.assets\image-20231220132334373.png)

随着累加阈值的减少，被识别为直线的线段增加。

<center class="half">    <img src="E:\typora\Project\OpenCV4.assets\image-20231220134028892.png" width="400"/>    <img src="E:\typora\Project\OpenCV4.assets\image-20231220134122784.png" width="400"/> </center>		



## 9.3 霍夫圆检测

### 9.3.1 原理详解

对于半径已知的情况，可以通过画圆求交点的方式寻找圆心，但对于半径未知的情况，需要引入新的变量

![img](https://img-blog.csdnimg.cn/20210225231811918.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L20wXzM4MDA3Njk1,size_16,color_FFFFFF,t_70#pic_center)

我们引入三维的变量r,对应任何一个点，对应到abr空间中的锥面
$$
r^2=(a-x_i)^2+(b-y_i)^2
$$
如果多个锥面相交于一点（a1,b1,r1),则对应在xoy平面内的是共圆的，且圆心为(a1,b1)，半径为r1。但这样的程序运行效率很低，速度会很慢。一般通过“霍夫梯度法”的方法来解决这个问题。

**霍夫梯度法:**

1. 首先对图像应用边缘检测，比如用canny边缘检测。
2. 然后，对边缘图像中的每一个非零点，考虑其局部梯度，即用Sobel（）函数计算x和y方向的Sobel一阶导数得到梯度。
3. 利用得到的梯度，由斜率指定的直线上的每一个点都在累加器中被累加，这里的斜率是从一个指定的最小值到指定的最大值的距离。
4. 同时，标记边缘图像中每一个非0像素的位置。
5. 然后从二维累加器中这些点中选择候选的中心，这些中心都大于给定阈值并且大于其所有近邻。这些候选的中心按照累加值降序排列，以便于最支持像素的中心首先出现。
6. 接下来对每一个中心，考虑所有的非0像素。
7. 这些像素按照其与中心的距离排序。从到最大半径的最小距离算起，选择非0像素最支持的一条半径。8.如果一个中心收到边缘图像非0像素最充分的支持，并且到前期被选择的中心有足够的距离，那么它就会被保留下来。

### 9.3.2 代码详解

```c++
C++: void HoughCircles(
InputArray image,				// 即源图像，需为8位的灰度单通道图像。
OutputArray circles, 			// 存储了检测到的圆的输出矢量，每个矢量由包含了3个元素的浮点矢量(x, y, radius)表示
int method, 					// 即使用的检测方法，目前OpenCV中就霍夫梯度法一种可以使用，它的标识符为CV_HOUGH_GRADIENT
double dp, 						// 用来检测圆心的累加器图像的分辨率于输入图像之比的倒数，且此参数允许创建一个比输入图像分辨率低的累加器。
double minDist, 				// 为霍夫变换检测到的圆的圆心之间的最小距离，即让我们的算法能明显区分的两个不同圆之间的最小距离。
double param1=100,				// 它是第三个参数method设置的检测方法的对应的参数。对当前唯一的方法霍夫梯度法CV_HOUGH_GRADIENT，它表示传递给canny边缘检测算子的高阈值，而低阈值为高阈值的一半。
double param2=100, 				// 它是第三个参数method设置的检测方法的对应的参数。对当前唯一的方法霍夫梯度法CV_HOUGH_GRADIENT，它表示在检测阶段圆心的累加器阈值。它越小的话，就可以检测到更多根本不存在的圆，而它越大的话，能通过检测的圆就更加接近完美的圆形了。
int minRadius=0, 				// 表示圆半径的最小值
int maxRadius=0 				// 表示圆半径的最大值
)

```

```c++
    // 1、载入原始图、Mat变量定义
    Mat srcImage = imread("F:/C++/2. OPENCV 3.1.0/7.2 Hough 霍夫变换/2.jpg");  //工程目录下应该有一张名为1.jpg的素材图
    Mat midImage,dstImage;//临时变量和目标图的定义

    // 2、显示原始图
    imshow("【原始图】", srcImage);

    // 3、转为灰度图并进行图像平滑
    cvtColor(srcImage,midImage, COLOR_BGR2GRAY);//转化边缘检测后的图为灰度图
    GaussianBlur( midImage, midImage, Size(9, 9), 2, 2 );

    // 4、进行霍夫圆变换
    vector<Vec3f> circles;
    //HoughCircles( midImage, circles, HOUGH_GRADIENT,1.5, 10, 200, 100, 0, 0 );
    HoughCircles( midImage, circles, HOUGH_GRADIENT,1.5, 5, 200, 50, 0, 0 );
    // 5、依次在图中绘制出圆
    for( size_t i = 0; i < circles.size(); i++ )
    {
        //参数定义
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //绘制圆心
        circle( srcImage, center, 3, Scalar(0,255,0), -1, 8, 0 ); // 填满
        //绘制圆轮廓
        circle( srcImage, center, radius, Scalar(155,50,255), 2, 8, 0 );
    }

    //6、显示效果图
    imshow("【效果图】", srcImage);

```

### 9.3.3 输出结果

param2=100时

![image-20231220145348905](E:\typora\Project\OpenCV4.assets\image-20231220145348905.png)

param2=50时，产生了很多不存在的圆

![image-20231220145532278](E:\typora\Project\OpenCV4.assets\image-20231220145532278.png)

param2=200时，产生的圆质量很好

![image-20231220145645912](E:\typora\Project\OpenCV4.assets\image-20231220145645912.png)





## 9.4 轮廓

### 9.4.1 查找、绘制轮廓

之前的点集是手动输入或者随机生成的，OpenCV提供了一个可以返回一个有序的点集的方法

```c++
void findContours(InputOutputArray image, OutputArrayOfArrays contours, OutputArray hierarchy, int mode, int method, Point offset=Point());	
```

- `image`   要绘制轮廓的图像,8位单通道的图像（256级灰度图）
- `contours`  所有输入的轮廓，找到的轮廓，其中每个轮廓会被存储为vector<Point>
- `hierachy`  层次结构，可选的输出向量，包含关于图像的拓扑结构信息,其具有跟轮廓数相同的元素个数,类型为vector<Vec4i>，后一个轮廓、前一个轮廓、第一个子轮廓、父轮廓的索引编号，如果没有对应项，该值设置为-1
- `mode`  检索轮廓的模式分别表示
- `method`  为轮廓的近似办法
- `offset`  代表轮廓点的偏移量，可以设置为任意值

**mode:**

| RETR_EXTERNAL 表示只检测外轮廓                               |
| :----------------------------------------------------------- |
| RETR_LIST 检测的轮廓不建立等级关系                           |
| RETR_CCOMP 建立两个等级的轮廓，上面的一层为外边界，里面的一层为内孔的边 界信息。  如果内孔内还有一个连通物体，这个物体的边界也在顶层。 |
| RETR_TREE 建立一个等级树结构的轮廓。具体参考contours.c这个demo |

**method:**

| CHAIN_APPROX_NONE 存储所有的轮廓点，相邻的两个点的像素位置差不超过1，即max（abs（x1-x2），abs（y2-y1））==1 |
| ------------------------------------------------------------ |
| CHAIN_APPROX_SIMPLE 压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标，例如一个矩形轮廓只需4个点来保存轮廓信息 |
| CHAIN_APPROX_TC89_L1，CV_CHAIN_APPROX_TC89_KCOS 使用teh-Chinl chain 近似算法 |

为了直观地理解所找到的轮廓，可以通过函数：

```c
void drawContours(
InputOutputArray image, 		// 要绘制轮廓的图像
InputArrayOfArrays contours, 	// 所有输入的轮廓，每个轮廓被保存成一个point向量(vector<vector<Point>>)
int contourIdx, 				// 指定要绘制轮廓的编号，如果是负数，则绘制所有的轮廓
const Scalar& color, 			// 绘制轮廓所用的颜色
int thickness=1, 				// 绘制轮廓的线的粗细，如果是负数，则轮廓内部被填充(CV_FILLED 填充内部)
int lineType=8, 				// 绘制轮廓的线的连通性(LINE_AA 抗锯齿线形)
InputArray hierarchy=noArray(), 	// 关于层级的可选参数，只有绘制部分轮廓时才会用到（hierarchy=hierarchy，绘制所有轮廓）
int maxLevel=INT_MAX, 			// 绘制轮廓的最高级别，这个参数只有hierarchy有效的时候才有效
Point offset=Point() 			// 代表轮廓点的偏移量，可以设置为任意值
)

```

- `image`	要绘制轮廓的图像
- `contours`   所有输入的轮廓，每个轮廓被保存成一个point向量(vector<vector<Point>>)
- `contourIdx`  指定要绘制轮廓的编号，如果是负数，则绘制所有的轮廓
- `color ` 绘制轮廓所用的颜色
- `thickness ` 绘制轮廓的线的粗细，如果是负数，则轮廓内部被填充(CV_FILLED 填充内部)
- `lineType`  绘制轮廓的线的连通性(LINE_AA 抗锯齿线形)
- `hierarchy`  关于层级的可选参数，只有绘制部分轮廓时才会用到
- `maxLevel`  绘制轮廓的最高级别，这个参数只有hierarchy有效的时候才有效
- `offset`  代表轮廓点的偏移量，可以设置为任意值

**maxLevel**

| maxLevel=0，绘制与输入轮廓属于同一等级的所有轮廓即输入轮廓和与其相邻的轮廓 |
| ------------------------------------------------------------ |
| maxLevel=1, 绘制与输入轮廓同一等级的所有轮廓与其子节点。     |
| maxLevel=2，绘制与输入轮廓同一等级的所有轮廓与其子节点以及子节点的子节点 |

```c
//第一步：输入图像
	//第二步：边缘检测，得到边缘二值图
	GaussianBlur(img, img, Size(3, 3), 0.5);
	Mat binaryImg;
	Canny(img, binaryImg, 50, 200);
	imshow("显示边缘", binaryImg);
	imwrite("cannyEdge.jpg", binaryImg);
	//第三步：边缘的轮廓
	vector<vector<Point>> contours;
	vector<Vec4i> hei;
	findContours(binaryImg, contours, hei, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	
	//第四步：对每一个轮廓作拟合，这里用旋转矩形
	int num = contours.size();//轮廓的数量
	for (int i = 0; i < num; i++)
	{
		//最小外包直立矩形
		Rect rect = boundingRect(Mat(contours[i]));
		if (rect.area() > 2000)//筛选出面积大于 10000 的矩形
		{
			//在原图中画出外包矩形
			rectangle(img, rect, Scalar(255), 2);
			cout << rect.area() << endl;
		}
	}
	imshow("img", img);
	imwrite("img0.jpg", img);
	waitKey(0);
	return 0;
```

![image-20231220152203094](E:\typora\Project\OpenCV4.assets\image-20231220152203094.png)

### 9.4.2 外包、拟合轮廓

**approxPolyDP()函数:**

​		以指定的精度近似生成多边形曲线。函数逼近一条曲线或另一条曲线/顶点较少的多边形，使它们之间的距离小于或等于指定的精度。它使用Douglas-Peucker算法

```c
void approxPolyDP(InputArray curve,OutputArray approxCurve, double epsilon,bool closed);
```

`curve`：输入的点集（存储在std::vector或Mat中的二维点的输入向量）

`approxCurve`：输出的点集，当前点集是能最小包容指定点集的。draw出来即是一个多边形

`epsilon`：指定的精度，也即是原始曲线与近似曲线之间的最大距离。

`closed`：若为true,则说明近似曲线是闭合的，它的首位都是相连，反之，若为false，则断开。

```c++
void Approx(Mat img,string outdir)
{
    Mat gaussImg;
    Mat binaryImg;
    GaussianBlur(img, gaussImg, Size(3, 3), 0.5);
    threshold(gaussImg, binaryImg, 0, 255, THRESH_OTSU);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    //形态学开运算（消除细小白点）
    morphologyEx(binaryImg, binaryImg, MORPH_OPEN, kernel);
    imwrite(outdir + "canny.jpg", binaryImg);
    // 边缘的轮廓
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat contourImg = Mat::zeros(img.rows, img.cols, CV_8UC1);
    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(contourImg, contours, i, Scalar(255), 2);
        // 画出轮廓的最小外包圆
//        Point2f center;
//        float radius;
//        minEnclosingCircle(contours[i], center, radius);
//        circle(img, center, radius, Scalar(255), 2);
        // 多边形逼近
        vector<Point> approxCurve;
        approxPolyDP(contours[i], approxCurve, 0.3, true);
        for (int i = 0; i < approxCurve.size() - 2; i++)
        {
            line(img, approxCurve[i], approxCurve[i + 1], Scalar(0), 2);
        }
        line(img, approxCurve[approxCurve.size() - 1], approxCurve[0], Scalar(0), 2);
    }
    imshow("图像.jpg", img);
    imshow("图像轮廓.jpg", contourImg);
}
```

输出结果：

![image-20231220155247005](E:\typora\Project\OpenCV4.assets\image-20231220155247005.png)



### 9.4.3 轮廓的周长和面积

常用函数详解

**计算点集所围的区域的周长和面积。**

```c++
double arcLength(InputArray curve, bool closed)
```

- InputArray类型的curve，输入的向量，二维点（轮廓顶点），可以为std::vector或Mat类型。
- bool类型的closed，用于指示曲线是否封闭的标识符，一般设置为true。

**计算点集所围成的区域的面积。**

```c
//函数原型1
double ContourArea(InputArray contour,
    bool oriented = false)
 
//函数原型2
double ContourArea(IEnumerable<Point> contour,
    bool oriented = false)
 
//函数原型3
double ContourArea(IEnumerable<Point2f> contour,
    bool oriented = false)
```

![image-20231224125108700](E:\typora\Project\OpenCV4.assets\image-20231224125108700.png)

![image-20231224125657987](E:\typora\Project\OpenCV4.assets\image-20231224125657987.png)

### 9.4.4 点和轮廓的位置关系

空间中任意一点与这个轮廓无非有三种关系：点在轮廓外、点在轮廓上、点在轮廓外。

OpenCV提供的函数：

```c++
double pointPolygonTest(InputArray contour,Point2f pt, bool measureDist)
```

- contour 为轮廓。

- pt 为待判定的点。

- measureDist 为布尔型值，表示距离的判定方式。

  ​	当值为 True 时，表示计算点到轮廓的距离。如果点在轮廓的外部，返回值为负数；如果点在轮廓上，返回值为 0；如果点在轮廓内部，返回值为正数。

  ​	当值为 False 时，不计算距离，只返回“-1”、“0”和“1”中的一个值，表示点相对于轮廓的位置关系。如果点在轮廓的外部，返回值为“-1”；如果点在轮廓上，返回值为“0”；如果点在轮廓内部，返回值为“1”。



![image-20231224215409334](E:\typora\Project\OpenCV4.assets\image-20231224215409334.png)

![image-20231224215423398](E:\typora\Project\OpenCV4.assets\image-20231224215423398.png)



### 9.4.5 轮廓的凸包缺陷

用来衡量凸包的缺陷，OpenCV提供了如下函数

```c++
void cv::convexityDefects(InputArray contour,InputArray convexhull,OutputArray convexityDefects);
```

- contour：输入的轮廓点信息，可以通过cv::findContours()函数获取。
- convexhull：输入的凸包点信息，可以通过cv::convexHull()函数获取。
- convexityDefects：输出的凸缺陷信息，是一个N行4列的矩阵，每行包含4个元素，分别是凸缺陷起点索引、终点索引、最远点索引和距离

函数使用步骤如下

1.读取图像并转换为灰度图像。

```c++
Mat src = imread("image.jpg");
Mat gray;
cvtColor(src, gray, CV_BGR2GRAY);
```

2.对灰度图像进行二值化处理。

```c++
Mat binary;threshold(gray, binary, 100, 255, THRESH_BINARY);
```

3.获取图像中的轮廓信息。

```c++
vector<vector<Point>> contours;
findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
```

4.对每个轮廓计算凸包点信息。

```c
vector<vector<int>> hull(contours.size());
for (int i = 0; i < contours.size(); i++)
 {   
  convexHull(contours[i], hull[i], false);
 }
```

5.对每个轮廓计算凸缺陷信息

```c++
vector<vector<Vec4i>> defects(contours.size());for (int i = 0; i < contours.size(); i++) { 
   if (hull[i].size() > 3) {  
         convexityDefects(contours[i], hull[i], defects[i]);  
           }
        }
```

6.遍历所有轮廓的凸缺陷信息，并绘制凸缺陷。

```c++
for (int i = 0; i < contours.size(); i++) {    
	for (int j = 0; j < defects[i].size(); j++) {        
	Vec4i& v = defects[i][j];        
	float depth = v[3] / 256.0;        
	if (depth > 10) {            
	int startidx = v[0];            
	Point start(contours[i][startidx]);            
	int endidx = v[1];            
	Point end(contours[i][endidx]);            
	int faridx = v[2];            
	Point far(contours[i][faridx]);            
	line(src, start, end, Scalar(0, 0, 255), 2);            
	line(src, start, far, Scalar(0, 255, 0), 2);            
	line(src, end, far, Scalar(0, 255, 0), 2);            
	circle(src, far, 4, Scalar(0, 255, 0), -1);  
		}    
	}
}
```









# 十、傅里叶变换

傅里叶变换是线性系统分析的一个有力工具，它使我们能够定量分析数字化系统。

## 10.1二维离散的傅里叶(逆)变换



​		OpenCV提供了如下函数来实现矩阵的傅里叶（逆)变换

```c
void cv::dft(cv::InputArray src, cv::OutputArray dst, int flags = 0, int nonzeroRows = 0)
```

src：输入矩阵，只支持 CV_32F 或者 CV_64F 的单通道或双通道矩阵

dst：输出矩阵

flags：用于说明是傅里叶变换还是傅里叶逆变换

- DFT_COMPLEX_OUTPUT：输出复数形式
- DFT_REAL_OUTPUT：只输出实部
- DFT_INVERSE：傅里叶逆变换
- DFT_SCALE：是否除以 𝑀*𝑁
- DFT_ROWS：输入矩阵的每行进行傅里叶变换或者逆变换



​		在 OpenCV 中实现的傅里叶变换的快速算法是针对行数和列数均满足可以分解为 2𝑝 × 3𝑞 × 5𝑟 的情况的。所以计算二维矩阵的快速傅里叶变换时需要先对原矩阵进行扩充，在矩阵的右侧和下侧补 0，以满足该规则。对于补多少行多少列的 0，可以使用函数：

```c
int cv::getOptimalDFTSize(int vecsize)
```



## 10.2 傅里叶幅度谱与相位谱

下面对幅度谱和相位谱进行灰度级显示：

​		OpenCV中直接计算两个矩阵对应位置平方和的平方根：

```c
void cv::magnitude(cv::InputArray x, cv::InputArray y, cv::OutputArray magnitude)
```

- x：浮点型矩阵
- y：浮点型矩阵
- magnitude：幅度谱

​		因为对图像进行傅里叶变换后得到的是一个复数矩阵，保存在一个双通道 Mat 类中，所以在使用函数 `magnitude` 计算幅度谱时，需要利用 OpenCV 提供的函数 `split` 将傅里叶变换的实部和虚部分开。具体实现代码如下：

```c
void amplitudeSpectrum(InputArray _srcFFT, OutputArray _dstSpectrum)
{
    //判断傅里叶变换有两个通道
    CV_Assert(_srcFFT.channels() == 2);
    //分离通道
    vector<Mat> FFT2Channel;
    split(_srcFFT, FFT2Channel);
    //计算傅里叶变换的幅度谱 sqrt(pow(R,2)+pow(I,2))
    magnitude(FFT2Channel[0], FFT2Channel[1], _dstSpectrum);
}
```

​		对于傅里叶谱的灰度级显示，OpenCV 提供了函数 `log`，该函数可以计算矩阵中每一个值的对数。进行归一化后，为了保存傅里叶谱的灰度级，有时需要将矩阵乘以 255，然后转换为 8 位图。具体代码如下：

```c++
Mat graySpectrum(Mat spectrum)
{
    Mat dst;
    log(spectrum + 1, dst);
    //归一化
    normalize(dst, dst, 0, 1, NORM_MINMAX);
    //为了进行灰度级显示，做类型转换
    dst.convertTo(dst, CV_8UC1, 255, 0);
    return dst;
}
```

OpenCV提供的计算相位谱的函数：

```c++
void cv::phase(cv::InputArray x, cv::InputArray y, cv::OutputArray angle, bool angleInDegrees = false)
```

- x：输入矩阵（傅里叶变换的实部矩阵）
- y：输入矩阵（傅里叶变换的虚部矩阵）
- angle：输出矩阵
- angleInDegrees：是否将角度转换到 [-180, 180]



整体运行函数

```c
void Spectrum(Mat img)
{
	//将图像转换为浮点型
	Mat fImg;
	img.convertTo(fImg, CV_64FC1, 1.0, 0);
	//乘以 -1^(r+c)
	int rows = fImg.rows;
	int cols = fImg.cols;
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			if ((r + c) % 2)
				fImg.at<double>(r, c) *= -1;
		}
	}
	//快速傅里叶变换
	Mat fft2;
	fft2Image(fImg, fft2);
	//傅里叶变换的幅度谱
	Mat ampSpec;
	amplitudeSpectrum(fft2, ampSpec);
	//幅度谱的灰度级显示
	Mat graySpec = graySpectrum(ampSpec);
	imshow("幅度谱的灰度级显示", graySpec);
	//相位谱
	Mat phaSpec = phaseSpectrum(fft2);
	//相位谱的灰度级显示
	imshow("相位谱", phaSpec);
}

```

输出结果：

![image-20231225124210715](E:\typora\Project\OpenCV4.assets\image-20231225124210715.png)

## 10.3 谱残差显著性检测

生物视觉研究表明，视觉注意机制是一种具有选择性的注意，它首先由视觉内容中最显著的、与其周围其他内容相比差异更大的成分引起，然后根据主观意识去选择注意。

视觉显著性检测可以看作抽取信息中最具差异化的部分或者最感兴趣的或首先关注的部分，赋予对图像分析的选择性能力，对提高图像的处理效率是极为重要的。

### 10.3.1 原理详解

**第一步：**计算图像的快速傅里叶变换矩阵 F 。

**第二步：**计算傅里叶变换的幅度谱的灰度级 graySpectrum。

**第三步：**计算相位谱 phaseSpectrum，然后根据相位谱计算对应的正弦谱和余弦谱。

**第四步：**对第二步计算出的灰度级进行均值平滑，记为 𝑓mean(graySpectrum)。

**第五步：**计算谱残差（spectralResidual）。谱残差的定义是第二步得到的幅度谱的灰度级减去第四步得到的均值平滑结果，即：
$$
spectralResidual = graySpectrum − f_{mean} ( graySpectrum )
$$
**第六步：**对谱残差进行幂指数运算 exp(spectralResidual)，即对谱残差矩阵中的每一个值进行指数运算。

**第七步：**将第六步得到的幂指数作为新的“幅度谱”，仍然使用原图的相位谱，根据新的“幅度谱”和相位谱进行傅里叶逆变换，可得到一个复数矩阵。

**第八步：**对于第七步得到的复数矩阵，计算该矩阵的实部和虚部的平方和的开方，然后进行高斯平滑，最后进行灰度级的转换，即得到显著性。

### 10.3.2 代码实现

```c
void SaliencyMap(Mat image)
{
	//转换为 double 类型
	Mat fImage;
	image.convertTo(fImage, CV_64FC1, 1.0 / 255);
	//快速傅里叶变换
	Mat fft2;
	fft2Image(fImage, fft2);
	//幅度谱（又称傅里叶谱）
	Mat amplitude;
	amplitudeSpectrum(fft2, amplitude);
	//对幅度谱进行对数运算
	Mat logAmplitude;
	cv::log(amplitude + 1.0, logAmplitude);
	//均值平滑
	Mat meanLogAmplitude;
	cv::blur(logAmplitude, meanLogAmplitude, Size(3, 3), Point(-1, -1));
	//谱残差
	Mat spectralResidual = logAmplitude - meanLogAmplitude;
	//相位谱
	Mat phase = phaseSpectrum(fft2);
	//余弦谱 cos(phase)
	Mat cosSpectrum(phase.size(), CV_64FC1);
	//正弦谱 sin(phase)
	Mat sinSpectrum(phase.size(), CV_64FC1);
	for (int r = 0; r < phase.rows; r++)
	{
		for (int c = 0; c < phase.cols; c++)
		{
			cosSpectrum.at<double>(r, c) = cos(phase.at<double>(r, c));
			sinSpectrum.at<double>(r, c) = sin(phase.at<double>(r, c));
		}
	}
	//指数运算
	exp(spectralResidual, spectralResidual);
	Mat real = spectralResidual.mul(cosSpectrum);
	Mat imaginary = spectralResidual.mul(sinSpectrum);
	vector<Mat> realAndImag;
	realAndImag.push_back(real);
	realAndImag.push_back(imaginary);
	Mat complex;
	merge(realAndImag, complex);
	//快速傅里叶逆变换
	Mat ifft2;
	dft(complex, ifft2, DFT_COMPLEX_OUTPUT + DFT_INVERSE);
	//傅里叶逆变换的幅度
	Mat ifft2Amp;
	amplitudeSpectrum(ifft2, ifft2Amp);
	//平方运算
	pow(ifft2Amp, 2.0, ifft2Amp);
	//高斯平滑
	GaussianBlur(ifft2Amp, ifft2Amp, Size(11, 11), 2.5);
	//显著性显示
	normalize(ifft2Amp, ifft2Amp, 1.0, 0, NORM_MINMAX);
	//提升对比度，进行伽马变换
	pow(ifft2Amp, 0.5, ifft2Amp);
	//数据类型转换
	Mat saliencyMap;
	ifft2Amp.convertTo(saliencyMap, CV_8UC1, 255);
	imshow("显著性", saliencyMap);
}
```

### 10.3.3 输出结果

![image-20231225131853774](E:\typora\Project\OpenCV4.assets\image-20231225131853774.png)



# 十一、频率域滤波

## 11.1 概述及原理详解

低频：图像的傅里叶变换“中心位置”附近的区域。

高频：随着到中心位置的距离的增加而增加，即傅里叶变换中心位置的外围区域。

频率域滤波器在程序或者数学运算中可以理解为一个矩阵，下面所涉及的常用的低通、高通、带通、带阻等滤波的关键步骤，就是通过一定的准则构造该函数的。

![image-20231225155339389](E:\typora\Project\OpenCV4.assets\image-20231225155339389.png)



## 11.2 低通滤波和高通滤波

低频信息表示的是图像中灰度值缓慢变化的区域；高频信息表示图像中变化迅速的部分。

低通滤波保留低频信息，去除高频信息；高频滤波保留高频信息，去除低频信息。



### 11.2.1 三种常用的低通滤波器

（1）理想低通滤波器

![image-20231225160427688](E:\typora\Project\OpenCV4.assets\image-20231225160427688.png)

（2）巴特沃斯滤波器

![image-20231225160437085](E:\typora\Project\OpenCV4.assets\image-20231225160437085.png)

  (3)  高斯低通滤波器

![image-20231225160504994](E:\typora\Project\OpenCV4.assets\image-20231225160504994.png)

代码实现如下：

```c
Mat createLPFilter(Size size, Point center, float radius, int type, int n=2)
{
    Mat lpFilter = Mat::zeros(size, CV_32FC1);
    int rows = size.height;
    int cols = size.width;
    if (radius <= 0)
    return lpFilter;
    //构建理想低通滤波器
    if (type == ILP_FILTER)
    {
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                float norm2 = pow(abs(float(r - center.y)), 2) + pow(abs(float(c - center.x)), 2);
                if (sqrt(norm2) < radius)
                    lpFilter.at<float>(r, c) = 1;
                else
                    lpFilter.at<float>(r, c) = 0;
            } 
        } 
    }
    //构建巴特沃斯低通滤波器
    if (type == BLP_FILTER)
    {
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c<cols; c++)
            {
                lpFilter.at<float>(r, c) = float(1.0 / (1.0 + pow(sqrt(pow(r - center.y, 2.0) + pow(c - center.x, 2.0)) /radius, 2.0*n)));
            } 
        } 
    }
    //构建高斯低通滤波器
    if (type == GLP_FILTER)
    {
        for (int r = 0; r< rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                lpFilter.at<float>(r, c) = float(exp(-(pow(c - center.x, 2.0) + pow(r - center.y, 2.0)) / (2 * pow(radius, 2.0))));
            } 
        } 
    }
    return lpFilter;
}

```



### 11.2.2 三种常用的高通滤波器

（1）理想高通滤波器

![image-20231225165218667](E:\typora\Project\OpenCV4.assets\image-20231225165218667.png)

（2）巴特沃斯高通滤波器

![image-20231225165229448](E:\typora\Project\OpenCV4.assets\image-20231225165229448.png)

（3）高斯高通滤波器

![image-20231225165240001](E:\typora\Project\OpenCV4.assets\image-20231225165240001.png)

​		代码方面只需要将低通滤波的滤波部分改为高通滤波。



## 11.3 带通和带阻滤波

### 11.3.1 三种常用的带通滤波器

​		带通滤波是指只保留某一范围区域的频率带

（1）理想带通滤波器

![image-20231225170026381](E:\typora\Project\OpenCV4.assets\image-20231225170026381.png)

（2）巴特沃斯带通滤波器

<img src="E:\typora\Project\OpenCV4.assets\image-20231225165942037.png" alt="image-20231225165942037" style="zoom:120%;" />

（3)  高通带通滤波器

<img src="E:\typora\Project\OpenCV4.assets\image-20231225165952514.png" alt="image-20231225165952514" style="zoom:150%;" />

### 11.3.2 三种常用的带阻滤波器

（1）理想带阻滤波器

![image-20231225170748109](E:\typora\Project\OpenCV4.assets\image-20231225170748109.png)

（2）巴特沃斯带阻滤波器

![image-20231225170803773](E:\typora\Project\OpenCV4.assets\image-20231225170803773.png)

（3）高斯带阻滤波器

![image-20231225170819933](E:\typora\Project\OpenCV4.assets\image-20231225170819933.png)





# 十二、色彩空间

## 12.1 常见的色彩空间

灰度图像的每一个像素都是由一个数字量化成的，而彩色图像的每一个像素都是由三个数字量化的。比较常用的三色色彩空间包括RGB、HSV、HLS、Lab、YUV等。

### 12.1.1 RGB色彩空间

### 12.1.2 HSV色彩空间

### 12.1.3 HLS色彩空间



## 12.2 调整彩色图像的饱和度和亮度

因为在HLS和HSV色彩空间中都将饱和度和亮度单独分离出来，所以首先将RGB图像转化为HLS或者HSV图像，然后调整饱和度和亮度分量，最后将调整后的HLS或者HSV图像转化为RGB图像。





1.

![image-20231215145052107](E:\typora\Project\OpenCV4.assets\image-20231215145052107.png)

​		解决方法：检查con2D的申明和定义中是否重复赋给了某个参数初值。去掉赋的初值后，报错解除。



2.

![image-20231215164731139](E:\typora\Project\OpenCV4.assets\image-20231215164731139.png)

​		解决方法：在头文件中#include <opencv2/core/types.hpp>



3.

![img](https://img-blog.csdn.net/20170824100630314?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvc2luYXRfMzYyNjQ2NjY=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/Center)

​		解决方法：矩阵越位或者矩阵元素不够，检查一下矩阵的元素是否数量正确



4.

![image-20231217223013973](E:\typora\Project\OpenCV4.assets\image-20231217223013973.png)

​		解决方法：\#include <opencv2/imgproc/imgproc_c.h>



5.

![image-20231218001437655](E:\typora\Project\OpenCV4.assets\image-20231218001437655.png)

​		解决方法：main()打错了，很低级的错误。

6.

![image-20231218172820064](E:\typora\Project\OpenCV4.assets\image-20231218172820064.png)

​		解决方法：由于不同OpenCV版本不同，CV_THRESH_BINARY_INV 修改为cv::THRESH_BINARY_INV

