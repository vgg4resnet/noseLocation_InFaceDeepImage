# noseLocation_InFaceDeepImage
### 预处理  
zjraw文件夹中有深度图.raw文件,同名.txt文件位人脸坐标信息  
otherraw文件夹中只有深度图像.raw文件，未经过人脸位置信息标定，可以利用facePosition_Process.m进行标定，每次用鼠标框出人脸部分后，会自动生成包含人脸坐标信息的同名.txt文件，直到文件夹中所有.raw文件遍历完。  
### 运行  
修改深度图与坐标文本路径  
运行noseLocation_InFaceDeepImage.m后，显示脸部中心区域鼻翼边缘置信点  
![image](https://github.com/zj19941113/noseLocation_InFaceDeepImage/blob/master/img/1.jpg)  
显示人脸部分深度图与鼻子三角区域  
![image](https://github.com/zj19941113/noseLocation_InFaceDeepImage/blob/master/img/2.jpg)  
