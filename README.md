# 小车平衡跷跷板

**设计并制作一辆自动控制小车，实现小车在跷跷板上自主保持平衡。具体要求如下：**

 

1、 跷跷板用1m*1m的木工板制作，回转中心线在木板的中线位置；跷跷板平衡后，板面离地面高度不小于50mm；跷跷板无任何动力，其回转运动无明显阻力。

2、 小车先放置在跷跷板的一端，然后一键启动。

3、 小车的投影尺寸不超过300mm*300mm。

4、 保持10秒以上，跷跷板两端的高度差不超过2cm，认为平衡。

5、 小车完成任务的时间不超过50s。

 

**完成如下任务：**

1、 小车有声/光提示功能，可提示平衡状态。

2、 小车启动后，自主运动控制跷跷板达到平衡状态。

3、 小车启动后，自主运动控制跷跷板达到平衡状态，然后人为在跷跷板上，与小车运动方向一致的某个位置，放入一个盒子，盒内装有一枚500g钢球（钢球可在盒子中自由滚动，滚动范围不小于50mm），小车继续保持跷跷板平衡。整个过程不得重新启动小车。



## 硬件设计

### PCB设计

**原理图**

![Schematic_Car_seesaw.png](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/Gallery/2021/02/10/Schematic_Car_seesaw.png)

**PCB**

![image.png](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/Gallery/2021/02/10/image.png)

### 引脚分配

![image602b314e0c445b6c.png](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/Gallery/2021/02/10/image602b314e0c445b6c.png)

## 软件设计

### 传感器

#### DMP姿态解算驱动

如果选择了使用DMP姿态解算，那么前面的GY-86初始化实际上可以直接调用DMP固件库函数完成。

DMP姿态解算采用的MPU6050官方固件库，我们需要完成的工作就是将官方固件库移植到板子上。

1.预留I2C读写接口。

2.预留毫秒延迟接口。

3.预留节拍获取接口。

```C
#define i2c_write   MPU_Write_Len
#define i2c_read    MPU_Read_Len
#define delay_ms    HAL_Delay
#define get_ms      mget_ms
```

以上及完成了基本接口的对接。

下面利用DMP固件库完成模块的初始化：

```c
//mpu6050,dmp初始化
//返回值:0,正常
//    其他,失败
u8 mpu_dmp_init(void)
{
	u8 res=0;
	if(mpu_init()==0)	//初始化MPU6050
	{	 
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		if(res)return 1; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//设置FIFO
		if(res)return 2; 
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	//设置采样率
		if(res)return 3; 
		res=dmp_load_motion_driver_firmware();		//加载dmp固件
		if(res)return 4; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res)return 5; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//设置dmp功能
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
		if(res)return 7;   
		res=run_self_test();		//自检
		if(res)return 8;    
		res=mpu_set_dmp_state(1);	//使能DMP
		if(res)return 9;     
	}
	return 0;
}
```

### 前后台系统

后台程序：向上位机发送数据。

前台程序：串级PID控制。PID参数修改（未完善）。

利用TIM中断产生ms级软件时序控制任务频率。

### PID调节大致框图

![47d1d883cc452da6fcd8dfd96b3f22c1.png](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/Gallery/2021/02/10/47d1d883cc452da6fcd8dfd96b3f22c1.png)

图源：[【串级PID】浅谈串级PID作用及意义——快速理解串级PID结构优势（附图](https://blog.csdn.net/ReadAir/article/details/103030418)）

## 测试视频

