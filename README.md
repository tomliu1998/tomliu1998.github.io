# tomliu1998.github.io
基于独立四轮转向线控底盘与rtk差分定位技术的规划器
HNU_rtk_recorder: 1.1
				1.1_SW:加入模式切换信息录制
				1.2_SW:加入前后轴转角信息录制
				1.3_SW:将所有self.chassis的信息接收地址修改为self.chassis_detail


HNU_rtk_player:  1.1
			   1.1_SW：加入模式切换
			   1.2:更新横向偏差算法，采用面积积算法
			   1.3:加入智能驾驶进入逻辑程序
			   1.4:加入前后轴信息与曲率半径正负值判断算法;设置曲率拟合点数超参数；完善智能驾驶进入逻辑程序；给曲率计算公式添加分母绝对值；
			
				1.5:将planningdata调整至_init_构造函数中去，由局部变量变为成员变量;将智能驾驶模式进入三个函数的警告信息warning改为报错信息error
				
				1.6:
				• 删除多余的publishmsg()调用
				• 横向偏差函数中，直接将shortest_dist_sqr修改为一个具体的很大的数：100000.0
				• 将曲率计算函数中的曲率半径计算公式的分子分母分开，并对分母进行约束防止为0
				• 将所有self.chassis的信息接收地址修改为self.chassis_detail
			
				1.7:
				• 按照舍弗勒最新更新的自动驾驶模式进入逻辑，更新了自动驾驶模式进入程序
				
				1.8: 循迹打通之后的第一版程序
				
				1.2_SW:加入原地转向模式切换程序
				1.3_SW:更新了原地转向模式切换程序(while循环代替遍历轨迹图标思路，利用i_js_5实现对特殊转向模式标记点数量的计数)
				
				1.4_SW:
				• 第一次全新大改版规划器
				• 更新了航向角播放地址:
				self.planningdata.headingangle_deviation = self.data['theta'][i]
			
			
			
2.0版本规划器调试


				HNU_rtk_player_2.0_offical： 第一次调通的2.0规划器
				rtk_player_2.0_test6_3：更新了新速度规划器；将90度横移模式刹车释放速度判断改为绝对值
				HNU_rtk_player_2.1_offical: 第一次调通弯道循迹，极限蛇形循迹，以及原地转向模式的模式切换前段部分
				
				rtk_player_2.0_test11 ：曲率优化测试，均值平均技术路线
				rtk_player_2.0_test11_1：曲率优化测试，单次变化限幅技术路线
				HNU_rtk_player_2.2_offical：所有模式切换与长距离循迹全部成功版本
				HNU_rtk_player_2.3_offical.py：大环线实现的第一版
				
				
				
				
![image](https://user-images.githubusercontent.com/59682745/187025776-de57c90a-4740-47bc-9236-080d8e2ae3a7.png)
