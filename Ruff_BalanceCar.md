# Ruff平衡车


## 原理简介

两轮自动平衡车是一个典型的自动控制系统，由执行元件（直流电机），传感模块（陀螺仪和编码器）和主控平台控制系统（Ruff开发板）组成。直流电机控制两轮正反转，陀螺仪检测车身姿态，编码器检测电机转速，这两组传感器数据反馈给控制系统，经由PID控制算法计算，给出控制直流电机的控制量，通过这一闭环过程，从而形成负反馈，保证车身平衡

## 物件清单

### 主控平台

Ruff MCU开发版（tm4c1294-v1）

![ControlBlock](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/tm4c1294.jpg)

> 开发板型号为TI TM4C1294-LaunchPad

### 传感器及执行元件

1. 陀螺仪模块 （GY-521）

![gy-521](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/gy-521.jpg)

2. 直流电机驱动模块 （TB6612FNG）

![tb6612fng](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/tb6612fng.jpg)

3. 编码器模块（随直流电机一体）（MG513-30）

![mg513-30_1](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/mg513-30_1.jpg)

![mg513-30_2](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/mg513-30_2.jpg)

> MG513-30是该直流电机的型号，由电机驱动模块进行驱动，这里我们用它自带的编码器模块

### 其它

1. 机械元件

![mechanicals](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/mechanicals.jpg)

2. 12V锂电池

![12v_battery](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/12v_battery.jpg)

3. 电压转换模块（12V-5V）

![12v-5v](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/12v-5v.jpg)

> 直流电机12V供电，开发板5V供电

## 开发步骤

##### 1. 初始化APP，选择tm4c1294-v1开发板（对应TM4C1294-LaunchPad）
```bash
$ rap init --board tm4c1294-v1
```

##### 2. 添加陀螺仪驱动，id为gyro，型号选择GY-521，其余参数默认
```bash
$ rap device add gyro (GY-521)
```

##### 3. 添加电机驱动，id为motor，型号选择TB6612FNG
```bash
$ rap device add motor (TB6612FNG)
```

##### 4. 添加编码器驱动，id为encoder，型号选择MG513-30，其余参数默认
```bash
$ rap device add encoder (MG513-30)
```

##### 5. 编写控制算法
(见下文）

##### 6. 扫描开发板
```bash
$ rap scan
```

##### 7. 部署应用
```bash
$ rap deploy
```

## 控制算法

PID（比例-积分-微分）控制算法是工程上最常用的自动控制算法，参数P实现基本控制作用，参数D避免系统震荡，参数I用来消除系统静差，本平衡小车系统由PID算法进行控制，从而保持平衡

平衡车内部有两个反馈环，一个是由陀螺仪反馈姿态倾角和角速度构成的**直立环**，一个是由编码器反馈直流电机转速构成的**速度环**，由此构成一个串级PID控制系统（见下图），速度环控制的输出作为直立环控制的输入，直立环由**PD控制**（比例-微分控制）系统构成，保证小车的基本平衡（参数P的作用）和避免震荡（参数D的作用），速度环由**PI控制**（比例-积分控制）系统，消除姿态倾角的静差（参数I的作用）

![control_block](https://raw.githubusercontent.com/young-mu/RuffAC/master/res/control_block.png)

> 具体PID算法的原理及推导过程请参考自控控制专业书籍，这里只对算法作用和各个参数的意义进行简要说明

## 控制程序

`src/index.js`

```js
$.ready(function(error) {
    if (error) {
        console.log('error', error);
        return;
    }

    var gyro = $('#gyro'); // 陀螺仪设备
    var motor = $('#motor'); // 电机驱动设备
    var enc = $('#encoder'); // 编码器设备

    // 直立环PD控制
    var getBalancePwm = function (actualAngle, actualGyro) {
        var targetAngle = 0.8; // 小车静止平衡时的姿态角度
        var kP = 80; // 直立环比例(P)控制参数
        var kD = 2; // 直立环微分(D)控制参数

        // 直立环控制分量
        var balancePwm = kP * (actualAngle - targetAngle) + kD * actualGyro;
        return balancePwm / 1000;
    };

    // 速度环PI控制
    var encoder = 0;
    var sumEncoder = 0;
    var getVelocityPwm = function (actualLEncoder, actualREncoder) {
        var targetVelocity = 0; // 小车静止平衡时的电机输出转速
        var kP = 3; // 速度环比例(P)控制参数
        var kI = kP / 200; // 速度环积分(I)控制参数

        // FIR二阶低通滤波
        encoder = 0.2 * (actualLEncoder + actualREncoder - targetVelocity) + 0.8 * encoder;
        sumEncoder += encoder;

        // 积分限幅
        if (sumEncoder >= 1000) {
            sumEncoder = 1000;
        }
        if (sumEncoder <= -1000) {
            sumEncoder = -1000;
        }

        // 速度换控制分量
        var velocityPwm = kP * encoder + kI * sumEncoder;
        return (velocityPwm / 1000);
    };

    var cycle = 20; // 采样/控制周期均为20ms，即1s采样/控制50次
    var gyroAcquire, encAcquire, balanceControl;

    var angleX = 0;
    var gyroY = 0;
    var rpm = 0;

    // 每隔20ms，获取陀螺仪沿X轴的姿态倾角和角速度
    gyroAcquire = setInterval(function() {
        gyro.getFusedMotionX(cycle, function (error, _angleX, _gyroY) {
            angleX = _angleX;
            gyroY = _gyroY;
        });
    }, cycle);

    // 每隔20ms，获取编码器的速度值
    encAcquire = setInterval(function() {
        enc.getRpm(function (error, _rpm) {
            rpm = _rpm;
        });
    }, cycle);

    // 每隔20ms，利用反馈值计算控制量，控制电机正反转
    balanceControl = setInterval(function() {
        var balancePwm = getBalancePwm(angleX, gyroY);
        var velocityPwm = getVelocityPwm(rpm, rpm);
        var pwmDuty = balancePwm - velocityPwm;

        if (pwmDuty >= 0) {
            if (pwmDuty >= 1) {
                pwmDuty = 1;
            }
            // 控制车身前进（电机A正转B反转，A与B相差180度安装）
            motor.forwardRotateA(pwmDuty);
            motor.backwardRotateB(pwmDuty);
        } else {
            if (pwmDuty <= -1) {
                pwmDuty = -1;
            }
            // 控制车身后退（电机A反转B正转，A与B相差180度安装）
            motor.backwardRotateA(-pwmDuty);
            motor.forwardRotateB(-pwmDuty);
        }

        // 若倾角超过20度，停止整个控制系统运行
        if (angleX >= 20 || angleX <= -20) {
            // 停止陀螺仪采样
            clearInterval(gyroAcquire);
            // 停止编码器采样
            clearInterval(encAcquire);
            // 停止电机控制逻辑
            clearInterval(balanceControl);
            // 停止电机A/B转动
            motor.stopRotateA();
            motor.stopRotateB();
        }
    }, cycle);
});
```

## Ruff MCU与常规MCU开发的区别

#### 开发效率

1. Ruff作为一个IOT操作系统，对上跨开发平台，即在Windows/Linux/Mac都可以进行开发，不需要安装交叉编译工具或者指定版本的IDE
2. Ruff SDK中rap生产力工具快速实现代码部署，驱动/软件包的安装
3. Ruff Registry软件仓库包含嵌入式开发常用的各种模块驱动，随拿随用，若没有某个模块的驱动，Ruff提供常用接口驱动（如GPIO/I2C/UART/PWM/ADC/QEI等），可根据这些接口驱动，迅速开发驱动，并可发布到软件仓库中

#### 可移植性

Ruff作为一个嵌入式操作系统，对下跨硬件平台（可以运行在任何开发板/芯片/架构上），而设备驱动逻辑（如本案列使用的GY-521等）由Javascript编写，天生具备跨平台能力，因此代码具有可复用性

## FAQ

> Q: Ruff MPU版（ruff-mbd-v1）可以作为主控平台么？

A: 不能，因为底层的OpenWRT（基于Linux）不是实时操作系统，系统启动后会运行很多进程，Ruff进程不一定时刻占有CPU，因此不能稳定地每隔一个控制周期（比如20ms）获得传感器数据，不满足控制系统的实时性要求。而MCU版Ruff的底层操作系统是Nuttx RTOS，能够保证实时操作

> Q: 控制系统中控制周期是多少？

A: 控制周期为20ms，即1秒内控制系统控制50次

> Q: 用Ruff MCU开发板开发平衡车与用其它开发板（如stm32的开发板）开发，有什么相同点与不同点？

A: 相同点是都满足实时性控制的要求（如本案例的20ms控制周期），不同点在前文已经说过，主要体现在开发效率和可移植性两个方面，用其它开发板开发，要面对各种接口协议，模块协议等其它问题，且代码不具备可移植性和复用性，但用Ruff开发，直面控制算法逻辑，而不用关心硬件模块的任何细节，正因为没有任何硬件平台的逻辑，程序本身具备可复用性
