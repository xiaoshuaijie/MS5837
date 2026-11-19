# MS5837

MS5837 压力温度传感器（I²C）驱动模块 / Driver module for the MS5837 pressure and temperature sensor over I²C

## 1. 模块作用 / Purpose

模块构造时复位 MS5837，读取 PROM 校准系数并执行 CRC4 校验，根据 C1 灵敏度自动识别 MS5837-30BA 或 MS5837-02BA。初始化失败时输出错误日志，按 retry_period_ms 重试；初始化成功后创建 ms5837_thread，默认每 100 ms 读取一次压力和温度。

The module resets the MS5837 during construction, reads the PROM calibration coefficients, verifies CRC4 and detects MS5837-30BA or MS5837-02BA from the C1 sensitivity value. Initialization failures are logged and retried according to retry_period_ms. After initialization, ms5837_thread reads pressure and temperature every 100 ms by default.

每次读取包含 D1 压力转换和 D2 温度转换，使用数据手册的二阶温度补偿公式计算压力、温度、深度和海拔，并通过 Topic 发布 MS5837::Data。I²C 或计算失败时发布 valid=false 的数据，保存错误码并重新初始化。

Each sample performs D1 pressure and D2 temperature conversions, applies the datasheet second-order compensation formulas and calculates pressure, temperature, depth and altitude. The result is published as MS5837::Data. I²C or calculation failures publish valid=false with the error code and trigger reinitialization.

模块还会在 RamFS 中注册 ms5837 命令，用于周期查看数据和查询当前状态。OnMonitor() 检查无效数据以及 NaN/Inf 浮点结果。

The module also registers the ms5837 RamFS command for periodic data display and status inspection. OnMonitor() checks invalid samples and NaN/Inf results.

## 2. MS5837 协议与计算 / Protocol and Calculations

MS5837 使用不带 R/W 位的 7 位 I²C 地址，默认地址为 0x76，部分器件或硬件配置使用 0x77。

MS5837 uses a 7-bit I²C address without the R/W bit. The default address is 0x76; some devices or board configurations use 0x77.

| 命令 / Command | 数值 / Value | 说明 / Meaning |
| --- | ---: | --- |
| RESET | 0x1E | 复位传感器 / Reset the sensor |
| ADC_READ | 0x00 | 读取 24 位 ADC / Read the 24-bit ADC |
| PROM_READ | 0xA0 + 2 * index | 读取 PROM 第 index 项 / Read PROM word index |
| CONVERT_D1_8192 | 0x4A | OSR 8192 压力转换 / OSR 8192 pressure conversion |
| CONVERT_D2_8192 | 0x5A | OSR 8192 温度转换 / OSR 8192 temperature conversion |

初始化读取 PROM 第 0～6 项，第 7 项按 CRC4 算法置零。C1 灵敏度小于 26000 或大于 49000 时视为未知型号；26000 <= C1 <= 37000 识别为 30BA，37000 < C1 <= 49000 识别为 02BA。Param::model 为 AUTO 时使用此自动识别；指定具体型号时可以覆盖自动识别结果。

Initialization reads PROM words 0 through 6 and treats word 7 as zero for CRC4. C1 values below 26000 or above 49000 are unrecognized; 26000 <= C1 <= 37000 selects 30BA and 37000 < C1 <= 49000 selects 02BA. Param::model = AUTO uses automatic detection; an explicit model overrides it.

压力、温度和深度的单位如下：

- pressure_pa：绝对压力，Pa。
- pressure_mbar：绝对压力，mbar。
- temperature_c：温度，°C。
- depth_m：按 (pressure_pa - 101300) / (fluid_density * 9.80665) 计算，单位 m。
- altitude_m：按参考库的气压高度公式计算，只适用于空气环境。

The published units are:

- pressure_pa: absolute pressure in Pa.
- pressure_mbar: absolute pressure in mbar.
- temperature_c: temperature in °C.
- depth_m: (pressure_pa - 101300) / (fluid_density * 9.80665) in m.
- altitude_m: altitude from the reference barometric formula, valid for air operation only.

## 3. 构造接口 / Constructor

    MS5837(LibXR::I2C& i2c,
           LibXR::RamFS& ramfs,
           const MS5837::Param& param = {...});

依赖参数：

- i2c：连接 MS5837 的 LibXR::I2C，模块独占该器件地址上的访问。
- ramfs：用于注册 ms5837 命令的 LibXR::RamFS。
- param.i2c_address：7 位 I²C 地址，默认 0x76，可配置为 0x77。
- param.model：AUTO、MS5837_30BA 或 MS5837_02BA。
- param.fluid_density：工作流体密度，默认海水 1029 kg/m³；淡水可设置为约 997 kg/m³。
- param.topic_name：发布 Topic 名称，默认 ms5837；多个实例必须使用不同名称。
- param.command_name：RamFS 命令名称，默认 ms5837；多个实例必须使用不同名称。
- param.sample_period_ms：成功采样后的周期，默认 100 ms。
- param.retry_period_ms：初始化或采样失败后的重试周期，默认 1000 ms。
- param.task_stack_depth：采样线程栈大小，单位为字节，默认 1024。

Dependency parameters:

- i2c: the LibXR::I2C connected to the MS5837.
- ramfs: the LibXR::RamFS that receives the ms5837 command.
- i2c_address: 7-bit I²C address, default 0x76, configurable to 0x77.
- model: AUTO, MS5837_30BA or MS5837_02BA.
- fluid_density: working-fluid density, default seawater 1029 kg/m³.
- topic_name: published Topic name, default ms5837; each instance needs a distinct name.
- command_name: RamFS command name, default ms5837; each instance needs a distinct name.
- sample_period_ms: interval after a successful sample, default 100 ms.
- retry_period_ms: retry interval after initialization or sampling failure, default 1000 ms.
- task_stack_depth: sampling-thread stack size in bytes, default 1024.

实例和 I²C 对象必须在整个采样线程生命周期内保持有效。构造函数会阻塞等待传感器初始化成功，因此应在 LibXR 平台初始化和硬件注册完成后创建实例。

The instance and the I²C object must remain valid for the lifetime of the sampling thread. Construction waits until sensor initialization succeeds, so create the instance after LibXR platform initialization and hardware registration.

## 4. Topic

模块发布一个 Topic，名称由 param.topic_name 指定，默认名称为 ms5837。Topic 载荷类型为 MS5837::Data。

The module publishes one Topic named by param.topic_name, defaulting to ms5837. Its payload type is MS5837::Data.

| 字段 / Field | 类型 / Type | 单位 / Unit | 说明 / Meaning |
| --- | --- | --- | --- |
| pressure_pa | float | Pa | 绝对压力 / Absolute pressure |
| pressure_mbar | float | mbar | 绝对压力 / Absolute pressure |
| temperature_c | float | °C | 温度 / Temperature |
| depth_m | float | m | 按流体密度估算的水深 / Water depth estimate |
| altitude_m | float | m | 按气压公式估算的海拔 / Barometric altitude estimate |
| model | MS5837::Model | — | 检测到的型号 / Detected model |
| valid | bool | — | 本次数据是否有效 / Whether this sample is valid |
| error | LibXR::ErrorCode | — | 最近一次采样或初始化结果 / Last sampling or initialization result |

成功消息的 valid=true 且 error=LibXR::ErrorCode::OK。失败消息的数值字段为零，valid=false，error 保存失败原因；消费者应先检查 valid 再使用压力、深度或海拔。

A successful message has valid=true and error=LibXR::ErrorCode::OK. Failure messages zero the numeric fields, set valid=false and retain the failure reason in error; consumers should check valid before using pressure, depth or altitude.

示例：

    MS5837::Data data{};
    LibXR::Topic::SyncSubscriber<MS5837::Data> subscriber("ms5837", data);

    while (true)
    {
      if (subscriber.Wait(500) == LibXR::ErrorCode::OK && data.valid)
      {
        // 使用 data.pressure_pa、data.temperature_c 或 data.depth_m。
      }
    }

## 5. RamFS 命令 / RamFS Commands

模块注册命令 ms5837：

- ms5837：显示帮助。
- ms5837 show <time_ms> <interval_ms>：在指定时间内打印最新数据；打印间隔限制为 2～1000 ms。
- ms5837 info：打印地址、型号、流体密度、最近错误和有效状态。

The module registers ms5837:

- ms5837: print usage.
- ms5837 show <time_ms> <interval_ms>: print the latest data for the requested duration; interval is clamped to 2–1000 ms.
- ms5837 info: print address, model, fluid density, last error and validity.

示例：

    ramfs:/bin$ ms5837 show 1000 100
    pressure=101325.000000 Pa (1013.250000 mbar), temperature=24.800000 C, depth=0.002000 m, altitude=-0.002000 m, model=MS5837-30BA, valid=1, error=0

## 6. 配置示例 / Configuration Example

当模块作为 XRobot 模块解析时，依赖填写为 BSP 通过 XR_REGISTER 注册的名称：

When the module is assembled by XRobot, dependencies use the names registered by the BSP with XR_REGISTER:

    modules:
      - module: ms5837
        id: ms5837_0
        args:
          - i2c: i2c1
          - ramfs: ramfs
          - param:
              i2c_address: 0x76
              model: MS5837::Model::AUTO
              fluid_density: 1029.0f
              topic_name: "ms5837"
              command_name: "ms5837"
              sample_period_ms: 100
              retry_period_ms: 1000
              task_stack_depth: 1024

如果模块暂时作为本地头文件使用，可在硬件初始化之后手动构造：

For a temporary local-header integration, construct it manually after hardware initialization:

    #include "../Modules/MS5837/MS5837.hpp"

    static MS5837 ms5837_0(
        i2c1, ramfs,
        {.i2c_address = MS5837::ADDRESS_DEFAULT,
         .model = MS5837::Model::AUTO,
         .fluid_density = 1029.0f,
         .topic_name = "ms5837",
         .command_name = "ms5837",
         .sample_period_ms = 100,
         .retry_period_ms = 1000,
         .task_stack_depth = 1024});

## 7. 依赖与硬件 / Dependencies and Hardware

依赖：

- LibXR 的 I2C、Topic、Thread、Mutex、RamFS 和 STDIO。
- 本模块不依赖 GPIO、PWM、Database 或其他 Module。

Dependencies:

- LibXR I2C, Topic, Thread, Mutex, RamFS and STDIO.
- The module has no GPIO, PWM, Database or other Module dependency.

硬件要求：

- 一片 MS5837-30BA 或 MS5837-02BA。
- 3.3 V 供电，I²C SDA/SCL 配置正确。
- SDA/SCL 需要符合总线电气要求的上拉电阻。
- BSP 必须启用 CubeMX I²C 外设，并将生成的 LibXR I²C 对象通过 XR_REGISTER 注册。
- BSP 必须提供并注册 LibXR::RamFS。

Hardware requirements:

- One MS5837-30BA or MS5837-02BA.
- 3.3 V power and correctly configured I²C SDA/SCL pins.
- SDA/SCL pull-ups suitable for the bus electrical characteristics.
- The BSP must enable an I²C peripheral in CubeMX and register the generated LibXR I²C object with XR_REGISTER.
- The BSP must provide and register a LibXR::RamFS.
