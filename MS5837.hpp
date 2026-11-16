#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: MS5837 压力温度传感器（I2C）驱动模块 / Driver module for the MS5837 pressure and temperature sensor over I2C
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "i2c.hpp"
#include "libxr_def.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "print.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

/**
 * @brief MS5837 压力温度传感器驱动模块。
 *        Driver module for the MS5837 pressure and temperature sensor.
 * @note 构造函数等待初始化成功后启动采样线程；实例及 I2C 须长期有效。
 *       Construction waits for initialization before starting the sampling
 *       thread; the instance and I2C must outlive the thread.
 * @note Init() 和 Read() 使用实例内互斥锁；共享 I2C 的其他访问须由调用方串行化。
 *       Init() and Read() use an instance mutex; callers must serialize
 *       other accesses to the shared I2C controller.
 */
class MS5837 {
public:
  /// 默认的 7 位 I2C 从设备地址（0x76），不含读写位。
  /// Default 7-bit I2C device address (0x76), without the R/W bit.
  static constexpr uint8_t ADDRESS_DEFAULT = 0x76;
  /// 备用的 7 位 I2C 地址（0x77）；是否可用取决于硬件。
  /// Alternate 7-bit I2C address (0x77); availability depends on hardware.
  static constexpr uint8_t ADDRESS_ALTERNATE = 0x77;
  /// 复位命令（0x1E），重新初始化传感器内部状态。
  /// Reset command (0x1E), reinitializing the sensor's internal state.
  static constexpr uint8_t COMMAND_RESET = 0x1E;
  /// ADC 结果读取命令（0x00），读取 24 位转换值。
  /// ADC read command (0x00), reading a 24-bit conversion result.
  static constexpr uint8_t COMMAND_ADC_READ = 0x00;
  /// PROM 读取命令基址（0xA0）；各字地址按 2 递增。
  /// PROM read command base (0xA0); word addresses increase by two.
  static constexpr uint8_t COMMAND_PROM_READ = 0xA0;
  /// 启动压力 D1 转换（0x4A），过采样率 OSR 为 8192。
  /// Start pressure D1 conversion (0x4A) with an OSR of 8192.
  static constexpr uint8_t COMMAND_CONVERT_D1_8192 = 0x4A;
  /// 启动温度 D2 转换（0x5A），过采样率 OSR 为 8192。
  /// Start temperature D2 conversion (0x5A) with an OSR of 8192.
  static constexpr uint8_t COMMAND_CONVERT_D2_8192 = 0x5A;

  /// 自动识别允许的 PROM 灵敏度系数 C1 上限，属于经验阈值。
  /// Heuristic upper bound for PROM sensitivity C1 during auto-detection.
  static constexpr uint16_t MS5837_02BA_MAX_SENSITIVITY = 49000;
  /// 有效 C1 区间内的型号分界：大于此值为 02BA，否则为 30BA。
  /// Model boundary within the valid C1 range: above selects 02BA, else 30BA.
  static constexpr uint16_t MS5837_02BA_30BA_SEPARATION = 37000;
  /// 自动识别允许的 PROM 灵敏度系数 C1 下限，属于经验阈值。
  /// Heuristic lower bound for PROM sensitivity C1 during auto-detection.
  static constexpr uint16_t MS5837_30BA_MIN_SENSITIVITY = 26000;

  /**
   * @brief 传感器型号及自动识别状态。
   *        Sensor model and automatic detection state.
   */
  enum class Model : uint8_t {
    AUTO = 0,  ///< 根据 PROM 系数 C1 自动识别
               ///< Detect the model from PROM coefficient C1
    MS5837_30BA = 1,  ///< MS5837-30BA 型号
                      ///< MS5837-30BA model
    MS5837_02BA = 2,  ///< MS5837-02BA 型号
                      ///< MS5837-02BA model
    UNRECOGNISED = 255  ///< 型号未知或不受支持
                        ///< Unknown or unsupported model
  };

  /**
   * @brief 一次采样的测量结果，也是 Topic 发布的数据类型。
   *        Measurement result of one sample and payload published by the Topic.
   * @note 使用测量值前先检查 valid；数值为零也可能表示无效数据。
   *       Check valid before using measurements; zeros may indicate invalid data.
   */
  struct Data {
    float pressure_pa = 0.0f;  ///< 补偿后的绝对压力，单位 Pa
                               ///< Compensated absolute pressure in Pa
    float pressure_mbar = 0.0f;  ///< 补偿后的绝对压力，单位 mbar（hPa）
                                 ///< Compensated absolute pressure in mbar (hPa)
    float temperature_c = 0.0f;  ///< 补偿后的传感器温度，单位摄氏度
                                 ///< Compensated sensor temperature in degrees Celsius
    float depth_m = 0.0f;  ///< 以 101300 Pa 为参考压力的液体深度，单位 m，可为负
                           ///< Liquid depth relative to 101300 Pa in m; may be negative
    float altitude_m = 0.0f;  ///< 按气压公式估算的海拔，单位 m，适用于空气环境
                              ///< Barometric altitude estimate in m, intended for air
                              ///< operation
    Model model = Model::UNRECOGNISED;  ///< 本次采样使用的型号；无效数据默认为未知
                                        ///< Model used for this sample; defaults to
                                        ///< unknown for invalid data
    bool valid = false;  ///< 压力 Pa、温度、深度及海拔均为有限值时为 true
                         ///< True when pressure in Pa, temperature, depth and altitude
                         ///< are finite
    LibXR::ErrorCode error = LibXR::ErrorCode::FAILED;  ///< 采样或初始化错误码；有效采样为 OK
                                                        ///< Sample or initialization
                                                        ///< error code; OK for a valid
                                                        ///< sample
  };

  /**
   * @brief I2C 地址、型号、深度计算及后台采样配置。
   *        I2C address, model, depth calculation and background sampling settings.
   */
  struct Param {
    uint16_t i2c_address;  ///< 7 位 I2C 从设备地址，无需左移，默认 0x76
                           ///< Unshifted 7-bit I2C device address, default 0x76
    Model model;  ///< AUTO 使用 C1 经验规则；显式型号覆盖识别结果
                  ///< AUTO uses the C1 heuristic; an explicit model overrides detection
    float fluid_density;  ///< 深度计算的液体密度，单位 kg/m^3，须大于零
                          ///< Liquid density for depth in kg/m^3; must be positive
    const char *topic_name;  ///< 发布 Data 的 Topic 名称，须为非空字符串
                             ///< Topic name for Data publication; must be nonempty
    const char *command_name;  ///< RamFS 命令名称，须为非空字符串
                               ///< RamFS command name; must be nonempty
    uint32_t sample_period_ms;  ///< 成功采样后的休眠时间，单位 ms，须大于零
                                ///< Sleep after a successful sample in ms; must be
                                ///< positive
    uint32_t retry_period_ms;  ///< 初始化或采样失败后的重试等待，单位 ms，须大于零
                               ///< Retry delay after initialization or sampling failure
                               ///< in ms; positive
    size_t task_stack_depth;  ///< 采样线程栈大小，单位字节，须大于零
                              ///< Sampling thread stack size in bytes; must be positive
  };

  /**
   * @brief 注册 Topic 和 RamFS 命令，初始化传感器并启动采样线程。
   *        Register the Topic and RamFS command, initialize the sensor and start sampling.
   * @param i2c 连接传感器的 I2C，须在采样线程生命周期内有效。
   *            I2C connected to the sensor; must outlive the sampling thread.
   * @param ramfs 注册传感器命令的文件系统。
   *              Filesystem receiving the sensor command.
   * @param param 地址、型号、液体密度、名称、间隔及线程栈配置。
   *              Address, model, density, names, delays and thread stack settings.
   * @note 在线程上下文调用；初始化失败时发布无效数据并持续重试，成功后才返回。
   *       Call from thread context; initialization failures publish invalid data and
   *       retry until initialization succeeds, so construction may block indefinitely.
   */
  MS5837(LibXR::I2C &i2c, LibXR::RamFS &ramfs,
         const Param &param = {.i2c_address = ADDRESS_DEFAULT,
                               .model = Model::AUTO,
                               .fluid_density = 1029.0f,
                               .topic_name = "ms5837",
                               .command_name = "ms5837",
                               .sample_period_ms = 100,
                               .retry_period_ms = 1000,
                               .task_stack_depth = 1024})
      : i2c_(&i2c), address_(param.i2c_address), requested_model_(param.model),
        fluid_density_(param.fluid_density),
        sample_period_ms_(param.sample_period_ms),
        retry_period_ms_(param.retry_period_ms),
        topic_(LibXR::Topic::CreateTopic<Data>(param.topic_name)),
        command_file_(
            LibXR::RamFS::CreateCommand(param.command_name, CommandFunc, this)),
        read_op_(io_sem_, kIoTimeoutMs), write_op_(io_sem_, kIoTimeoutMs) {
    ASSERT(address_ <= 0x7FU);
    ASSERT(fluid_density_ > 0.0f);
    ASSERT(param.topic_name != nullptr && param.topic_name[0] != '\0');
    ASSERT(param.command_name != nullptr && param.command_name[0] != '\0');
    ASSERT(sample_period_ms_ > 0U && retry_period_ms_ > 0U &&
           param.task_stack_depth > 0U);

    ramfs.Add(command_file_);

    while (true) {
      const auto result = Init();
      if (result == LibXR::ErrorCode::OK) {
        XR_LOG_PASS("MS5837: Init success.");
        break;
      }

      XR_LOG_ERROR("MS5837: Init failed (%d). Retry...",
                   static_cast<int>(result));
      PublishInvalid(result);
      LibXR::Thread::Sleep(retry_period_ms_);
    }

    thread_.Create(this, ThreadFunc, "ms5837_thread", param.task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 禁止复制包含后台线程及同步资源的实例。
   *        Prevent copying an instance with a background thread and synchronization resources.
   */
  MS5837(const MS5837 &) = delete;
  /**
   * @brief 禁止复制赋值传感器实例。
   *        Prevent copy assignment of sensor instances.
   */
  MS5837 &operator=(const MS5837 &) = delete;

  /**
   * @brief 复位传感器、读取 PROM、校验 CRC4 并确定型号。
   *        Reset the sensor, read PROM, validate CRC4 and select the model.
   * @return 成功为 OK；CRC 错误为 CHECK_ERR，未知型号为 NOT_SUPPORT，通信失败返回 I2C 错误码。
   *         OK on success, CHECK_ERR for CRC failure, NOT_SUPPORT for an unknown model,
   *         or the I2C error code on communication failure.
   * @note 在线程上下文阻塞执行，与 Read() 串行；失败后 initialized_ 为 false。
   *       Blocks in thread context and is serialized with Read(); failure leaves
   *       initialized_ false.
   * @note 更新最近错误码；此方法本身不发布 Topic，也不更新缓存快照。
   *       Updates the last error; this method itself neither publishes the Topic
   *       nor updates the cached snapshot.
   */
  LibXR::ErrorCode Init() {
    LibXR::Mutex::LockGuard guard(bus_mutex_);
    initialized_ = false;

    auto result = WriteCommand(COMMAND_RESET);
    if (result != LibXR::ErrorCode::OK) {
      SetLastError(result);
      return result;
    }
    LibXR::Thread::Sleep(10);

    std::array<uint16_t, 8> prom{};
    for (uint8_t index = 0; index < 7; ++index) {
      result =
          WriteCommand(static_cast<uint8_t>(COMMAND_PROM_READ + index * 2U));
      if (result != LibXR::ErrorCode::OK) {
        SetLastError(result);
        return result;
      }

      uint8_t bytes[2]{};
      result = ReadBytes(bytes, sizeof(bytes));
      if (result != LibXR::ErrorCode::OK) {
        SetLastError(result);
        return result;
      }
      prom[index] = static_cast<uint16_t>(
          static_cast<uint16_t>(bytes[0]) << 8U | bytes[1]);
    }

    if ((prom[0] >> 12U) != Crc4(prom)) {
      SetLastError(LibXR::ErrorCode::CHECK_ERR);
      return LibXR::ErrorCode::CHECK_ERR;
    }

    Model detected = DetectModel(prom[1]);
    if (requested_model_ != Model::AUTO) {
      detected = requested_model_;
    }
    if (detected != Model::MS5837_30BA && detected != Model::MS5837_02BA) {
      model_ = Model::UNRECOGNISED;
      SetLastError(LibXR::ErrorCode::NOT_SUPPORT);
      return LibXR::ErrorCode::NOT_SUPPORT;
    }

    prom_ = prom;
    model_ = detected;
    initialized_ = true;
    SetLastError(LibXR::ErrorCode::OK);
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 顺序执行 OSR 8192 压力和温度转换，计算补偿后的测量值。
   *        Run pressure and temperature conversions at OSR 8192 and compensate the results.
   * @param output 采样结果；未初始化时保持原值，通信失败时写入无效数据。
   *               Sample result; unchanged when uninitialized, invalid on I2C failure.
   * @return 有效采样为 OK，未初始化为 STATE_ERR，计算结果非有限为 CHECK_ERR，
   *         通信失败返回 I2C 错误码。
   *         OK for a valid sample, STATE_ERR when uninitialized, CHECK_ERR for
   *         non-finite results, or the I2C error code on communication failure.
   * @note 在线程上下文调用；与 Init() 串行，两次转换各等待 20 ms，另有 I2C 耗时。
   *       Call from thread context; serialized with Init(), with a 20 ms wait per
   *       conversion plus I2C transfer time.
   * @note 通信或计算失败后须重新 Init()；此方法本身不发布 Topic 或更新缓存快照。
   *       Communication or calculation failure requires Init(); this method itself
   *       neither publishes the Topic nor updates the cached snapshot.
   */
  LibXR::ErrorCode Read(Data &output) {
    LibXR::Mutex::LockGuard guard(bus_mutex_);
    if (!initialized_) {
      return LibXR::ErrorCode::STATE_ERR;
    }

    uint32_t pressure_raw = 0;
    uint32_t temperature_raw = 0;
    auto result = ReadAdc(COMMAND_CONVERT_D1_8192, pressure_raw);
    if (result == LibXR::ErrorCode::OK) {
      result = ReadAdc(COMMAND_CONVERT_D2_8192, temperature_raw);
    }
    if (result != LibXR::ErrorCode::OK) {
      initialized_ = false;
      SetLastError(result);
      output = InvalidData(result);
      return result;
    }

    output = Calculate(pressure_raw, temperature_raw);
    if (output.error != LibXR::ErrorCode::OK) {
      initialized_ = false;
    }
    SetLastError(output.error);
    return output.error;
  }

  /**
   * @brief 加锁复制最近一次发布的数据，不触发传感器采样。
   *        Copy the latest published data under a mutex without sampling the sensor.
   * @return 最近的有效或无效数据；首次发布前为默认构造的无效 Data。
   *         Latest valid or invalid data; default-constructed invalid Data before publication.
   * @note 在线程上下文调用；可与后台发布并发使用。
   *       Call from thread context; safe alongside background publication.
   */
  Data Snapshot() const {
    LibXR::Mutex::LockGuard guard(data_mutex_);
    return latest_data_;
  }

  /**
   * @brief 检查缓存数据的有效性及非有限测量值，并输出警告日志。
   *        Check cached validity and non-finite measurements, logging warnings.
   * @note 在线程上下文调用，仅检查快照，不触发采样或重新初始化。
   *       Call from thread context; inspects a snapshot without sampling or reinitializing.
   */
  void OnMonitor() {
    const Data data = Snapshot();
    if (!data.valid) {
      XR_LOG_WARN("MS5837 invalid sample, error=%d",
                  static_cast<int>(data.error));
      return;
    }
    if (!std::isfinite(data.pressure_pa) ||
        !std::isfinite(data.temperature_c) || !std::isfinite(data.depth_m) ||
        !std::isfinite(data.altitude_m)) {
      XR_LOG_WARN("MS5837 non-finite data: pressure=%f temperature=%f depth=%f "
                  "altitude=%f",
                  data.pressure_pa, data.temperature_c, data.depth_m,
                  data.altitude_m);
    }
  }

private:
  static constexpr uint32_t kIoTimeoutMs = 100;
  static constexpr float kAtmosphericPressurePa = 101300.0f;
  static constexpr float kGravity = 9.80665f;

  static Model DetectModel(uint16_t sensitivity) {
    if (sensitivity < MS5837_30BA_MIN_SENSITIVITY ||
        sensitivity > MS5837_02BA_MAX_SENSITIVITY) {
      return Model::UNRECOGNISED;
    }
    return sensitivity > MS5837_02BA_30BA_SEPARATION ? Model::MS5837_02BA
                                                     : Model::MS5837_30BA;
  }

  static uint8_t Crc4(const std::array<uint16_t, 8> &source) {
    auto prom = source;
    prom[0] &= 0x0FFFU;
    prom[7] = 0;
    uint16_t remainder = 0;
    for (uint8_t index = 0; index < 16; ++index) {
      remainder ^= (index % 2U == 0U)
                       ? static_cast<uint16_t>(prom[index >> 1U] >> 8U)
                       : static_cast<uint16_t>(prom[index >> 1U] & 0x00FFU);
      for (uint8_t bit = 0; bit < 8; ++bit) {
        remainder = (remainder & 0x8000U) != 0U
                        ? static_cast<uint16_t>((remainder << 1U) ^ 0x3000U)
                        : static_cast<uint16_t>(remainder << 1U);
      }
    }
    return static_cast<uint8_t>((remainder >> 12U) & 0x0FU);
  }

  LibXR::ErrorCode WriteCommand(uint8_t command) {
    return i2c_->Write(address_, LibXR::ConstRawData(&command, sizeof(command)),
                       write_op_);
  }

  LibXR::ErrorCode ReadBytes(uint8_t *data, size_t size) {
    return i2c_->Read(address_, LibXR::RawData(data, size), read_op_);
  }

  LibXR::ErrorCode ReadAdc(uint8_t conversion_command, uint32_t &value) {
    auto result = WriteCommand(conversion_command);
    if (result != LibXR::ErrorCode::OK) {
      return result;
    }
    LibXR::Thread::Sleep(20);

    result = WriteCommand(COMMAND_ADC_READ);
    if (result != LibXR::ErrorCode::OK) {
      return result;
    }

    uint8_t bytes[3]{};
    result = ReadBytes(bytes, sizeof(bytes));
    if (result != LibXR::ErrorCode::OK) {
      return result;
    }
    value = (static_cast<uint32_t>(bytes[0]) << 16U) |
            (static_cast<uint32_t>(bytes[1]) << 8U) |
            static_cast<uint32_t>(bytes[2]);
    return LibXR::ErrorCode::OK;
  }

  Data Calculate(uint32_t pressure_raw, uint32_t temperature_raw) const {
    const int64_t d_temperature = static_cast<int64_t>(temperature_raw) -
                                  static_cast<int64_t>(prom_[5]) * 256LL;
    int64_t sensitivity = 0;
    int64_t offset = 0;
    int64_t pressure = 0;
    int32_t temperature =
        static_cast<int32_t>(2000LL + d_temperature * prom_[6] / 8388608LL);
    int64_t offset_second = 0;
    int64_t sensitivity_second = 0;

    if (model_ == Model::MS5837_02BA) {
      sensitivity = static_cast<int64_t>(prom_[1]) * 65536LL +
                    static_cast<int64_t>(prom_[3]) * d_temperature / 128LL;
      offset = static_cast<int64_t>(prom_[2]) * 131072LL +
               static_cast<int64_t>(prom_[4]) * d_temperature / 64LL;
      if (temperature / 100 < 20) {
        const int64_t delta = static_cast<int64_t>(temperature) - 2000LL;
        offset_second = 31LL * delta * delta / 8LL;
        sensitivity_second = 63LL * delta * delta / 32LL;
        temperature -= static_cast<int32_t>(11LL * d_temperature *
                                            d_temperature / 34359738368LL);
      }
      pressure = (static_cast<int64_t>(pressure_raw) *
                      (sensitivity - sensitivity_second) / 2097152LL -
                  (offset - offset_second)) /
                 32768LL;
    } else {
      sensitivity = static_cast<int64_t>(prom_[1]) * 32768LL +
                    static_cast<int64_t>(prom_[3]) * d_temperature / 256LL;
      offset = static_cast<int64_t>(prom_[2]) * 65536LL +
               static_cast<int64_t>(prom_[4]) * d_temperature / 128LL;
      const int32_t base_temperature = temperature;
      if (base_temperature / 100 < 20) {
        const int64_t delta = static_cast<int64_t>(base_temperature) - 2000LL;
        offset_second = 3LL * delta * delta / 2LL;
        sensitivity_second = 5LL * delta * delta / 8LL;
        temperature -= static_cast<int32_t>(3LL * d_temperature *
                                            d_temperature / 8589934592LL);
        if (base_temperature / 100 < -15) {
          const int64_t very_low_delta =
              static_cast<int64_t>(base_temperature) + 1500LL;
          offset_second += 7LL * very_low_delta * very_low_delta;
          sensitivity_second += 4LL * very_low_delta * very_low_delta;
        }
      } else if (base_temperature / 100 >= 20) {
        const int64_t delta = static_cast<int64_t>(base_temperature) - 2000LL;
        offset_second = delta * delta / 16LL;
        temperature -= static_cast<int32_t>(2LL * d_temperature *
                                            d_temperature / 137438953472LL);
      }
      pressure = (static_cast<int64_t>(pressure_raw) *
                      (sensitivity - sensitivity_second) / 2097152LL -
                  (offset - offset_second)) /
                 8192LL;
    }

    const float pressure_mbar = model_ == Model::MS5837_02BA
                                    ? static_cast<float>(pressure) / 100.0f
                                    : static_cast<float>(pressure) / 10.0f;
    const float pressure_pa = pressure_mbar * 100.0f;
    const float temperature_c = static_cast<float>(temperature) / 100.0f;

    Data output{};
    output.pressure_pa = pressure_pa;
    output.pressure_mbar = pressure_mbar;
    output.temperature_c = temperature_c;
    output.depth_m =
        (pressure_pa - kAtmosphericPressurePa) / (fluid_density_ * kGravity);
    output.altitude_m = (1.0f - std::pow(pressure_mbar / 1013.25f, 0.190284f)) *
                        145366.45f * 0.3048f;
    output.model = model_;
    output.valid = std::isfinite(output.pressure_pa) &&
                   std::isfinite(output.temperature_c) &&
                   std::isfinite(output.depth_m) &&
                   std::isfinite(output.altitude_m);
    output.error =
        output.valid ? LibXR::ErrorCode::OK : LibXR::ErrorCode::CHECK_ERR;
    return output;
  }

  static Data InvalidData(LibXR::ErrorCode error) {
    Data output{};
    output.error = error;
    return output;
  }

  void SetLastError(LibXR::ErrorCode error) {
    LibXR::Mutex::LockGuard guard(data_mutex_);
    last_error_ = error;
  }

  void PublishInvalid(LibXR::ErrorCode error) {
    Data data = InvalidData(error);
    {
      LibXR::Mutex::LockGuard guard(data_mutex_);
      latest_data_ = data;
      last_error_ = error;
    }
    topic_.Publish(data);
  }

  void PublishSample(const Data &data) {
    {
      LibXR::Mutex::LockGuard guard(data_mutex_);
      latest_data_ = data;
      last_error_ = data.error;
    }
    Data copy = data;
    topic_.Publish(copy);
  }

  static void ThreadFunc(MS5837 *self) {
    while (true) {
      Data data{};
      const auto result = self->Read(data);
      if (result == LibXR::ErrorCode::OK && data.valid) {
        self->PublishSample(data);
        LibXR::Thread::Sleep(self->sample_period_ms_);
      } else {
        const auto error = result == LibXR::ErrorCode::OK ? data.error : result;
        self->PublishInvalid(error);
        XR_LOG_WARN("MS5837: sample failed (%d). Reinitializing.",
                    static_cast<int>(error));
        LibXR::Thread::Sleep(self->retry_period_ms_);
        while (self->Init() != LibXR::ErrorCode::OK) {
          LibXR::Thread::Sleep(self->retry_period_ms_);
        }
      }
    }
  }

  static const char *ModelName(Model model) {
    switch (model) {
    case Model::MS5837_30BA:
      return "MS5837-30BA";
    case Model::MS5837_02BA:
      return "MS5837-02BA";
    case Model::AUTO:
      return "AUTO";
    default:
      return "UNRECOGNISED";
    }
  }

  static void PrintData(const Data &data) {
    LibXR::STDIO::Printf<"pressure=%+f Pa (%+f mbar), temperature=%+f C, "
                         "depth=%+f m, altitude=%+f m, "
                         "model=%s, valid=%d, error=%d\r\n">(
        data.pressure_pa, data.pressure_mbar, data.temperature_c, data.depth_m,
        data.altitude_m, ModelName(data.model), data.valid ? 1 : 0,
        static_cast<int>(data.error));
  }

  static int CommandFunc(MS5837 *self, int argc, char **argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<"  show [time_ms] [interval_ms] - Print sensor data "
                           "periodically.\r\n">();
      LibXR::STDIO::Printf<
          "  info                         - Show sensor status.\r\n">();
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "info") == 0) {
      Data data{};
      LibXR::ErrorCode last_error = LibXR::ErrorCode::FAILED;
      {
        LibXR::Mutex::LockGuard guard(self->data_mutex_);
        data = self->latest_data_;
        last_error = self->last_error_;
      }
      LibXR::STDIO::Printf<"address=0x%02x, model=%s, density=%+f kg/m^3, "
                           "last_error=%d, valid=%d\r\n">(
          self->address_, ModelName(data.model), self->fluid_density_,
          static_cast<int>(last_error), data.valid ? 1 : 0);
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "show") == 0) {
      const int duration = std::atoi(argv[2]);
      const int interval = std::clamp(std::atoi(argv[3]), 2, 1000);
      if (duration < 0) {
        LibXR::STDIO::Printf<"Error: time_ms must be non-negative.\r\n">();
        return -1;
      }
      int elapsed = 0;
      while (elapsed < duration) {
        PrintData(self->Snapshot());
        LibXR::Thread::Sleep(static_cast<uint32_t>(interval));
        elapsed += interval;
      }
      return 0;
    }

    LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
    return -1;
  }

  LibXR::I2C *i2c_;
  uint16_t address_;
  Model requested_model_;
  Model model_ = Model::UNRECOGNISED;
  float fluid_density_;
  uint32_t sample_period_ms_;
  uint32_t retry_period_ms_;
  bool initialized_ = false;
  std::array<uint16_t, 8> prom_{};
  LibXR::Topic topic_;
  LibXR::RamFS::File command_file_;
  Data latest_data_{};
  LibXR::ErrorCode last_error_ = LibXR::ErrorCode::FAILED;
  mutable LibXR::Mutex data_mutex_;
  LibXR::Mutex bus_mutex_;
  LibXR::Semaphore io_sem_;
  LibXR::ReadOperation read_op_;
  LibXR::WriteOperation write_op_;
  LibXR::Thread thread_;
};
