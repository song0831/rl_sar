/*
 * MJX ET6 two-PWM RL deployment for rl_sar.
 *
 * This target follows the rl_sar hardware-deployment style: C++ executable,
 * yaml config under policy/, and ONNX inference through InferenceRuntime.
 *
 * Hardware contract:
 *   observation = IMU(9) + four wheel RPM(4) + previous PWM action(2), history
 *   action      = steering PWM + forward-only throttle PWM
 */

#include "inference_runtime.hpp"
#include "logger.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/input.h>
#include <netdb.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef POLICY_DIR
#define POLICY_DIR "."
#endif

#ifndef BTN_SOUTH
#define BTN_SOUTH BTN_A
#endif
#ifndef BTN_EAST
#define BTN_EAST BTN_B
#endif
#ifndef BTN_NORTH
#define BTN_NORTH BTN_Y
#endif
#ifndef BTN_WEST
#define BTN_WEST BTN_X
#endif
#ifndef ABS_GAS
#define ABS_GAS 0x09
#endif
#ifndef ABS_BRAKE
#define ABS_BRAKE 0x0a
#endif

namespace
{

std::atomic<bool> g_stop_requested{false};

void handle_signal(int)
{
    g_stop_requested.store(true);
}

double now_monotonic_s()
{
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}

std::string now_iso()
{
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&time, &tm);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;
    std::ostringstream out;
    out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << "."
        << std::setw(3) << std::setfill('0') << ms.count();
    return out.str();
}

std::string now_filename_stamp()
{
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&time, &tm);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%d-%H%M%S") << "-"
        << std::setw(3) << std::setfill('0') << ms.count();
    return out.str();
}

std::filesystem::path unique_csv_path(const std::filesystem::path& requested)
{
    const std::filesystem::path dir = requested.parent_path();
    const std::string stem = requested.stem().empty() ? "rl_log_mjx_et6" : requested.stem().string();
    const std::string ext = requested.extension().empty() ? ".csv" : requested.extension().string();
    const std::string stamp = now_filename_stamp();

    for (int index = 0; index < 1000; ++index)
    {
        std::ostringstream name;
        name << stem << "_" << stamp;
        if (index > 0)
            name << "_" << std::setw(3) << std::setfill('0') << index;
        name << ext;
        std::filesystem::path candidate = dir.empty() ? std::filesystem::path(name.str()) : dir / name.str();
        if (!std::filesystem::exists(candidate))
            return candidate;
    }

    throw std::runtime_error("Could not allocate a unique telemetry CSV path near: " + requested.string());
}

float clampf(float value, float lo, float hi)
{
    return std::max(lo, std::min(hi, value));
}

float median_value(std::vector<float> values)
{
    if (values.empty())
        throw std::runtime_error("Cannot calculate median of an empty vector");
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    float median = values[middle];
    if (values.size() % 2 == 0)
    {
        const auto lower = std::max_element(values.begin(), values.begin() + middle);
        median = 0.5f * (median + *lower);
    }
    return median;
}

float signed_to_pulse(float value, float min_ms, float neutral_ms, float max_ms)
{
    value = clampf(value, -1.0f, 1.0f);
    if (value >= 0.0f)
        return neutral_ms + value * (max_ms - neutral_ms);
    return neutral_ms + value * (neutral_ms - min_ms);
}

float pulse_to_signed(float value_ms, float min_ms, float neutral_ms, float max_ms)
{
    if (value_ms >= neutral_ms)
        return clampf((value_ms - neutral_ms) / std::max(max_ms - neutral_ms, 1.0e-6f), 0.0f, 1.0f);
    return -clampf((neutral_ms - value_ms) / std::max(neutral_ms - min_ms, 1.0e-6f), 0.0f, 1.0f);
}

float slew(float current, float target, float rate_per_s, float dt)
{
    if (rate_per_s <= 0.0f)
        return target;
    const float max_delta = rate_per_s * std::max(dt, 0.0f);
    return current + clampf(target - current, -max_delta, max_delta);
}

float normalize_degrees(float value)
{
    float result = std::fmod(value + 180.0f, 360.0f);
    if (result < 0.0f)
        result += 360.0f;
    return result - 180.0f;
}

template <typename T>
T yaml_get(const YAML::Node& node, const std::string& key, const T& fallback)
{
    if (node[key])
        return node[key].as<T>();
    return fallback;
}

std::string lowercase(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string uppercase(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

int input_button_code_from_name(const std::string& raw_name)
{
    std::string name = uppercase(raw_name);
    name.erase(std::remove_if(name.begin(), name.end(), [](unsigned char c) { return std::isspace(c); }), name.end());
    if (name.empty())
        throw std::runtime_error("Empty gamepad button name");
    if (std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isdigit(c); }))
        return std::stoi(name);

    static const std::map<std::string, int> kButtonCodes = {
        {"BTN_SOUTH", BTN_SOUTH},
        {"BTN_A", BTN_SOUTH},
        {"A", BTN_SOUTH},
        {"BTN_EAST", BTN_EAST},
        {"BTN_B", BTN_EAST},
        {"B", BTN_EAST},
        {"BTN_NORTH", BTN_NORTH},
        {"BTN_Y", BTN_NORTH},
        {"Y", BTN_NORTH},
        {"BTN_WEST", BTN_WEST},
        {"BTN_X", BTN_WEST},
        {"X", BTN_WEST},
        {"BTN_TL", BTN_TL},
        {"LB", BTN_TL},
        {"BTN_TR", BTN_TR},
        {"RB", BTN_TR},
        {"BTN_TL2", BTN_TL2},
        {"LT", BTN_TL2},
        {"BTN_TR2", BTN_TR2},
        {"RT", BTN_TR2},
        {"BTN_SELECT", BTN_SELECT},
        {"BACK", BTN_SELECT},
        {"BTN_START", BTN_START},
        {"START", BTN_START},
        {"BTN_MODE", BTN_MODE},
        {"MODE", BTN_MODE},
    };
    auto it = kButtonCodes.find(name);
    if (it == kButtonCodes.end())
        throw std::runtime_error("Unsupported gamepad button name: " + raw_name);
    return it->second;
}

int yaml_get_button_code(const YAML::Node& node, const std::string& key, int fallback)
{
    if (!node[key])
        return fallback;
    return input_button_code_from_name(node[key].as<std::string>());
}

template <size_t N>
bool test_bit(const std::array<unsigned long, N>& bits, int bit)
{
    if (bit < 0)
        return false;
    const size_t index = static_cast<size_t>(bit) / (8 * sizeof(unsigned long));
    const size_t offset = static_cast<size_t>(bit) % (8 * sizeof(unsigned long));
    return index < bits.size() && ((bits[index] >> offset) & 1UL);
}

template <size_t N>
bool read_event_bits(int fd, int ev_type, std::array<unsigned long, N>& bits)
{
    bits.fill(0);
    return ioctl(fd, EVIOCGBIT(ev_type, sizeof(unsigned long) * bits.size()), bits.data()) >= 0;
}

std::filesystem::path resolve_relative(const std::filesystem::path& base_dir, const std::string& value)
{
    std::filesystem::path path(value);
    if (path.is_absolute())
        return path;
    return base_dir / path;
}

struct HttpEndpoint
{
    std::string host = "127.0.0.1";
    std::string port = "80";
    std::string path = "/";
};

HttpEndpoint parse_http_url(const std::string& url)
{
    static const std::regex pattern(R"(^http://([^/:]+)(?::([0-9]+))?(/.*)?$)");
    std::smatch match;
    if (!std::regex_match(url, match, pattern))
        throw std::runtime_error("Only http://host:port/path URLs are supported: " + url);

    HttpEndpoint endpoint;
    endpoint.host = match[1].str();
    endpoint.port = match[2].matched ? match[2].str() : "80";
    endpoint.path = match[3].matched ? match[3].str() : "/";
    return endpoint;
}

std::string http_get(const HttpEndpoint& endpoint, double timeout_s)
{
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* result = nullptr;
    int rc = getaddrinfo(endpoint.host.c_str(), endpoint.port.c_str(), &hints, &result);
    if (rc != 0)
        throw std::runtime_error(std::string("getaddrinfo failed: ") + gai_strerror(rc));

    int fd = -1;
    for (addrinfo* item = result; item != nullptr; item = item->ai_next)
    {
        fd = socket(item->ai_family, item->ai_socktype, item->ai_protocol);
        if (fd < 0)
            continue;

        timeval tv{};
        tv.tv_sec = static_cast<int>(timeout_s);
        tv.tv_usec = static_cast<int>((timeout_s - tv.tv_sec) * 1.0e6);
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        if (connect(fd, item->ai_addr, item->ai_addrlen) == 0)
            break;
        close(fd);
        fd = -1;
    }
    freeaddrinfo(result);

    if (fd < 0)
        throw std::runtime_error("Could not connect to " + endpoint.host + ":" + endpoint.port);

    std::ostringstream request;
    request << "GET " << endpoint.path << " HTTP/1.1\r\n"
            << "Host: " << endpoint.host << "\r\n"
            << "Connection: close\r\n\r\n";
    const std::string req = request.str();
    ssize_t sent = send(fd, req.data(), req.size(), 0);
    if (sent < 0)
    {
        close(fd);
        throw std::runtime_error("HTTP send failed");
    }

    std::string response;
    char buffer[4096];
    while (true)
    {
        ssize_t got = recv(fd, buffer, sizeof(buffer), 0);
        if (got > 0)
        {
            response.append(buffer, buffer + got);
            continue;
        }
        if (got == 0)
            break;
        close(fd);
        throw std::runtime_error("HTTP receive failed");
    }
    close(fd);

    const std::string sep = "\r\n\r\n";
    auto pos = response.find(sep);
    if (pos == std::string::npos)
        throw std::runtime_error("Malformed HTTP response");
    const std::string header = response.substr(0, pos);
    if (header.find(" 200 ") == std::string::npos)
        throw std::runtime_error("HTTP response was not 200 OK");
    return response.substr(pos + sep.size());
}

size_t find_matching_brace(const std::string& text, size_t open_pos, char open_ch, char close_ch)
{
    int depth = 0;
    bool in_string = false;
    bool escape = false;
    for (size_t i = open_pos; i < text.size(); ++i)
    {
        char c = text[i];
        if (escape)
        {
            escape = false;
            continue;
        }
        if (c == '\\')
        {
            escape = true;
            continue;
        }
        if (c == '"')
        {
            in_string = !in_string;
            continue;
        }
        if (in_string)
            continue;
        if (c == open_ch)
            depth++;
        else if (c == close_ch)
        {
            depth--;
            if (depth == 0)
                return i;
        }
    }
    return std::string::npos;
}

std::string json_object_for_key(const std::string& json, const std::string& key)
{
    const std::string needle = "\"" + key + "\"";
    auto key_pos = json.find(needle);
    if (key_pos == std::string::npos)
        return "{}";
    auto open_pos = json.find('{', key_pos + needle.size());
    if (open_pos == std::string::npos)
        return "{}";
    auto close_pos = find_matching_brace(json, open_pos, '{', '}');
    if (close_pos == std::string::npos)
        return "{}";
    return json.substr(open_pos, close_pos - open_pos + 1);
}

std::string json_array_for_key(const std::string& json, const std::string& key)
{
    const std::string needle = "\"" + key + "\"";
    auto key_pos = json.find(needle);
    if (key_pos == std::string::npos)
        return "[]";
    auto open_pos = json.find('[', key_pos + needle.size());
    if (open_pos == std::string::npos)
        return "[]";
    auto close_pos = find_matching_brace(json, open_pos, '[', ']');
    if (close_pos == std::string::npos)
        return "[]";
    return json.substr(open_pos, close_pos - open_pos + 1);
}

std::vector<std::string> json_objects_in_array(const std::string& array_json)
{
    std::vector<std::string> objects;
    size_t pos = 0;
    while (true)
    {
        auto open_pos = array_json.find('{', pos);
        if (open_pos == std::string::npos)
            break;
        auto close_pos = find_matching_brace(array_json, open_pos, '{', '}');
        if (close_pos == std::string::npos)
            break;
        objects.push_back(array_json.substr(open_pos, close_pos - open_pos + 1));
        pos = close_pos + 1;
    }
    return objects;
}

double json_number(const std::string& json, const std::string& key, double fallback = 0.0)
{
    const std::regex pattern("\"" + key + R"("\s*:\s*([-+0-9.eE]+))");
    std::smatch match;
    if (!std::regex_search(json, match, pattern))
        return fallback;
    return std::stod(match[1].str());
}

bool json_bool(const std::string& json, const std::string& key, bool fallback = false)
{
    const std::regex pattern("\"" + key + R"("\s*:\s*(true|false))");
    std::smatch match;
    if (!std::regex_search(json, match, pattern))
        return fallback;
    return match[1].str() == "true";
}

std::string json_string(const std::string& json, const std::string& key, const std::string& fallback = "")
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch match;
    if (!std::regex_search(json, match, pattern))
        return fallback;
    return match[1].str();
}

struct DeployConfig
{
    std::filesystem::path config_path;
    std::filesystem::path model_path;
    std::filesystem::path telemetry_path;
    std::string policy_output = "action";

    int obs_dim = 120;
    int history_length = 8;
    bool include_circle_state = false;

    std::string imu_url = "http://127.0.0.1:8080/api/data";
    std::string wheel_url = "http://127.0.0.1:8081/api/data";
    double request_timeout_s = 0.08;
    double max_data_age_s = 0.5;
    std::vector<std::string> wheel_order = {"front_left", "front_right", "rear_left", "rear_right"};
    std::string wheel_rpm_key = "fast_rpm";
    float max_free_spin_rpm = 2581.3785f;
    float wheel_rpm_clip_before_scale = 2.0f;
    float imu_mount_roll_rad = 0.0f;
    float imu_mount_pitch_rad = 0.0f;
    int imu_startup_calibration_samples = 30;
    float imu_gyro_z_sign = 1.0f;
    bool mask_absolute_yaw = false;

    float frequency_hz = 60.0f;
    float pwm_frequency_hz = 50.0f;
    float min_pulse_ms = 1.0f;
    float neutral_pulse_ms = 1.5f;
    float max_pulse_ms = 2.0f;
    float steering_limit = 1.0f;
    float throttle_limit = 1.0f;
    bool allow_reverse = false;
    float steering_sign = 1.0f;
    float throttle_sign = 1.0f;
    float max_steering_slew_per_s = 6.0f;
    float max_throttle_slew_per_s = 4.0f;
    bool gyro_assist_enabled = false;
    float gyro_assist_wheel_radius_m = 0.0515f;
    float gyro_assist_target_radius_m = 3.6f;
    float gyro_assist_direction = -1.0f;
    float gyro_assist_cutoff_hz = 8.0f;
    float gyro_assist_kp = 0.08f;
    float gyro_assist_max_correction = 0.12f;
    float gyro_assist_error_deadband_radps = 0.10f;
    float gyro_assist_activation_speed_mps = 0.8f;
    float gyro_assist_activation_blend_mps = 0.4f;
    float gyro_assist_correction_slew_per_s = 4.0f;
    float gyro_assist_max_target_yaw_rate_radps = 2.5f;
    bool launch_assist_enabled = false;
    double launch_assist_duration_s = 2.0;
    float launch_assist_throttle = 0.30f;
    float launch_assist_release_rpm = 80.0f;

    std::string steering_pwm = "32e0000.pwm:0";
    std::string throttle_pwm = "32c0000.pwm:0";
    bool steering_output_inverted = false;
    bool throttle_output_inverted = false;
    bool disable_pwm_on_exit = false;

    bool dry_run = true;
    bool real_output_enabled = false;
    bool stop_gamepad_service = true;
    bool restart_gamepad_service_on_exit = true;
    bool require_gamepad = true;
    std::string gamepad_device = "auto";
    std::vector<std::string> gamepad_name_contains = {
        "G5",
        "MACHENIKE",
        "Xbox",
        "Wireless Controller",
        "Gamepad",
        "Controller",
    };
    bool gamepad_grab = true;
    bool gamepad_require_arm = true;
    int gamepad_arm_button = BTN_SOUTH;
    int gamepad_pause_button = BTN_EAST;
    int gamepad_abort_button = BTN_START;
    int gamepad_deadman_button = BTN_TR;
    bool neutral_on_exit = true;
    double max_runtime_s = 16.0;
    double print_rate_hz = 2.0;
};

DeployConfig load_config(const std::filesystem::path& config_path)
{
    DeployConfig cfg;
    cfg.config_path = config_path;
    YAML::Node root = YAML::LoadFile(config_path.string());
    YAML::Node node = root["MJX_ET6/nio_lab/drift"] ? root["MJX_ET6/nio_lab/drift"] : root;
    const auto base_dir = config_path.parent_path();

    cfg.policy_output = yaml_get<std::string>(node, "policy_output", cfg.policy_output);
    cfg.obs_dim = yaml_get<int>(node, "obs_dim", cfg.obs_dim);
    cfg.history_length = yaml_get<int>(node, "history_length", cfg.history_length);
    cfg.include_circle_state = yaml_get<bool>(node, "include_circle_state", cfg.include_circle_state);

    cfg.imu_url = yaml_get<std::string>(node, "imu_url", cfg.imu_url);
    cfg.wheel_url = yaml_get<std::string>(node, "wheel_url", cfg.wheel_url);
    cfg.request_timeout_s = yaml_get<double>(node, "request_timeout_s", cfg.request_timeout_s);
    cfg.max_data_age_s = yaml_get<double>(node, "max_data_age_s", cfg.max_data_age_s);
    cfg.wheel_order = yaml_get<std::vector<std::string>>(node, "wheel_order", cfg.wheel_order);
    cfg.wheel_rpm_key = yaml_get<std::string>(node, "wheel_rpm_key", cfg.wheel_rpm_key);
    cfg.max_free_spin_rpm = yaml_get<float>(node, "max_free_spin_rpm", cfg.max_free_spin_rpm);
    cfg.wheel_rpm_clip_before_scale = yaml_get<float>(
        node, "wheel_rpm_clip_before_scale", cfg.wheel_rpm_clip_before_scale);
    cfg.imu_mount_roll_rad = yaml_get<float>(node, "imu_mount_roll_rad", cfg.imu_mount_roll_rad);
    cfg.imu_mount_pitch_rad = yaml_get<float>(node, "imu_mount_pitch_rad", cfg.imu_mount_pitch_rad);
    cfg.imu_startup_calibration_samples = yaml_get<int>(
        node, "imu_startup_calibration_samples", cfg.imu_startup_calibration_samples);
    cfg.imu_gyro_z_sign = yaml_get<float>(node, "imu_gyro_z_sign", cfg.imu_gyro_z_sign);
    cfg.mask_absolute_yaw = yaml_get<bool>(node, "mask_absolute_yaw", cfg.mask_absolute_yaw);

    cfg.frequency_hz = yaml_get<float>(node, "frequency_hz", cfg.frequency_hz);
    cfg.pwm_frequency_hz = yaml_get<float>(node, "pwm_frequency_hz", cfg.pwm_frequency_hz);
    cfg.min_pulse_ms = yaml_get<float>(node, "min_pulse_ms", cfg.min_pulse_ms);
    cfg.neutral_pulse_ms = yaml_get<float>(node, "neutral_pulse_ms", cfg.neutral_pulse_ms);
    cfg.max_pulse_ms = yaml_get<float>(node, "max_pulse_ms", cfg.max_pulse_ms);
    cfg.steering_limit = yaml_get<float>(node, "steering_limit", cfg.steering_limit);
    cfg.throttle_limit = yaml_get<float>(node, "throttle_limit", cfg.throttle_limit);
    cfg.allow_reverse = yaml_get<bool>(node, "allow_reverse", cfg.allow_reverse);
    cfg.steering_sign = yaml_get<float>(node, "steering_sign", cfg.steering_sign);
    cfg.throttle_sign = yaml_get<float>(node, "throttle_sign", cfg.throttle_sign);
    cfg.max_steering_slew_per_s = yaml_get<float>(node, "max_steering_slew_per_s", cfg.max_steering_slew_per_s);
    cfg.max_throttle_slew_per_s = yaml_get<float>(node, "max_throttle_slew_per_s", cfg.max_throttle_slew_per_s);
    cfg.gyro_assist_enabled = yaml_get<bool>(node, "gyro_assist_enabled", cfg.gyro_assist_enabled);
    cfg.gyro_assist_wheel_radius_m = yaml_get<float>(
        node, "gyro_assist_wheel_radius_m", cfg.gyro_assist_wheel_radius_m);
    cfg.gyro_assist_target_radius_m = yaml_get<float>(
        node, "gyro_assist_target_radius_m", cfg.gyro_assist_target_radius_m);
    cfg.gyro_assist_direction = yaml_get<float>(
        node, "gyro_assist_direction", cfg.gyro_assist_direction);
    cfg.gyro_assist_cutoff_hz = yaml_get<float>(
        node, "gyro_assist_cutoff_hz", cfg.gyro_assist_cutoff_hz);
    cfg.gyro_assist_kp = yaml_get<float>(node, "gyro_assist_kp", cfg.gyro_assist_kp);
    cfg.gyro_assist_max_correction = yaml_get<float>(
        node, "gyro_assist_max_correction", cfg.gyro_assist_max_correction);
    cfg.gyro_assist_error_deadband_radps = yaml_get<float>(
        node,
        "gyro_assist_error_deadband_radps",
        cfg.gyro_assist_error_deadband_radps);
    cfg.gyro_assist_activation_speed_mps = yaml_get<float>(
        node, "gyro_assist_activation_speed_mps", cfg.gyro_assist_activation_speed_mps);
    cfg.gyro_assist_activation_blend_mps = yaml_get<float>(
        node, "gyro_assist_activation_blend_mps", cfg.gyro_assist_activation_blend_mps);
    cfg.gyro_assist_correction_slew_per_s = yaml_get<float>(
        node,
        "gyro_assist_correction_slew_per_s",
        cfg.gyro_assist_correction_slew_per_s);
    cfg.gyro_assist_max_target_yaw_rate_radps = yaml_get<float>(
        node,
        "gyro_assist_max_target_yaw_rate_radps",
        cfg.gyro_assist_max_target_yaw_rate_radps);
    cfg.launch_assist_enabled = yaml_get<bool>(node, "launch_assist_enabled", cfg.launch_assist_enabled);
    cfg.launch_assist_duration_s = yaml_get<double>(node, "launch_assist_duration_s", cfg.launch_assist_duration_s);
    cfg.launch_assist_throttle = yaml_get<float>(node, "launch_assist_throttle", cfg.launch_assist_throttle);
    cfg.launch_assist_release_rpm = yaml_get<float>(node, "launch_assist_release_rpm", cfg.launch_assist_release_rpm);

    cfg.steering_pwm = yaml_get<std::string>(node, "steering_pwm", cfg.steering_pwm);
    cfg.throttle_pwm = yaml_get<std::string>(node, "throttle_pwm", cfg.throttle_pwm);
    cfg.steering_output_inverted = yaml_get<bool>(node, "steering_output_inverted", cfg.steering_output_inverted);
    cfg.throttle_output_inverted = yaml_get<bool>(node, "throttle_output_inverted", cfg.throttle_output_inverted);
    cfg.disable_pwm_on_exit = yaml_get<bool>(node, "disable_pwm_on_exit", cfg.disable_pwm_on_exit);

    cfg.real_output_enabled = yaml_get<bool>(node, "real_output_enabled", cfg.real_output_enabled);
    cfg.stop_gamepad_service = yaml_get<bool>(node, "stop_gamepad_service", cfg.stop_gamepad_service);
    cfg.restart_gamepad_service_on_exit = yaml_get<bool>(node, "restart_gamepad_service_on_exit", cfg.restart_gamepad_service_on_exit);
    cfg.require_gamepad = yaml_get<bool>(node, "require_gamepad", cfg.require_gamepad);
    cfg.gamepad_device = yaml_get<std::string>(node, "gamepad_device", cfg.gamepad_device);
    cfg.gamepad_name_contains = yaml_get<std::vector<std::string>>(
        node, "gamepad_name_contains", cfg.gamepad_name_contains);
    cfg.gamepad_grab = yaml_get<bool>(node, "gamepad_grab", cfg.gamepad_grab);
    cfg.gamepad_require_arm = yaml_get<bool>(
        node, "gamepad_require_arm", cfg.gamepad_require_arm);
    cfg.gamepad_arm_button = yaml_get_button_code(node, "gamepad_arm_button", cfg.gamepad_arm_button);
    cfg.gamepad_pause_button = yaml_get_button_code(node, "gamepad_pause_button", cfg.gamepad_pause_button);
    cfg.gamepad_abort_button = yaml_get_button_code(node, "gamepad_abort_button", cfg.gamepad_abort_button);
    cfg.gamepad_deadman_button = yaml_get_button_code(node, "gamepad_deadman_button", cfg.gamepad_deadman_button);
    cfg.neutral_on_exit = yaml_get<bool>(node, "neutral_on_exit", cfg.neutral_on_exit);
    cfg.max_runtime_s = yaml_get<double>(node, "max_runtime_s", cfg.max_runtime_s);
    cfg.print_rate_hz = yaml_get<double>(node, "print_rate_hz", cfg.print_rate_hz);

    std::string model_name = yaml_get<std::string>(node, "model_name", "policy.onnx");
    cfg.model_path = resolve_relative(base_dir, model_name);

    std::string telemetry_csv = yaml_get<std::string>(node, "telemetry_csv", "policy/MJX_ET6/rl_log_mjx_et6.csv");
    std::filesystem::path telemetry(telemetry_csv);
    if (telemetry.is_absolute())
        cfg.telemetry_path = telemetry;
    else
        cfg.telemetry_path = std::filesystem::path(POLICY_DIR).parent_path() / telemetry;

    if (cfg.frequency_hz <= 0.0f)
        throw std::runtime_error("frequency_hz must be positive");
    if (cfg.pwm_frequency_hz <= 0.0f)
        throw std::runtime_error("pwm_frequency_hz must be positive");
    if (cfg.wheel_rpm_clip_before_scale <= 0.0f)
        throw std::runtime_error("wheel_rpm_clip_before_scale must be positive");
    if (cfg.imu_startup_calibration_samples < 0 || cfg.imu_startup_calibration_samples > 300)
        throw std::runtime_error("imu_startup_calibration_samples must be in [0, 300]");
    if (cfg.gyro_assist_enabled)
    {
        if (cfg.gyro_assist_wheel_radius_m <= 0.0f || cfg.gyro_assist_target_radius_m <= 0.0f)
            throw std::runtime_error("gyro assist wheel and target radii must be positive");
        if (cfg.gyro_assist_cutoff_hz <= 0.0f || cfg.gyro_assist_kp < 0.0f)
            throw std::runtime_error("gyro assist cutoff must be positive and kp non-negative");
        if (cfg.gyro_assist_max_correction < 0.0f || cfg.gyro_assist_max_correction > 1.0f)
            throw std::runtime_error("gyro_assist_max_correction must be in [0, 1]");
        if (cfg.gyro_assist_activation_blend_mps <= 0.0f)
            throw std::runtime_error("gyro_assist_activation_blend_mps must be positive");
        if (cfg.gyro_assist_max_target_yaw_rate_radps <= 0.0f)
            throw std::runtime_error("gyro_assist_max_target_yaw_rate_radps must be positive");
    }

    const int frame_dim = 9 + 4 + 2 + (cfg.include_circle_state ? 5 : 0);
    const int implied_obs_dim = frame_dim * cfg.history_length;
    if (cfg.obs_dim != implied_obs_dim)
    {
        throw std::runtime_error("obs_dim does not match config: obs_dim=" + std::to_string(cfg.obs_dim) +
                                 " implied=" + std::to_string(implied_obs_dim));
    }
    if (cfg.real_output_enabled)
    {
        const float expected_rpm_clip = 2.0f * cfg.max_free_spin_rpm;
        const bool direct_pwm_output =
            cfg.policy_output == "pwm_ms" || cfg.policy_output == "pwm" || cfg.policy_output == "ms";
        if (!direct_pwm_output || cfg.obs_dim != 120 || cfg.history_length != 8 || cfg.include_circle_state)
            throw std::runtime_error("Real output requires the deployable 120-input, 8-frame direct-PWM policy contract");
        const std::vector<std::string> expected_wheel_order = {
            "front_left", "front_right", "rear_left", "rear_right"};
        if (cfg.wheel_order != expected_wheel_order)
            throw std::runtime_error("Real output requires wheel order front_left, front_right, rear_left, rear_right");
        if (!cfg.mask_absolute_yaw || std::abs(cfg.imu_gyro_z_sign + 1.0f) > 1.0e-4f)
            throw std::runtime_error("Real output requires masked absolute yaw and imu_gyro_z_sign=-1");
        if (std::abs(cfg.frequency_hz - 60.0f) > 1.0e-4f)
            throw std::runtime_error("Real output requires the 60 Hz training policy rate");
        if (!cfg.gyro_assist_enabled)
            throw std::runtime_error("Real output requires the training-matched gyro residual assist");
        if (std::abs(cfg.wheel_rpm_clip_before_scale - expected_rpm_clip) > 1.0e-3f)
            throw std::runtime_error("Real output requires wheel RPM clip-before-scale = 2 * max_free_spin_rpm");
        if (cfg.allow_reverse || cfg.launch_assist_enabled)
            throw std::runtime_error("Real output requires reverse and launch assist to remain disabled");
    }
    return cfg;
}

std::vector<std::string> parse_csv_row(const std::string& line)
{
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (size_t i = 0; i < line.size(); ++i)
    {
        const char c = line[i];
        if (c == '"')
        {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"')
            {
                field.push_back('"');
                ++i;
            }
            else
            {
                quoted = !quoted;
            }
        }
        else if (c == ',' && !quoted)
        {
            fields.push_back(field);
            field.clear();
        }
        else if (c != '\r')
        {
            field.push_back(c);
        }
    }
    if (quoted)
        throw std::runtime_error("Unterminated quoted CSV field");
    fields.push_back(field);
    return fields;
}

struct PwmReplaySample
{
    size_t sequence_index = 0;
    size_t episode_index = 0;
    size_t episode_command_index = 0;
    bool done = false;
    double dt_s = 0.0;
    float steering_pwm_ms = 1.5f;
    float throttle_pwm_ms = 1.5f;
    float steering_action_raw = 0.0f;
    float throttle_action_raw = 0.0f;
};

class PwmReplay
{
public:
    PwmReplay(const std::filesystem::path& path, size_t selected_episode, const DeployConfig& cfg)
        : path_(path), selected_episode_(selected_episode)
    {
        std::ifstream file(path_);
        if (!file.is_open())
            throw std::runtime_error("Could not open PWM replay CSV: " + path_.string());

        std::string line;
        if (!std::getline(file, line))
            throw std::runtime_error("PWM replay CSV is empty: " + path_.string());
        const auto headers = parse_csv_row(line);
        std::map<std::string, size_t> columns;
        for (size_t i = 0; i < headers.size(); ++i)
            columns[headers[i]] = i;

        const std::vector<std::string> required = {
            "sequence_index",
            "dt_s",
            "episode_index",
            "episode_command_index",
            "done",
            "steering_pwm_ms",
            "throttle_pwm_ms",
        };
        for (const std::string& name : required)
        {
            if (columns.find(name) == columns.end())
                throw std::runtime_error("PWM replay CSV is missing required column: " + name);
        }

        const auto value = [&](const std::vector<std::string>& row, const std::string& name) -> const std::string& {
            const size_t index = columns.at(name);
            if (index >= row.size())
                throw std::runtime_error("Short PWM replay CSV row while reading: " + name);
            return row[index];
        };
        const bool has_steering_raw = columns.find("steering_action_raw") != columns.end();
        const bool has_throttle_raw = columns.find("throttle_action_raw") != columns.end();

        size_t line_number = 1;
        while (std::getline(file, line))
        {
            ++line_number;
            if (line.empty())
                continue;
            try
            {
                const auto row = parse_csv_row(line);
                const size_t episode = static_cast<size_t>(std::stoull(value(row, "episode_index")));
                if (episode != selected_episode_)
                    continue;

                PwmReplaySample sample;
                sample.sequence_index = static_cast<size_t>(std::stoull(value(row, "sequence_index")));
                sample.episode_index = episode;
                sample.episode_command_index = static_cast<size_t>(std::stoull(value(row, "episode_command_index")));
                sample.done = std::stoi(value(row, "done")) != 0;
                sample.dt_s = std::stod(value(row, "dt_s"));
                sample.steering_pwm_ms = std::stof(value(row, "steering_pwm_ms"));
                sample.throttle_pwm_ms = std::stof(value(row, "throttle_pwm_ms"));
                if (has_steering_raw)
                    sample.steering_action_raw = std::stof(value(row, "steering_action_raw"));
                else
                    sample.steering_action_raw = pulse_to_signed(
                        sample.steering_pwm_ms, cfg.min_pulse_ms, cfg.neutral_pulse_ms, cfg.max_pulse_ms);
                if (has_throttle_raw)
                    sample.throttle_action_raw = std::stof(value(row, "throttle_action_raw"));
                else
                    sample.throttle_action_raw = pulse_to_signed(
                        sample.throttle_pwm_ms, cfg.min_pulse_ms, cfg.neutral_pulse_ms, cfg.max_pulse_ms);
                samples_.push_back(sample);
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error(
                    "Invalid PWM replay CSV row " + std::to_string(line_number) + ": " + e.what());
            }
        }

        if (samples_.empty())
            throw std::runtime_error("PWM replay CSV has no rows for episode " + std::to_string(selected_episode_));

        const double target_dt_s = 1.0 / std::max(cfg.frequency_hz, 1.0f);
        const double dt_tolerance_s = std::max(1.0e-4, target_dt_s * 0.02);
        const float pulse_tolerance_ms = 1.0e-4f;
        for (size_t i = 0; i < samples_.size(); ++i)
        {
            const auto& sample = samples_[i];
            if (!std::isfinite(sample.dt_s) || std::abs(sample.dt_s - target_dt_s) > dt_tolerance_s)
            {
                throw std::runtime_error(
                    "PWM replay dt does not match the configured control frequency at command " +
                    std::to_string(i));
            }
            if (sample.episode_command_index != i)
                throw std::runtime_error("PWM replay episode_command_index is not contiguous");
            if (i > 0 && sample.sequence_index != samples_[i - 1].sequence_index + 1)
                throw std::runtime_error("PWM replay sequence_index is not contiguous");
            if (!std::isfinite(sample.steering_pwm_ms) ||
                sample.steering_pwm_ms < cfg.min_pulse_ms - pulse_tolerance_ms ||
                sample.steering_pwm_ms > cfg.max_pulse_ms + pulse_tolerance_ms)
            {
                throw std::runtime_error("Steering PWM is outside the calibrated range");
            }
            if (!std::isfinite(sample.throttle_pwm_ms) ||
                sample.throttle_pwm_ms < cfg.min_pulse_ms - pulse_tolerance_ms ||
                sample.throttle_pwm_ms > cfg.max_pulse_ms + pulse_tolerance_ms)
            {
                throw std::runtime_error("Throttle PWM is outside the calibrated range");
            }
            if (!cfg.allow_reverse && sample.throttle_pwm_ms < cfg.neutral_pulse_ms - pulse_tolerance_ms)
                throw std::runtime_error("PWM replay contains reverse/brake throttle while allow_reverse is false");
            if (sample.done && i + 1 != samples_.size())
                throw std::runtime_error("PWM replay episode contains commands after done=1");
        }
        if (!samples_.back().done)
            throw std::runtime_error("Selected PWM replay episode is incomplete: final row does not have done=1");
    }

    const PwmReplaySample& sample(size_t index) const
    {
        return samples_.at(index);
    }

    size_t size() const
    {
        return samples_.size();
    }

    double duration_s() const
    {
        double duration = 0.0;
        for (const auto& sample : samples_)
            duration += sample.dt_s;
        return duration;
    }

    const std::filesystem::path& path() const
    {
        return path_;
    }

    size_t episode() const
    {
        return selected_episode_;
    }

private:
    std::filesystem::path path_;
    size_t selected_episode_ = 0;
    std::vector<PwmReplaySample> samples_;
};

struct GyroAssistSample
{
    bool active = false;
    float policy_steering = 0.0f;
    float front_speed_mps = 0.0f;
    float measured_yaw_rate_radps = 0.0f;
    float filtered_yaw_rate_radps = 0.0f;
    float target_yaw_rate_radps = 0.0f;
    float yaw_rate_error_radps = 0.0f;
    float correction = 0.0f;
    float final_steering = 0.0f;
};

class CsvLogger
{
public:
    explicit CsvLogger(const std::filesystem::path& path)
    {
        path_ = unique_csv_path(path);
        if (!path_.parent_path().empty())
            std::filesystem::create_directories(path_.parent_path());
        file_.open(path_, std::ios::out | std::ios::trunc);
        if (!file_.is_open())
            throw std::runtime_error("Could not open telemetry CSV: " + path_.string());
        std::cout << LOGGER::INFO << "Telemetry CSV: " << path_ << std::endl;
        file_ << "time_iso,dt_s,sensor_ok,policy_ok,safe_reason,control_source,"
              << "replay_episode,replay_command_index,replay_sequence_index,"
              << "policy_raw_0,policy_raw_1,"
              << "policy_steering_action,policy_throttle_action,"
              << "steering_action,throttle_action,steering_pwm_ms,throttle_pwm_ms,"
              << "gyro_assist_active,gyro_assist_front_speed_mps,"
              << "gyro_assist_measured_yaw_rate_radps,gyro_assist_filtered_yaw_rate_radps,"
              << "gyro_assist_target_yaw_rate_radps,gyro_assist_yaw_rate_error_radps,"
              << "gyro_assist_correction,gyro_assist_final_steering,"
              << "imu_accel_x,imu_accel_y,imu_accel_z,imu_gyro_x,imu_gyro_y,imu_gyro_z,"
              << "imu_roll_rad,imu_pitch_rad,imu_yaw_rad,"
              << "front_left_rpm,front_right_rpm,rear_left_rpm,rear_right_rpm\n";
    }

    void write(
        double dt_s,
        bool sensor_ok,
        bool policy_ok,
        const std::string& reason,
        const std::string& control_source,
        long long replay_episode,
        long long replay_command_index,
        long long replay_sequence_index,
        float policy_raw_0,
        float policy_raw_1,
        float policy_steering_action,
        float policy_throttle_action,
        float steering_action,
        float throttle_action,
        float steering_pwm_ms,
        float throttle_pwm_ms,
        const GyroAssistSample& gyro_assist,
        const std::vector<float>& imu_obs,
        const std::vector<float>& rpm)
    {
        file_ << now_iso() << ","
              << dt_s << ","
              << (sensor_ok ? 1 : 0) << ","
              << (policy_ok ? 1 : 0) << ","
              << "\"" << reason << "\","
              << control_source << ","
              << replay_episode << "," << replay_command_index << "," << replay_sequence_index << ","
              << policy_raw_0 << "," << policy_raw_1 << ","
              << policy_steering_action << "," << policy_throttle_action << ","
              << steering_action << "," << throttle_action << ","
              << steering_pwm_ms << "," << throttle_pwm_ms << ","
              << (gyro_assist.active ? 1 : 0) << ","
              << gyro_assist.front_speed_mps << ","
              << gyro_assist.measured_yaw_rate_radps << ","
              << gyro_assist.filtered_yaw_rate_radps << ","
              << gyro_assist.target_yaw_rate_radps << ","
              << gyro_assist.yaw_rate_error_radps << ","
              << gyro_assist.correction << ","
              << gyro_assist.final_steering;
        for (int i = 0; i < 9; ++i)
            file_ << "," << (i < static_cast<int>(imu_obs.size()) ? imu_obs[i] : 0.0f);
        for (int i = 0; i < 4; ++i)
            file_ << "," << (i < static_cast<int>(rpm.size()) ? rpm[i] : 0.0f);
        file_ << "\n";
        if (++rows_ % 20 == 0)
            file_.flush();
    }

    const std::filesystem::path& path() const
    {
        return path_;
    }

private:
    std::filesystem::path path_;
    std::ofstream file_;
    int rows_ = 0;
};

class SensorClient
{
public:
    explicit SensorClient(const DeployConfig& cfg)
        : cfg_(cfg),
          imu_endpoint_(parse_http_url(cfg.imu_url)),
          wheel_endpoint_(parse_http_url(cfg.wheel_url))
    {
    }

    void calibrate_startup()
    {
        if (cfg_.imu_startup_calibration_samples == 0)
            return;

        startup_accel_bias_.fill(0.0f);
        startup_gyro_bias_.fill(0.0f);
        startup_attitude_bias_.fill(0.0f);
        std::array<std::vector<float>, 3> accel_samples;
        std::array<std::vector<float>, 3> gyro_samples;
        std::array<std::vector<float>, 2> attitude_samples;
        std::vector<float> imu_obs;
        std::vector<float> wheel_obs;
        std::vector<float> raw_rpm;
        float max_wheel_rpm = 0.0f;
        for (int i = 0; i < cfg_.imu_startup_calibration_samples; ++i)
        {
            read(imu_obs, wheel_obs, raw_rpm);
            for (int axis = 0; axis < 3; ++axis)
            {
                accel_samples[axis].push_back(imu_obs[axis]);
                gyro_samples[axis].push_back(imu_obs[3 + axis]);
            }
            attitude_samples[0].push_back(imu_obs[6]);
            attitude_samples[1].push_back(imu_obs[7]);
            for (float rpm : raw_rpm)
                max_wheel_rpm = std::max(max_wheel_rpm, std::abs(rpm));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::array<float, 3> accel_median{};
        std::array<float, 3> gyro_median{};
        for (int axis = 0; axis < 3; ++axis)
        {
            accel_median[axis] = median_value(accel_samples[axis]);
            gyro_median[axis] = median_value(gyro_samples[axis]);
        }
        const float gravity_magnitude = std::sqrt(
            accel_median[0] * accel_median[0] +
            accel_median[1] * accel_median[1] +
            accel_median[2] * accel_median[2]);
        float max_abs_gyro = 0.0f;
        float max_accel_span = 0.0f;
        for (int axis = 0; axis < 3; ++axis)
        {
            for (float value : gyro_samples[axis])
                max_abs_gyro = std::max(max_abs_gyro, std::abs(value));
            const auto minmax = std::minmax_element(accel_samples[axis].begin(), accel_samples[axis].end());
            max_accel_span = std::max(max_accel_span, *minmax.second - *minmax.first);
        }
        if (
            gravity_magnitude < 8.0f || gravity_magnitude > 11.5f ||
            max_abs_gyro > 0.15f || max_accel_span > 1.5f || max_wheel_rpm > 20.0f)
        {
            throw std::runtime_error(
                "IMU startup calibration rejected: keep the car stationary before launch");
        }

        startup_accel_bias_[0] = accel_median[0];
        startup_accel_bias_[1] = accel_median[1];
        startup_accel_bias_[2] = accel_median[2] + 9.81f;
        startup_gyro_bias_ = gyro_median;
        startup_attitude_bias_[0] = median_value(attitude_samples[0]);
        startup_attitude_bias_[1] = median_value(attitude_samples[1]);
        std::cout << LOGGER::INFO << "IMU startup calibration: accel_bias=("
                  << startup_accel_bias_[0] << ", " << startup_accel_bias_[1] << ", "
                  << startup_accel_bias_[2] << ") gyro_bias=("
                  << startup_gyro_bias_[0] << ", " << startup_gyro_bias_[1] << ", "
                  << startup_gyro_bias_[2] << ") roll/pitch_bias=("
                  << startup_attitude_bias_[0] << ", " << startup_attitude_bias_[1] << ")"
                  << std::endl;
    }

    void read(std::vector<float>& imu_obs, std::vector<float>& wheel_obs, std::vector<float>& raw_rpm)
    {
        const std::string imu = http_get(imu_endpoint_, cfg_.request_timeout_s);
        const std::string wheel = http_get(wheel_endpoint_, cfg_.request_timeout_s);

        const bool online = json_bool(imu, "online", false);
        const double age_s = json_number(imu, "data_age_ms", 0.0) / 1000.0;
        if (!online || age_s > cfg_.max_data_age_s)
            throw std::runtime_error("IMU stale/offline");

        const std::string acc_obj = json_object_for_key(imu, "acceleration");
        const std::string gyro_obj = json_object_for_key(imu, "gyro");
        const std::string attitude_obj = json_object_for_key(imu, "attitude");
        std::vector<float> acc = {
            static_cast<float>(json_number(acc_obj, "x")),
            static_cast<float>(json_number(acc_obj, "y")),
            static_cast<float>(json_number(acc_obj, "z")),
        };
        std::vector<float> gyro = {
            static_cast<float>(json_number(gyro_obj, "x")),
            static_cast<float>(json_number(gyro_obj, "y")),
            static_cast<float>(json_number(gyro_obj, "z")),
        };
        std::string gyro_unit = json_string(imu, "gyro_unit", "rad/s");
        if (gyro_unit.find("deg") != std::string::npos)
        {
            constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
            for (float& value : gyro)
                value *= kDegToRad;
        }
        acc = rotate_sensor_to_body(acc);
        gyro = rotate_sensor_to_body(gyro);
        gyro[2] *= cfg_.imu_gyro_z_sign;
        constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
        std::vector<float> attitude = {
            normalize_degrees(static_cast<float>(json_number(attitude_obj, "roll"))) * kDegToRad -
                cfg_.imu_mount_roll_rad,
            normalize_degrees(static_cast<float>(json_number(attitude_obj, "pitch"))) * kDegToRad -
                cfg_.imu_mount_pitch_rad,
            normalize_degrees(static_cast<float>(json_number(attitude_obj, "yaw"))) * kDegToRad,
        };
        for (int axis = 0; axis < 3; ++axis)
        {
            acc[axis] -= startup_accel_bias_[axis];
            gyro[axis] -= startup_gyro_bias_[axis];
        }
        attitude[0] -= startup_attitude_bias_[0];
        attitude[1] -= startup_attitude_bias_[1];

        imu_obs.clear();
        imu_obs.reserve(9);
        for (float value : acc)
            imu_obs.push_back(clampf(value, -30.0f, 30.0f));
        for (float value : gyro)
            imu_obs.push_back(clampf(value, -30.0f, 30.0f));
        for (float value : attitude)
            imu_obs.push_back(clampf(value, -30.0f, 30.0f));

        const std::string channels_array = json_array_for_key(wheel, "channels");
        const auto channel_objects = json_objects_in_array(channels_array);
        std::map<std::string, std::string> channels;
        for (const std::string& object : channel_objects)
        {
            std::string name = json_string(object, "name", "");
            if (!name.empty())
                channels[name] = object;
        }

        raw_rpm.clear();
        wheel_obs.clear();
        for (const std::string& name : cfg_.wheel_order)
        {
            float rpm = 0.0f;
            auto it = channels.find(name);
            if (it != channels.end())
            {
                rpm = static_cast<float>(json_number(it->second, cfg_.wheel_rpm_key, json_number(it->second, "rpm", 0.0)));
            }
            rpm = std::max(0.0f, rpm);
            raw_rpm.push_back(rpm);
            // Isaac Lab ObservationTerm applies raw clip before scale.
            const float clipped_rpm = clampf(rpm, 0.0f, cfg_.wheel_rpm_clip_before_scale);
            wheel_obs.push_back(clipped_rpm / std::max(cfg_.max_free_spin_rpm, 1.0f));
        }
    }

private:
    std::vector<float> rotate_sensor_to_body(const std::vector<float>& vector) const
    {
        if (vector.size() != 3)
            throw std::runtime_error("IMU vector must contain exactly three values");

        // R_body_sensor = Ry(mount_pitch) * Rx(mount_roll). The calibrated
        // level-car gravity vector is therefore mapped back to [0, 0, -g].
        const float cr = std::cos(cfg_.imu_mount_roll_rad);
        const float sr = std::sin(cfg_.imu_mount_roll_rad);
        const float cp = std::cos(cfg_.imu_mount_pitch_rad);
        const float sp = std::sin(cfg_.imu_mount_pitch_rad);
        const float rx_x = vector[0];
        const float rx_y = cr * vector[1] - sr * vector[2];
        const float rx_z = sr * vector[1] + cr * vector[2];
        return {
            cp * rx_x + sp * rx_z,
            rx_y,
            -sp * rx_x + cp * rx_z,
        };
    }

    DeployConfig cfg_;
    HttpEndpoint imu_endpoint_;
    HttpEndpoint wheel_endpoint_;
    std::array<float, 3> startup_accel_bias_{};
    std::array<float, 3> startup_gyro_bias_{};
    std::array<float, 2> startup_attitude_bias_{};
};

class ObservationHistory
{
public:
    explicit ObservationHistory(const DeployConfig& cfg)
        : cfg_(cfg)
    {
        frame_dim_ = 9 + 4 + 2 + (cfg_.include_circle_state ? 5 : 0);
    }

    std::vector<float> update(
        const std::vector<float>& imu_obs,
        const std::vector<float>& wheel_obs,
        float last_steering_action,
        float last_throttle_action)
    {
        std::vector<float> frame;
        frame.reserve(frame_dim_);
        std::vector<float> policy_imu = imu_obs;
        if (cfg_.mask_absolute_yaw && policy_imu.size() >= 9)
            policy_imu[8] = 0.0f;
        frame.insert(frame.end(), policy_imu.begin(), policy_imu.end());
        frame.insert(frame.end(), wheel_obs.begin(), wheel_obs.end());
        frame.push_back(clampf(last_steering_action, -1.0f, 1.0f));
        frame.push_back(clampf(last_throttle_action, -1.0f, 1.0f));
        if (cfg_.include_circle_state)
        {
            frame.insert(frame.end(), 5, 0.0f);
        }
        if (static_cast<int>(frame.size()) != frame_dim_)
            throw std::runtime_error("Observation frame dimension mismatch");

        if (frames_.empty())
        {
            for (int i = 0; i < cfg_.history_length; ++i)
                frames_.push_back(frame);
        }
        else
        {
            frames_.push_back(frame);
            while (static_cast<int>(frames_.size()) > cfg_.history_length)
                frames_.pop_front();
        }

        // Isaac Lab concatenates observation-term histories term-by-term:
        // IMU[8x9], wheel RPM[8x4], previous action[8x2], optional circle state.
        // Frame-major flattening has the same size but feeds every feature to
        // the wrong ONNX input index.
        std::vector<float> obs;
        obs.reserve(cfg_.obs_dim);
        for (const auto& item : frames_)
            obs.insert(obs.end(), item.begin(), item.begin() + 9);
        for (const auto& item : frames_)
            obs.insert(obs.end(), item.begin() + 9, item.begin() + 13);
        for (const auto& item : frames_)
            obs.insert(obs.end(), item.begin() + 13, item.begin() + 15);
        if (cfg_.include_circle_state)
        {
            for (const auto& item : frames_)
                obs.insert(obs.end(), item.begin() + 15, item.end());
        }
        if (static_cast<int>(obs.size()) != cfg_.obs_dim)
            throw std::runtime_error("Observation history dimension mismatch");
        return obs;
    }

private:
    DeployConfig cfg_;
    int frame_dim_ = 15;
    std::deque<std::vector<float>> frames_;
};

class PwmChannel
{
public:
    PwmChannel(std::string spec, std::string label, bool inverted)
        : spec_(std::move(spec)), label_(std::move(label)), inverted_(inverted)
    {
    }

    void start(float frequency_hz, float neutral_ms, bool dry_run)
    {
        dry_run_ = dry_run;
        period_ns_ = static_cast<int>(std::round(1.0e9f / frequency_hz));
        if (dry_run_)
        {
            last_ms_ = neutral_ms;
            std::cout << LOGGER::INFO << "[DRY PWM] " << label_ << " neutral=" << neutral_ms << " ms" << std::endl;
            return;
        }

        resolve();
        if (!std::filesystem::exists(pwm_path_))
        {
            write_text(chip_path_ / "export", std::to_string(index_));
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            while (!std::filesystem::exists(pwm_path_) && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!std::filesystem::exists(pwm_path_))
            throw std::runtime_error("PWM export failed for " + label_);

        try_write(pwm_path_ / "enable", "0");
        write_text(pwm_path_ / "period", std::to_string(period_ns_));
        set_pulse_ms(neutral_ms);
        write_text(pwm_path_ / "enable", "1");
        std::cout << LOGGER::INFO << "PWM " << label_ << " -> " << chip_path_ << "/pwm" << index_ << std::endl;
    }

    void set_pulse_ms(float pulse_ms)
    {
        last_ms_ = pulse_ms;
        if (dry_run_)
            return;
        int duty = static_cast<int>(std::round(pulse_ms * 1.0e6f));
        duty = std::max(0, std::min(period_ns_, duty));
        if (inverted_)
            duty = period_ns_ - duty;
        write_text(pwm_path_ / "duty_cycle", std::to_string(duty));
    }

    void close(float neutral_ms, bool disable)
    {
        try
        {
            set_pulse_ms(neutral_ms);
            if (!dry_run_ && disable)
                try_write(pwm_path_ / "enable", "0");
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::WARNING << "Could not close PWM " << label_ << ": " << e.what() << std::endl;
        }
    }

private:
    void resolve()
    {
        std::string chip_spec = spec_;
        index_ = 0;
        auto colon = spec_.rfind(':');
        if (colon != std::string::npos)
        {
            chip_spec = spec_.substr(0, colon);
            index_ = std::stoi(spec_.substr(colon + 1));
        }
        if (chip_spec.rfind("/sys/class/pwm/", 0) == 0)
        {
            chip_path_ = chip_spec;
        }
        else if (chip_spec.rfind("pwmchip", 0) == 0)
        {
            chip_path_ = std::filesystem::path("/sys/class/pwm") / chip_spec;
        }
        else
        {
            for (const auto& entry : std::filesystem::directory_iterator("/sys/class/pwm"))
            {
                if (!entry.is_directory() && !entry.is_symlink())
                    continue;
                std::error_code ec;
                auto target = std::filesystem::canonical(entry.path(), ec);
                if (!ec && target.string().find(chip_spec) != std::string::npos)
                {
                    chip_path_ = entry.path();
                    break;
                }
            }
        }
        if (chip_path_.empty())
            throw std::runtime_error("Could not resolve PWM spec: " + spec_);
        pwm_path_ = chip_path_ / ("pwm" + std::to_string(index_));
    }

    static void write_text(const std::filesystem::path& path, const std::string& value)
    {
        std::ofstream out(path);
        if (!out.is_open())
            throw std::runtime_error("Could not open " + path.string());
        out << value;
        if (!out.good())
            throw std::runtime_error("Could not write " + path.string());
    }

    static void try_write(const std::filesystem::path& path, const std::string& value)
    {
        try
        {
            write_text(path, value);
        }
        catch (...)
        {
        }
    }

    std::string spec_;
    std::string label_;
    bool inverted_ = false;
    bool dry_run_ = true;
    int index_ = 0;
    int period_ns_ = 20000000;
    float last_ms_ = 1.5f;
    std::filesystem::path chip_path_;
    std::filesystem::path pwm_path_;
};

class PwmOutputs
{
public:
    explicit PwmOutputs(const DeployConfig& cfg)
        : cfg_(cfg),
          steering_(cfg.steering_pwm, "steering", cfg.steering_output_inverted),
          throttle_(cfg.throttle_pwm, "throttle", cfg.throttle_output_inverted)
    {
    }

    ~PwmOutputs()
    {
        try
        {
            close();
        }
        catch (...)
        {
        }
    }

    void start()
    {
        steering_.start(cfg_.pwm_frequency_hz, cfg_.neutral_pulse_ms, cfg_.dry_run);
        try
        {
            throttle_.start(cfg_.pwm_frequency_hz, cfg_.neutral_pulse_ms, cfg_.dry_run);
            started_ = true;
            neutral();
        }
        catch (...)
        {
            steering_.close(cfg_.neutral_pulse_ms, cfg_.disable_pwm_on_exit);
            throw;
        }
    }

    void set_pulse_ms(float steering_ms, float throttle_ms)
    {
        steering_.set_pulse_ms(clampf(steering_ms, cfg_.min_pulse_ms, cfg_.max_pulse_ms));
        throttle_.set_pulse_ms(clampf(throttle_ms, cfg_.min_pulse_ms, cfg_.max_pulse_ms));
    }

    void neutral()
    {
        set_pulse_ms(cfg_.neutral_pulse_ms, cfg_.neutral_pulse_ms);
    }

    void close()
    {
        if (!started_)
            return;
        neutral();
        steering_.close(cfg_.neutral_pulse_ms, cfg_.disable_pwm_on_exit);
        throttle_.close(cfg_.neutral_pulse_ms, cfg_.disable_pwm_on_exit);
        started_ = false;
    }

private:
    DeployConfig cfg_;
    PwmChannel steering_;
    PwmChannel throttle_;
    bool started_ = false;
};

class GyroSteeringAssist
{
public:
    explicit GyroSteeringAssist(const DeployConfig& cfg)
        : cfg_(cfg)
    {
    }

    float update(
        float policy_steering,
        const std::vector<float>& imu_obs,
        const std::vector<float>& wheel_rpm,
        float dt)
    {
        sample_ = GyroAssistSample{};
        sample_.policy_steering = policy_steering;
        sample_.final_steering = policy_steering;
        if (!cfg_.gyro_assist_enabled)
            return policy_steering;
        if (imu_obs.size() < 6 || wheel_rpm.size() < 2)
            throw std::runtime_error("Gyro assist requires body gyro_z and both front wheel RPM channels");

        constexpr float kTwoPi = 6.2831853071795864769f;
        const float front_rpm = 0.5f * (std::abs(wheel_rpm[0]) + std::abs(wheel_rpm[1]));
        const float front_speed_mps =
            front_rpm * (kTwoPi / 60.0f) * cfg_.gyro_assist_wheel_radius_m;
        const float measured_yaw_rate = imu_obs[5];
        const float safe_dt = clampf(dt, 1.0e-3f, 0.1f);
        const float alpha = 1.0f - std::exp(
            -kTwoPi * cfg_.gyro_assist_cutoff_hz * safe_dt);
        filtered_yaw_rate_ += alpha * (measured_yaw_rate - filtered_yaw_rate_);

        const float target_yaw_rate = clampf(
            cfg_.gyro_assist_direction * front_speed_mps / cfg_.gyro_assist_target_radius_m,
            -cfg_.gyro_assist_max_target_yaw_rate_radps,
            cfg_.gyro_assist_max_target_yaw_rate_radps);
        const float yaw_rate_error = target_yaw_rate - filtered_yaw_rate_;
        float controlled_error = 0.0f;
        if (std::abs(yaw_rate_error) > cfg_.gyro_assist_error_deadband_radps)
        {
            controlled_error = std::copysign(
                std::abs(yaw_rate_error) - cfg_.gyro_assist_error_deadband_radps,
                yaw_rate_error);
        }
        const float activation = clampf(
            (front_speed_mps - cfg_.gyro_assist_activation_speed_mps) /
                cfg_.gyro_assist_activation_blend_mps,
            0.0f,
            1.0f);
        const float requested_correction = activation * clampf(
            cfg_.gyro_assist_kp * controlled_error,
            -cfg_.gyro_assist_max_correction,
            cfg_.gyro_assist_max_correction);
        correction_ = slew(
            correction_,
            requested_correction,
            cfg_.gyro_assist_correction_slew_per_s,
            safe_dt);
        correction_ = clampf(
            correction_,
            -cfg_.gyro_assist_max_correction,
            cfg_.gyro_assist_max_correction);
        const float final_steering = clampf(policy_steering + correction_, -1.0f, 1.0f);

        sample_.active = activation > 0.0f || std::abs(correction_) > 1.0e-5f;
        sample_.front_speed_mps = front_speed_mps;
        sample_.measured_yaw_rate_radps = measured_yaw_rate;
        sample_.filtered_yaw_rate_radps = filtered_yaw_rate_;
        sample_.target_yaw_rate_radps = target_yaw_rate;
        sample_.yaw_rate_error_radps = yaw_rate_error;
        sample_.correction = correction_;
        sample_.final_steering = final_steering;
        return final_steering;
    }

    void reset()
    {
        filtered_yaw_rate_ = 0.0f;
        correction_ = 0.0f;
        sample_ = GyroAssistSample{};
    }

    const GyroAssistSample& sample() const
    {
        return sample_;
    }

private:
    DeployConfig cfg_;
    float filtered_yaw_rate_ = 0.0f;
    float correction_ = 0.0f;
    GyroAssistSample sample_;
};

class CommandMapper
{
public:
    explicit CommandMapper(const DeployConfig& cfg)
        : cfg_(cfg)
    {
    }

    void decode(const std::vector<float>& raw, float& steering, float& throttle) const
    {
        if (raw.size() < 2)
            throw std::runtime_error("Policy output must contain at least two values");

        if (cfg_.policy_output == "pwm_ms" || cfg_.policy_output == "pwm" || cfg_.policy_output == "ms")
        {
            const float steering_ms = clampf(raw[0], cfg_.min_pulse_ms, cfg_.max_pulse_ms);
            const float throttle_ms = clampf(raw[1], cfg_.min_pulse_ms, cfg_.max_pulse_ms);
            steering = pulse_to_signed(steering_ms, cfg_.min_pulse_ms, cfg_.neutral_pulse_ms, cfg_.max_pulse_ms);
            throttle = pulse_to_signed(throttle_ms, cfg_.min_pulse_ms, cfg_.neutral_pulse_ms, cfg_.max_pulse_ms);
            // policy_pwm.onnx already emits pulse widths, but deployment sign
            // corrections still need to act around the 1.5 ms neutral point.
            steering = clampf(steering * cfg_.steering_sign, -1.0f, 1.0f);
            throttle = clampf(throttle * cfg_.throttle_sign, -1.0f, 1.0f);
            steering = clampf(steering * cfg_.steering_limit, -1.0f, 1.0f);
            throttle = clampf(
                throttle * cfg_.throttle_limit,
                cfg_.allow_reverse ? -1.0f : 0.0f,
                1.0f);
        }
        else
        {
            steering = clampf(raw[0] * cfg_.steering_sign, -1.0f, 1.0f);
            throttle = clampf(raw[1] * cfg_.throttle_sign, -1.0f, 1.0f);
            throttle = clampf(throttle, cfg_.allow_reverse ? -1.0f : 0.0f, 1.0f);
            steering = clampf(steering * cfg_.steering_limit, -1.0f, 1.0f);
            throttle = clampf(throttle * cfg_.throttle_limit, cfg_.allow_reverse ? -1.0f : 0.0f, 1.0f);
        }

    }

    void finalize(
        float requested_steering,
        float requested_throttle,
        float dt,
        float& steering,
        float& throttle,
        float& steering_ms,
        float& throttle_ms)
    {
        steering = slew(
            last_steering_, requested_steering, cfg_.max_steering_slew_per_s, dt);
        throttle = slew(
            last_throttle_, requested_throttle, cfg_.max_throttle_slew_per_s, dt);
        throttle = clampf(throttle, cfg_.allow_reverse ? -1.0f : 0.0f, 1.0f);
        steering_ms = signed_to_pulse(steering, cfg_.min_pulse_ms, cfg_.neutral_pulse_ms, cfg_.max_pulse_ms);
        throttle_ms = signed_to_pulse(throttle, cfg_.min_pulse_ms, cfg_.neutral_pulse_ms, cfg_.max_pulse_ms);
        last_steering_ = steering;
        last_throttle_ = throttle;
    }

    void neutral(float& steering, float& throttle, float& steering_ms, float& throttle_ms)
    {
        last_steering_ = 0.0f;
        last_throttle_ = 0.0f;
        steering = 0.0f;
        throttle = 0.0f;
        steering_ms = cfg_.neutral_pulse_ms;
        throttle_ms = cfg_.neutral_pulse_ms;
    }

private:
    DeployConfig cfg_;
    float last_steering_ = 0.0f;
    float last_throttle_ = 0.0f;
};

class GamepadSafety
{
public:
    explicit GamepadSafety(const DeployConfig& cfg)
        : cfg_(cfg),
          enabled_(!cfg.dry_run && cfg.require_gamepad)
    {
        paused_ = cfg_.gamepad_require_arm;
        message_ = cfg_.gamepad_require_arm ? "Press A to arm" : "Hold RB deadman";
    }

    ~GamepadSafety()
    {
        close();
    }

    void start()
    {
        if (!enabled_)
            return;

        fd_ = open_gamepad();
        if (fd_ < 0)
        {
            throw std::runtime_error(
                "Gamepad safety is required for real PWM output, but no matching gamepad was found. "
                "Turn on the controller before starting this runner.");
        }

        if (cfg_.gamepad_grab && ioctl(fd_, EVIOCGRAB, 1) < 0)
        {
            std::cout << LOGGER::WARNING << "Could not exclusively grab gamepad; continuing with safety monitor"
                      << std::endl;
        }
        grabbed_ = cfg_.gamepad_grab;

        std::array<unsigned long, (KEY_CNT / (8 * sizeof(unsigned long))) + 1> key_state{};
        if (ioctl(fd_, EVIOCGKEY(sizeof(unsigned long) * key_state.size()), key_state.data()) >= 0)
        {
            deadman_ = test_bit(key_state, cfg_.gamepad_deadman_button);
            if (test_bit(key_state, cfg_.gamepad_abort_button))
            {
                abort_ = true;
                message_ = "START held at startup";
            }
        }

        std::cout << LOGGER::INFO << "Gamepad safety connected: " << gamepad_name_
                  << " (" << gamepad_path_ << ")" << std::endl;
        std::cout << LOGGER::WARNING
                  << (cfg_.gamepad_require_arm
                          ? "Press A, then hold RB. "
                          : "RB-only field mode. Hold RB to move. ")
                  << "Release RB or press B for neutral; START aborts."
                  << std::endl;
    }

    bool update(std::string& reason, bool& abort_run)
    {
        abort_run = false;
        reason.clear();
        if (!enabled_)
            return true;

        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN | POLLERR | POLLHUP | POLLNVAL;
        while (poll(&pfd, 1, 0) > 0)
        {
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                abort_ = true;
                message_ = "Gamepad disconnected";
                break;
            }
            if (!(pfd.revents & POLLIN))
                break;

            input_event events[16];
            const ssize_t bytes = read(fd_, events, sizeof(events));
            if (bytes < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;
                abort_ = true;
                message_ = std::string("Gamepad read failed: ") + std::strerror(errno);
                break;
            }
            if (bytes == 0)
                break;

            const size_t count = static_cast<size_t>(bytes) / sizeof(input_event);
            for (size_t i = 0; i < count; ++i)
            {
                const input_event& event = events[i];
                if (event.type != EV_KEY || (event.value != 0 && event.value != 1))
                    continue;

                if (event.code == cfg_.gamepad_deadman_button)
                {
                    deadman_ = event.value == 1;
                    message_ = deadman_ && !paused_ ? "RB held: motion enabled" : "RB released: neutral";
                }
                else if (event.value == 1 && event.code == cfg_.gamepad_pause_button)
                {
                    paused_ = cfg_.gamepad_require_arm;
                    deadman_ = false;
                    message_ = cfg_.gamepad_require_arm
                        ? "B pause: neutral"
                        : "B neutral: release and re-hold RB";
                }
                else if (event.value == 1 && event.code == cfg_.gamepad_arm_button)
                {
                    if (cfg_.gamepad_require_arm)
                    {
                        paused_ = false;
                        message_ = "A resumed; hold RB";
                    }
                }
                else if (event.value == 1 && event.code == cfg_.gamepad_abort_button)
                {
                    paused_ = true;
                    deadman_ = false;
                    abort_ = true;
                    message_ = "START emergency stop";
                }
            }
        }

        if (abort_)
        {
            reason = message_;
            abort_run = true;
            return false;
        }
        if (paused_)
        {
            reason = message_.empty()
                ? (cfg_.gamepad_require_arm ? "Press A to arm" : "Hold RB deadman")
                : message_;
            return false;
        }
        if (!deadman_)
        {
            reason = "Hold RB deadman";
            return false;
        }
        return true;
    }

    void close()
    {
        if (fd_ >= 0)
        {
            if (grabbed_)
                ioctl(fd_, EVIOCGRAB, 0);
            ::close(fd_);
            fd_ = -1;
        }
    }

private:
    std::string event_device_name(int fd) const
    {
        char name[256] = {0};
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0)
            return "unknown";
        return std::string(name);
    }

    int score_device(int fd, const std::string& name) const
    {
        std::array<unsigned long, (EV_CNT / (8 * sizeof(unsigned long))) + 1> ev_bits{};
        std::array<unsigned long, (KEY_CNT / (8 * sizeof(unsigned long))) + 1> key_bits{};
        std::array<unsigned long, (ABS_CNT / (8 * sizeof(unsigned long))) + 1> abs_bits{};
        if (!read_event_bits(fd, 0, ev_bits) || !test_bit(ev_bits, EV_KEY))
            return -1000;
        if (!read_event_bits(fd, EV_KEY, key_bits))
            return -1000;
        read_event_bits(fd, EV_ABS, abs_bits);

        const int required_buttons[] = {
            cfg_.gamepad_arm_button,
            cfg_.gamepad_pause_button,
            cfg_.gamepad_abort_button,
            cfg_.gamepad_deadman_button,
        };
        for (int code : required_buttons)
        {
            if (!test_bit(key_bits, code))
                return -1000;
        }

        const std::string lower_name = lowercase(name);
        const char* blocked[] = {"keyboard", "mouse", "touchpad", "consumer control", "system control", "hda", "gpio-keys"};
        for (const char* word : blocked)
        {
            if (lower_name.find(word) != std::string::npos)
                return -1000;
        }

        int score = 100;
        if (test_bit(abs_bits, ABS_X))
            score += 20;
        if (test_bit(abs_bits, ABS_Y))
            score += 10;
        if (test_bit(abs_bits, ABS_Z) || test_bit(abs_bits, ABS_RZ) ||
            test_bit(abs_bits, ABS_GAS) || test_bit(abs_bits, ABS_BRAKE))
            score += 10;
        if (lower_name.find("controller") != std::string::npos ||
            lower_name.find("gamepad") != std::string::npos ||
            lower_name.find("xbox") != std::string::npos)
            score += 30;
        for (const std::string& wanted : cfg_.gamepad_name_contains)
        {
            if (!wanted.empty() && lower_name.find(lowercase(wanted)) != std::string::npos)
            {
                score += 60;
                break;
            }
        }
        return score;
    }

    int open_checked_device(const std::filesystem::path& path)
    {
        const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
            return -1;
        const std::string name = event_device_name(fd);
        if (score_device(fd, name) < 0)
        {
            ::close(fd);
            return -1;
        }
        gamepad_name_ = name;
        gamepad_path_ = path.string();
        return fd;
    }

    int open_gamepad()
    {
        if (cfg_.gamepad_device != "auto")
            return open_checked_device(cfg_.gamepad_device);

        int best_fd = -1;
        int best_score = -1000;
        std::string best_name;
        std::string best_path;
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator("/dev/input", ec))
        {
            if (ec)
                continue;
            std::error_code type_ec;
            if (!entry.is_character_file(type_ec))
                continue;
            const std::string filename = entry.path().filename().string();
            if (filename.rfind("event", 0) != 0)
                continue;

            const int fd = ::open(entry.path().c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0)
                continue;
            const std::string name = event_device_name(fd);
            const int score = score_device(fd, name);
            if (score > best_score)
            {
                if (best_fd >= 0)
                    ::close(best_fd);
                best_fd = fd;
                best_score = score;
                best_name = name;
                best_path = entry.path().string();
            }
            else
            {
                ::close(fd);
            }
        }

        if (best_fd >= 0)
        {
            gamepad_name_ = best_name;
            gamepad_path_ = best_path;
        }
        return best_fd;
    }

    DeployConfig cfg_;
    bool enabled_ = false;
    int fd_ = -1;
    bool grabbed_ = false;
    bool paused_ = true;
    bool deadman_ = false;
    bool abort_ = false;
    std::string message_ = "Press A to arm";
    std::string gamepad_name_;
    std::string gamepad_path_;
};

void systemctl_user(const std::string& action, const std::string& service)
{
    std::string cmd = "systemctl --user " + action + " " + service + " >/dev/null 2>&1";
    std::system(cmd.c_str());
}

class GamepadServiceGuard
{
public:
    GamepadServiceGuard(bool should_stop, bool should_restart)
        : should_stop_(should_stop),
          should_restart_(should_restart)
    {
    }

    ~GamepadServiceGuard()
    {
        restart();
    }

    void stop()
    {
        if (!should_stop_ || stopped_)
            return;
        std::cout << LOGGER::WARNING << "Stopping gamepad-pwm.service before taking PWM ownership" << std::endl;
        systemctl_user("stop", "gamepad-pwm.service");
        stopped_ = true;
    }

    void restart()
    {
        if (stopped_ && should_restart_)
        {
            std::cout << LOGGER::WARNING << "Restarting gamepad-pwm.service" << std::endl;
            systemctl_user("start", "gamepad-pwm.service");
        }
        stopped_ = false;
    }

private:
    bool should_stop_ = false;
    bool should_restart_ = true;
    bool stopped_ = false;
};

void print_usage(const char* argv0)
{
    std::cout << "Usage: " << argv0 << " [--config PATH] [--model PATH] [--dry-run|--enable-output]"
              << " [--max-runtime-s N] [--pwm-replay PATH] [--replay-episode N]"
              << " [--rb-only] [--keep-gamepad-service-stopped]\n";
}

} // namespace

int main(int argc, char** argv)
{
    try
    {
        std::filesystem::path config_path =
            std::filesystem::path(POLICY_DIR) / "MJX_ET6/nio_lab/drift/config.yaml";
        std::string model_override;
        bool enable_output = false;
        bool force_dry_run = false;
        bool rb_only = false;
        bool keep_gamepad_service_stopped = false;
        double runtime_override = -1.0;
        std::filesystem::path pwm_replay_path;
        int replay_episode = 0;

        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];
            if (arg == "--help" || arg == "-h")
            {
                print_usage(argv[0]);
                return 0;
            }
            else if (arg == "--config" && i + 1 < argc)
            {
                config_path = argv[++i];
            }
            else if (arg == "--model" && i + 1 < argc)
            {
                model_override = argv[++i];
            }
            else if (arg == "--enable-output")
            {
                enable_output = true;
            }
            else if (arg == "--dry-run")
            {
                force_dry_run = true;
            }
            else if (arg == "--rb-only")
            {
                rb_only = true;
            }
            else if (arg == "--keep-gamepad-service-stopped")
            {
                keep_gamepad_service_stopped = true;
            }
            else if (arg == "--max-runtime-s" && i + 1 < argc)
            {
                runtime_override = std::stod(argv[++i]);
            }
            else if (arg == "--pwm-replay" && i + 1 < argc)
            {
                pwm_replay_path = argv[++i];
            }
            else if (arg == "--replay-episode" && i + 1 < argc)
            {
                replay_episode = std::stoi(argv[++i]);
            }
            else
            {
                throw std::runtime_error("Unknown or incomplete argument: " + arg);
            }
        }

        DeployConfig cfg = load_config(config_path);
        if (!model_override.empty())
            cfg.model_path = model_override;
        if (runtime_override >= 0.0)
            cfg.max_runtime_s = runtime_override;
        else if (!pwm_replay_path.empty())
            cfg.max_runtime_s = 1.0;
        cfg.dry_run = !enable_output || force_dry_run;
        if (rb_only)
            cfg.gamepad_require_arm = false;
        if (keep_gamepad_service_stopped)
        {
            cfg.stop_gamepad_service = false;
            cfg.restart_gamepad_service_on_exit = false;
        }
        if (replay_episode < 0)
            throw std::runtime_error("--replay-episode must be non-negative");
        if (!cfg.dry_run && !cfg.real_output_enabled)
        {
            throw std::runtime_error(
                "Real PWM output is locked. Retrain and deploy a policy using the masked-yaw, "
                "calibrated-gyro observation contract before setting real_output_enabled: true.");
        }

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        std::unique_ptr<PwmReplay> pwm_replay;
        if (!pwm_replay_path.empty())
        {
            pwm_replay = std::make_unique<PwmReplay>(
                pwm_replay_path, static_cast<size_t>(replay_episode), cfg);
        }

        std::cout << LOGGER::INFO << "MJX ET6 "
                  << (pwm_replay ? "recorded PWM replay" : "ONNX policy deployment") << std::endl;
        std::cout << LOGGER::INFO << "Config: " << cfg.config_path << std::endl;
        if (pwm_replay)
        {
            std::cout << LOGGER::INFO << "PWM replay: " << pwm_replay->path()
                      << " episode=" << pwm_replay->episode()
                      << " commands=" << pwm_replay->size()
                      << " duration=" << pwm_replay->duration_s() << " s" << std::endl;
        }
        else
        {
            std::cout << LOGGER::INFO << "Model: " << cfg.model_path << std::endl;
        }
        std::cout << LOGGER::INFO << "Output: " << (cfg.dry_run ? "DRY RUN" : "REAL PWM") << std::endl;
        std::cout << LOGGER::INFO << "Control loop: " << cfg.frequency_hz
                  << " Hz; PWM carrier: " << cfg.pwm_frequency_hz << " Hz" << std::endl;
        std::cout << LOGGER::INFO << "IMU mapping: gyro_z_sign=" << cfg.imu_gyro_z_sign
                  << "; policy absolute yaw=" << (cfg.mask_absolute_yaw ? "masked" : "enabled")
                  << "; mount roll/pitch=" << cfg.imu_mount_roll_rad << "/"
                  << cfg.imu_mount_pitch_rad
                  << std::endl;
        std::cout << LOGGER::INFO << "Observation history: term-major; wheel RPM clip-before-scale="
                  << cfg.wheel_rpm_clip_before_scale << " / " << cfg.max_free_spin_rpm
                  << std::endl;
        std::cout << LOGGER::INFO << "Gyro residual assist: "
                  << (cfg.gyro_assist_enabled && !pwm_replay ? "enabled" : "bypassed")
                  << "; radius=" << cfg.gyro_assist_target_radius_m
                  << " m kp=" << cfg.gyro_assist_kp
                  << " max=" << cfg.gyro_assist_max_correction
                  << " cutoff=" << cfg.gyro_assist_cutoff_hz << " Hz"
                  << std::endl;

        decltype(InferenceRuntime::ModelFactory::load_model(cfg.model_path.string())) model;
        if (!pwm_replay)
        {
            model = InferenceRuntime::ModelFactory::load_model(cfg.model_path.string());
            if (!model)
                throw std::runtime_error("Failed to load model: " + cfg.model_path.string());
        }

        SensorClient sensors(cfg);
        sensors.calibrate_startup();
        ObservationHistory history(cfg);
        CommandMapper mapper(cfg);
        GyroSteeringAssist gyro_assist(cfg);
        PwmOutputs outputs(cfg);
        CsvLogger csv(cfg.telemetry_path);

        GamepadServiceGuard gamepad_service(!cfg.dry_run && cfg.stop_gamepad_service, cfg.restart_gamepad_service_on_exit);
        gamepad_service.stop();
        GamepadSafety gamepad_safety(cfg);
        gamepad_safety.start();
        outputs.start();

        const double period_s = 1.0 / std::max(cfg.frequency_hz, 1.0f);
        const double print_period_s = 1.0 / std::max(cfg.print_rate_hz, 0.1);
        using LoopClock = std::chrono::steady_clock;
        const auto loop_period =
            std::chrono::duration_cast<LoopClock::duration>(std::chrono::duration<double>(period_s));
        auto next_loop_deadline = LoopClock::now();
        const double start_s = now_monotonic_s();
        const bool count_active_runtime = !cfg.dry_run && cfg.require_gamepad;
        double active_runtime_s = 0.0;
        double last_loop_s = start_s;
        double last_print_s = 0.0;
        float last_policy_steering_action = 0.0f;
        float last_policy_throttle_action = 0.0f;
        std::vector<float> imu_obs(9, 0.0f);
        std::vector<float> raw_rpm(4, 0.0f);
        std::size_t loop_count = 0;
        std::size_t loop_deadline_miss_count = 0;
        std::size_t policy_sample_count = 0;
        std::size_t replay_sample_index = 0;
        const std::size_t replay_sample_limit = pwm_replay && cfg.max_runtime_s > 0.0
            ? std::min(
                  pwm_replay->size(),
                  static_cast<std::size_t>(std::floor(cfg.max_runtime_s * cfg.frequency_hz + 1.0e-6)))
            : (pwm_replay ? pwm_replay->size() : 0);
        double measured_loop_time_s = 0.0;
        double max_loop_dt_s = 0.0;
        double sensor_time_s = 0.0;
        double inference_time_s = 0.0;
        double loop_work_time_s = 0.0;
        double max_loop_work_time_s = 0.0;

        while (!g_stop_requested.load())
        {
            if (pwm_replay && replay_sample_index >= replay_sample_limit)
            {
                std::cout << LOGGER::WARNING << "PWM replay command limit reached; returning neutral" << std::endl;
                break;
            }
            const double loop_start_s = now_monotonic_s();
            const float dt = static_cast<float>(std::max(1.0e-3, loop_start_s - last_loop_s));
            last_loop_s = loop_start_s;
            if (loop_count > 0)
            {
                measured_loop_time_s += static_cast<double>(dt);
                max_loop_dt_s = std::max(max_loop_dt_s, static_cast<double>(dt));
            }
            ++loop_count;

            if (!count_active_runtime && cfg.max_runtime_s > 0.0 && loop_start_s - start_s >= cfg.max_runtime_s)
            {
                std::cout << LOGGER::WARNING << "Max runtime reached; returning neutral" << std::endl;
                break;
            }

            bool sensor_ok = false;
            bool policy_ok = false;
            std::string safe_reason;
            float steering = 0.0f;
            float throttle = 0.0f;
            float steering_ms = cfg.neutral_pulse_ms;
            float throttle_ms = cfg.neutral_pulse_ms;
            float policy_steering_action = 0.0f;
            float policy_throttle_action = 0.0f;
            float policy_raw_0 = 0.0f;
            float policy_raw_1 = 0.0f;
            GyroAssistSample gyro_sample;
            long long replay_log_episode = -1;
            long long replay_log_command_index = -1;
            long long replay_log_sequence_index = -1;
            bool replay_command_pending = false;
            bool safety_abort = false;
            std::string safety_reason;
            const bool safety_allowed = gamepad_safety.update(safety_reason, safety_abort);
            if (safety_allowed && count_active_runtime && !pwm_replay)
                active_runtime_s += static_cast<double>(dt);

            if (!safety_allowed)
            {
                mapper.neutral(steering, throttle, steering_ms, throttle_ms);
                gyro_assist.reset();
                safe_reason = safety_reason;
            }
            else
            {
                try
                {
                    std::vector<float> wheel_obs;
                    const double sensor_start_s = now_monotonic_s();
                    sensors.read(imu_obs, wheel_obs, raw_rpm);
                    sensor_time_s += now_monotonic_s() - sensor_start_s;
                    sensor_ok = true;
                    if (pwm_replay)
                    {
                        const auto& sample = pwm_replay->sample(replay_sample_index);
                        steering_ms = sample.steering_pwm_ms;
                        throttle_ms = sample.throttle_pwm_ms;
                        steering = pulse_to_signed(
                            steering_ms, cfg.min_pulse_ms, cfg.neutral_pulse_ms, cfg.max_pulse_ms);
                        throttle = pulse_to_signed(
                            throttle_ms, cfg.min_pulse_ms, cfg.neutral_pulse_ms, cfg.max_pulse_ms);
                        policy_steering_action = steering;
                        policy_throttle_action = throttle;
                        gyro_sample.policy_steering = steering;
                        gyro_sample.final_steering = steering;
                        policy_raw_0 = sample.steering_action_raw;
                        policy_raw_1 = sample.throttle_action_raw;
                        replay_log_episode = static_cast<long long>(sample.episode_index);
                        replay_log_command_index = static_cast<long long>(sample.episode_command_index);
                        replay_log_sequence_index = static_cast<long long>(sample.sequence_index);
                        replay_command_pending = true;
                        safe_reason = "pwm_replay";
                    }
                    else
                    {
                        std::vector<float> obs = history.update(
                            imu_obs,
                            wheel_obs,
                            last_policy_steering_action,
                            last_policy_throttle_action);
                        const double inference_start_s = now_monotonic_s();
                        std::vector<float> raw_output = model->forward({obs});
                        inference_time_s += now_monotonic_s() - inference_start_s;
                        ++policy_sample_count;
                        if (!raw_output.empty())
                            policy_raw_0 = raw_output[0];
                        if (raw_output.size() > 1)
                            policy_raw_1 = raw_output[1];
                        mapper.decode(
                            raw_output, policy_steering_action, policy_throttle_action);
                        const float assisted_steering = gyro_assist.update(
                            policy_steering_action, imu_obs, raw_rpm, dt);
                        mapper.finalize(
                            assisted_steering,
                            policy_throttle_action,
                            dt,
                            steering,
                            throttle,
                            steering_ms,
                            throttle_ms);
                        gyro_sample = gyro_assist.sample();
                        const float rear_rpm = raw_rpm.size() >= 4 ? std::max(raw_rpm[2], raw_rpm[3]) : 0.0f;
                        const double runtime_s = count_active_runtime ? active_runtime_s : (loop_start_s - start_s);
                        if (
                            cfg.launch_assist_enabled &&
                            runtime_s <= cfg.launch_assist_duration_s &&
                            rear_rpm < cfg.launch_assist_release_rpm &&
                            throttle < cfg.launch_assist_throttle)
                        {
                            throttle = cfg.launch_assist_throttle;
                            throttle_ms = signed_to_pulse(
                                throttle,
                                cfg.min_pulse_ms,
                                cfg.neutral_pulse_ms,
                                cfg.max_pulse_ms);
                            safe_reason = "launch_assist";
                        }
                    }
                    policy_ok = true;
                }
                catch (const std::exception& e)
                {
                    mapper.neutral(steering, throttle, steering_ms, throttle_ms);
                    gyro_assist.reset();
                    gyro_sample = GyroAssistSample{};
                    policy_steering_action = 0.0f;
                    policy_throttle_action = 0.0f;
                    safe_reason = e.what();
                    std::cout << LOGGER::WARNING << "Neutral command: " << safe_reason << std::endl;
                }
            }

            outputs.set_pulse_ms(steering_ms, throttle_ms);
            if (replay_command_pending)
            {
                ++replay_sample_index;
                active_runtime_s = static_cast<double>(replay_sample_index) * period_s;
                ++policy_sample_count;
            }
            last_policy_steering_action = policy_steering_action;
            last_policy_throttle_action = policy_throttle_action;
            csv.write(
                dt,
                sensor_ok,
                policy_ok,
                safe_reason,
                pwm_replay ? "pwm_replay" : "onnx_policy",
                replay_log_episode,
                replay_log_command_index,
                replay_log_sequence_index,
                policy_raw_0,
                policy_raw_1,
                policy_steering_action,
                policy_throttle_action,
                steering,
                throttle,
                steering_ms,
                throttle_ms,
                gyro_sample,
                imu_obs,
                raw_rpm);

            if (loop_start_s - last_print_s >= print_period_s)
            {
                last_print_s = loop_start_s;
                std::cout << LOGGER::INFO << "cmd steer=" << steering
                          << " throttle=" << throttle
                          << " policy_steer=" << policy_steering_action
                          << " gyro_corr=" << gyro_sample.correction
                          << " yaw=(" << gyro_sample.filtered_yaw_rate_radps
                          << "/" << gyro_sample.target_yaw_rate_radps << ")"
                          << " raw=(" << policy_raw_0 << ", " << policy_raw_1 << ")"
                          << " pwm=(" << steering_ms << ", " << throttle_ms << ")"
                          << " rpm=(" << raw_rpm[0] << ", " << raw_rpm[1] << ", " << raw_rpm[2] << ", " << raw_rpm[3] << ")"
                          << (safe_reason.empty() ? "" : " safe=" + safe_reason)
                          << std::endl;
            }

            if (safety_abort)
            {
                std::cout << LOGGER::WARNING << "Gamepad safety abort: " << safe_reason << std::endl;
                break;
            }

            if (count_active_runtime && cfg.max_runtime_s > 0.0 && active_runtime_s >= cfg.max_runtime_s)
            {
                std::cout << LOGGER::WARNING << "Max active runtime reached; returning neutral" << std::endl;
                break;
            }

            const double loop_work_s = now_monotonic_s() - loop_start_s;
            loop_work_time_s += loop_work_s;
            max_loop_work_time_s = std::max(max_loop_work_time_s, loop_work_s);

            next_loop_deadline += loop_period;
            const auto now = LoopClock::now();
            if (now < next_loop_deadline)
            {
                std::this_thread::sleep_until(next_loop_deadline);
            }
            else
            {
                ++loop_deadline_miss_count;
                if (now - next_loop_deadline >= loop_period)
                    next_loop_deadline = now;
            }
        }

        if (loop_count > 1 && measured_loop_time_s > 0.0)
        {
            const double measured_hz = static_cast<double>(loop_count - 1) / measured_loop_time_s;
            std::cout << LOGGER::INFO << "Control loop measured=" << measured_hz
                      << " Hz target=" << cfg.frequency_hz
                      << " Hz max_dt_ms=" << max_loop_dt_s * 1000.0
                      << " deadline_misses=" << loop_deadline_miss_count << std::endl;
            std::cout << LOGGER::INFO << "Loop timing avg_work_ms="
                      << loop_work_time_s * 1000.0 / static_cast<double>(loop_count)
                      << " max_work_ms=" << max_loop_work_time_s * 1000.0;
            if (policy_sample_count > 0)
            {
                std::cout << " avg_sensor_ms="
                          << sensor_time_s * 1000.0 / static_cast<double>(policy_sample_count)
                          << " avg_inference_ms="
                          << inference_time_s * 1000.0 / static_cast<double>(policy_sample_count);
            }
            std::cout << std::endl;
        }

        if (cfg.neutral_on_exit)
        {
            std::cout << LOGGER::WARNING << "Setting neutral PWM outputs" << std::endl;
            outputs.neutral();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            outputs.neutral();
        }
        outputs.close();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << LOGGER::ERROR << e.what() << std::endl;
        return 2;
    }
}
