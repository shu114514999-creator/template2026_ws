/************************************************************************
 * myprogramme.cpp  ——  爱心任务
 *
 * 结构完全参考 hover.cpp：
 *   setlocale -> init -> 订阅/发布/服务 -> Rate -> 读参数 -> print_param
 *   -> 输入 1 -> 等连接
 *   -> 第一段循环: 切 OFFBOARD / 解锁 / 飞到目标高度后 break
 *   -> 第二段循环: switch(mission_num) 任务状态机
 *
 * 只有「画爱心」是新增的，其余（含 if_debug 遥控器/自动 offboard 分支、
 * lib_time_record_func 计时、precision_land 降落）都照 hover.cpp。
 *
 * 相对 hover.cpp 只多两处，都是你要求修的：
 *   1) 悬停后等速度收敛再降落（否则 precision_land 锁的落点在漂移途中，
 *      下降不垂直）
 *   2) 降落成功后轮询到 armed == false 才退出（hover.cpp 直接 exit(0)，
 *      节点一退出没人发设定点 -> PX4 失控保护 -> 桨叶一直转）
 ************************************************************************/
#include <template.h>

#include <iostream>
#include <algorithm> // std::min / std::max
#include <vector>
#include <cmath>

/* ==================== 爱心参数（yaml 可覆盖） ==================== */
#define HEART_DS 0.02f // 航点等弧长间距 m

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==================== 全局变量定义（同 hover.cpp） ==================== */
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;

/* 爱心任务参数 */
float heart_altitude = 1.2;
float heart_width = 1.6;
float heart_offset_x = 0.0;
float heart_offset_y = 0.0;
float heart_speed = 0.30;
float heart_hold = 3.0;
float heart_ctrl_hz = 20.0;

void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if (if_debug == 1)
    std::cout << "自动offboard" << std::endl;
  else
    std::cout << "遥控器offboard" << std::endl;
  std::cout << "heart: altitude=" << heart_altitude
            << " width=" << heart_width
            << " speed=" << heart_speed
            << " hold=" << heart_hold
            << " offset=(" << heart_offset_x << ", " << heart_offset_y << ")"
            << std::endl;
}

/************************************************************************
 * 函数 A: 心形线参数方程
 *      x(t) = 16*sin(t)^3
 *      y(t) = 13*cos(t) - 5*cos(2t) - 2*cos(3t) - cos(4t)
 *   t=0 在心口(上方凹处), t=pi 在心尖
 ************************************************************************/
static inline double heart_x(double t)
{
  const double s = sin(t);
  return 16.0 * s * s * s;
}
static inline double heart_y(double t) { return 13.0 * cos(t) - 5.0 * cos(2.0 * t) - 2.0 * cos(3.0 * t) - cos(4.0 * t); }

struct HeartPath
{
  std::vector<float> x, y; // 相对爱心中心
  float ds = HEART_DS;     // 航点间距 m
  float length = 0.0f;     // 周长 m
};

/************************************************************************
 * 函数 B: 生成爱心航点
 *   先密采样算累积弧长, 再按等弧长重采样 -> 匀速, 心尖不甩尾
 ************************************************************************/
static HeartPath make_heart(float width, float ds)
{
  const int N = 4000;
  std::vector<double> px(N + 1), py(N + 1);
  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;

  for (int i = 0; i <= N; ++i)
  {
    const double t = 2.0 * M_PI * i / N;
    px[i] = heart_x(t);
    py[i] = heart_y(t);
    xmin = std::min(xmin, px[i]);
    xmax = std::max(xmax, px[i]);
    ymin = std::min(ymin, py[i]);
    ymax = std::max(ymax, py[i]);
  }

  const float k = width / (xmax - xmin); // 曲线宽 32 单位 -> width 米
  const double cx = 0.5 * (xmin + xmax); // 居中
  const double cy = 0.5 * (ymin + ymax);

  std::vector<float> s(N + 1, 0.0f); // 累积弧长
  for (int i = 1; i <= N; ++i)
    s[i] = s[i - 1] + std::hypot((px[i] - px[i - 1]) * k, (py[i] - py[i - 1]) * k);

  HeartPath p;
  p.length = s[N];
  const int M = std::max(8, (int)std::lround(p.length / ds));
  p.ds = p.length / M;
  p.x.reserve(M + 1);
  p.y.reserve(M + 1);

  int j = 0;
  for (int m = 0; m <= M; ++m)
  {
    const float sd = p.length * m / M;
    while (j + 1 < N && s[j + 1] < sd)
      ++j;
    const float seg = s[j + 1] - s[j];
    const float w = (seg > 1e-9f) ? (sd - s[j]) / seg : 0.0f;
    p.x.push_back((px[j] + w * (px[j + 1] - px[j]) - cx) * k);
    p.y.push_back((py[j] + w * (py[j + 1] - py[j]) - cy) * k);
  }

  ROS_INFO("heart path: %zu 点, 周长 %.2f m, 间距 %.3f m", p.x.size(), p.length, p.ds);
  return p;
}

/************************************************************************
 * 函数 C: 按爱心轨迹发设定点（一圈画完返回 true）
 *   type_mask / coordinate_frame / init 偏移 全部和 mission_pos_cruise 一致
 ************************************************************************/
static bool heart_fly(const HeartPath &p, float ox, float oy, float z, float speed)
{
  static bool started = false;
  static ros::Time t0;
  static float length = 0.0f;

  if (!started)
  {
    t0 = ros::Time::now();
    length = p.length;
    started = true;
  }

  const float moved = speed * (ros::Time::now() - t0).toSec();
  const int idx = (int)(moved / p.ds);
  if (idx + 1 >= (int)p.x.size())
  {
    started = false;
    ROS_INFO("爱心画完, 用时 %.1f s", (ros::Time::now() - t0).toSec());
    return true;
  }

  setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = p.x[idx] + ox + init_position_x_take_off;
  setpoint_raw.position.y = p.y[idx] + oy + init_position_y_take_off;
  setpoint_raw.position.z = z + init_position_z_take_off;
  setpoint_raw.yaw = 0;

  ROS_INFO_THROTTLE(1.0, "heart: %.0f%%  (%.2f, %.2f)", 100.0 * moved / length,
                    setpoint_raw.position.x, setpoint_raw.position.y);
  return false;
}

/************************************************************************
 * 函数 D: 等飞机速度收敛（降落前用）
 *   precision_land() 在第一次调用时把「当前 xy」锁成落点，若此时还在横向
 *   漂移，落点就锁偏了，下降看起来不垂直。hover.cpp 是靠 hover(30) 干等，
 *   这里用速度阈值判断，通常 1~2 秒就够了。
 ************************************************************************/
static bool settle(float v_eps = 0.08f, double timeout_s = 8.0)
{
  static bool started = false;
  static ros::Time t0;

  if (!started)
  {
    t0 = ros::Time::now();
    started = true;
    ROS_INFO("等待停稳 ...");
  }

  const geometry_msgs::Twist &v = local_pos.twist.twist;
  const float vxy = (float)std::hypot(v.linear.x, v.linear.y);
  const float vz = (float)std::fabs(v.linear.z);

  if ((vxy < v_eps && vz < v_eps) || (ros::Time::now() - t0).toSec() > timeout_s)
  {
    started = false;
    ROS_INFO("已停稳 (vxy=%.3f m/s, vz=%.3f m/s)", vxy, vz);
    return true;
  }
  return false;
}

/************************************************************************
 * 函数 E: 落地后停桨（轮询到 armed == false 才算完）
 *
 *   hover.cpp 里 landing 完是 exit(0)，节点一退出就没人发设定点，
 *   PX4 1 秒后触发 OFFBOARD 失控保护（COM_OF_LOSS_T），桨叶继续转。
 *
 *   安全约束：
 *     - 只有高度 < 0.25 m 才允许强制上锁，空中强制上锁等于摔机
 *     - 空中超过 5 秒还没落地 -> 切 AUTO.LAND 交给飞控
 *     - 全程持续发设定点，避免掉进失控保护
 ************************************************************************/
static bool stop_motors(ros::Publisher &pub, ros::ServiceClient &setmode,
                        ros::ServiceClient &arming, ros::ServiceClient &cmd, ros::Rate &rate)
{
  mavros_msgs::CommandBool arm_false;
  arm_false.request.value = false;

  mavros_msgs::SetMode land_mode;
  land_mode.request.custom_mode = "AUTO.LAND";

  const ros::Time t0 = ros::Time::now();
  ros::Time t_last = t0;
  bool auto_land_sent = false;
  bool told_waiting = false;

  while (ros::ok() && current_state.armed)
  {
    const double dt = (ros::Time::now() - t0).toSec();
    const double h = local_pos.pose.pose.position.z - init_position_z_take_off;

    if ((ros::Time::now() - t_last).toSec() > 1.0)
    {
      t_last = ros::Time::now();

      if (h < 0.25)
      {
        // 已贴地：先普通上锁，失败再强制上锁
        if (arming.call(arm_false) && arm_false.response.success)
        {
          ROS_INFO("上锁成功 (h=%.2f m)", h);
        }
        else
        {
          mavros_msgs::CommandLong force;
          force.request.command = 400;     // MAV_CMD_COMPONENT_ARM_DISARM
          force.request.param1 = 0.0f;     // 0 = disarm
          force.request.param2 = 21196.0f; // 21196 = 强制
          if (cmd.call(force) && force.response.success)
            ROS_WARN("强制上锁成功 (h=%.2f m)", h);
        }
      }
      else
      {
        // 还在空中：绝对不能强制上锁
        if (!told_waiting)
        {
          ROS_INFO("还在 %.2f m 高，等待落地 ...", h);
          told_waiting = true;
        }
        if (!auto_land_sent && dt > 5.0)
        {
          if (setmode.call(land_mode) && land_mode.response.mode_sent)
          {
            auto_land_sent = true;
            ROS_WARN("切 AUTO.LAND，交给飞控降落并自动上锁");
          }
        }
      }
    }

    if (dt > 20.0)
    {
      ROS_ERROR("20 秒内未停桨%s，请用遥控器或 QGC 手动上锁",
                auto_land_sent ? "（已切 AUTO.LAND，飞控会自行降落上锁）" : "");
      return false;
    }

    pub.publish(setpoint_raw); // 持续发，别掉进失控保护
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("桨叶已停转（armed=false）");
  return true;
}

/************************************************************************
 * main（骨架照 hover.cpp）
 ************************************************************************/
int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "myprogramme");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取（launch 里 rosparam 在 node 外面，参数在根命名空间）
  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);
  nh.param<float>("altitude", heart_altitude, heart_altitude);
  nh.param<float>("width", heart_width, heart_width);
  nh.param<float>("offset_x", heart_offset_x, heart_offset_x);
  nh.param<float>("offset_y", heart_offset_y, heart_offset_y);
  nh.param<float>("speed", heart_speed, heart_speed);
  nh.param<float>("hold_time", heart_hold, heart_hold);
  nh.param<float>("ctrl_hz", heart_ctrl_hz, heart_ctrl_hz);
  print_param();

  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1)
    return 0;
  ros::spinOnce();
  rate.sleep();

  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  /* ====================================================================
   * 第一段：切 OFFBOARD / 解锁 / 爬到目标高度（照抄 hover.cpp）
   * ==================================================================== */
  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if (if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    // 到达目标高度后，等 1 秒再进入任务状态机
    if (std::fabs(local_pos.pose.pose.position.z - heart_altitude) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
        last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, heart_altitude, 0, err_max);
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  /* ---- 爱心航点（唯一新增的数据准备）---- */
  const HeartPath heart = make_heart(heart_width, HEART_DS);

  // precision_land() 的 type_mask 是 960，没有屏蔽 velocity / yaw_rate 字段，
  // 先清零，免得上一个任务留下的速度把飞机往横向带
  setpoint_raw.velocity.x = 0.0;
  setpoint_raw.velocity.y = 0.0;
  setpoint_raw.velocity.z = 0.0;
  setpoint_raw.yaw_rate = 0.0;

  /* ====================================================================
   * 第二段：任务状态机（hover.cpp 的 case 1/2/3，中间插入画爱心）
   * ==================================================================== */
  while (ros::ok())
  {
    ROS_WARN_THROTTLE(1.0, "mission_num = %d", mission_num);

    switch (mission_num)
    {
    // mission1: 起飞（到目标高度并稳定 1 秒）
    case 1:
      if (mission_pos_cruise(0, 0, heart_altitude, 0, err_max))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
        {
          mission_num = 2;
          last_request = ros::Time::now();
        }
      }
      else if (ros::Time::now() - last_request >= ros::Duration(3.0))
      {
        mission_num = 2;
        last_request = ros::Time::now();
        lib_time_init_flag = true;
      }
      break;

    // mission2: 飞到爱心起点（心口凹处）
    case 2:
      if (mission_pos_cruise(heart.x[0] + heart_offset_x,
                             heart.y[0] + heart_offset_y,
                             heart_altitude, 0, err_max))
      {
        ROS_INFO("到达爱心起点，开始画心");
        mission_num = 3;
      }
      break;

    // mission3: 画爱心
    case 3:
      if (heart_fly(heart, heart_offset_x, heart_offset_y, heart_altitude, heart_speed))
      {
        mission_num = 4;
      }
      break;

    // mission4: 悬停 + 等速度收敛（收敛完才能锁准降落点）
    case 4:
      if (hover(heart_hold) && settle())
      {
        mission_num = 5;
      }
      break;

    // mission5: 降落
    case 5:
      if (precision_land())
      {
        mission_num = 6;
      }
      break;

    // mission6: 等落地并停桨（hover.cpp 没有这一步，是要修的问题）
    case 6:
      stop_motors(mavros_setpoint_pos_pub, set_mode_client, arming_client, ctrl_pwm_client, rate);
      mission_num = -1; // 任务结束
      break;

    default:
      break;
    }

    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();

    if (mission_num == -1)
    {
      exit(0);
    }
  }
  return 0;
}
