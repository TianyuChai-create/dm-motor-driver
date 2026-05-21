#include "protocol/damiao.h"
#include <csignal>

// 原子标志，用于安全地跨线程修改
std::atomic<bool> running(true);

// Ctrl+C 触发的信号处理函数
void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

int main(int argc, char** argv)
{
  using clock = std::chrono::steady_clock;
  using duration = std::chrono::duration<double>;

  std::signal(SIGINT, signalHandler);

  try 
  {
      constexpr uint16_t canid1 = 0x01;
      constexpr uint16_t mstid1 = 0x11;
      constexpr uint16_t canid2 = 0x02;
      constexpr uint16_t mstid2 = 0x12;
      constexpr uint16_t canid3 = 0x03;
      constexpr uint16_t mstid3 = 0x13;
      constexpr uint16_t canid4 = 0x04;
      constexpr uint16_t mstid4 = 0x14;
      constexpr uint16_t canid5 = 0x05;
      constexpr uint16_t mstid5 = 0x15;
      constexpr uint16_t motor_ids[] = {canid1, canid2, canid3, canid4, canid5};
      
      uint32_t nom_baud = 1000000;
      uint32_t dat_baud = 1000000;

      std::vector<damiao::DmActData> init_data = {
        {damiao::DM4310, damiao::MIT_MODE, canid1, mstid1},
        {damiao::DM4310, damiao::MIT_MODE, canid2, mstid2},
        {damiao::DM4310, damiao::MIT_MODE, canid3, mstid3},
        {damiao::DM8009, damiao::MIT_MODE, canid4, mstid4},
        {damiao::DM8009, damiao::MIT_MODE, canid5, mstid5},
      };

      std::shared_ptr<damiao::Motor_Control> control = std::make_shared<damiao::Motor_Control>(nom_baud, dat_baud,
        "14AA044B241402B10DDBDAFE448040BB", &init_data);

      while (running) 
      { 
        const duration desired_duration(0.001); // 计算期望周期
        auto current_time = clock::now();

        for(auto id : motor_ids)
        {
          auto motor = control->getMotor(id);
          if(motor)
          {
            control->control_mit(*motor, 0.0, 0.0, 0.0, 0.0, 0.0);
          }
        }

        for(auto id : motor_ids)
        {
          auto motor = control->getMotor(id);
          if(!motor)
          {
            continue;
          }

          float pos = motor->Get_Position();
          float vel = motor->Get_Velocity();
          float tau = motor->Get_tau();
          double interval = motor->getTimeInterval();
          std::cerr<<"canid is: "<<id<<" pos: "<<pos<<" vel: "<<vel
                  <<" effort: "<<tau<<" time(s): "<<interval<<std::endl;
        }


        const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
        std::this_thread::sleep_until(sleep_till);    
      }

      std::cout <<  std::endl<<"The program exited safely." << std::endl<< std::endl;
  }
  catch (const std::exception& e) {
      std::cerr << "Error: hardware interface exception: " << e.what() << std::endl;
      return 1;
  }

  return 0;
}
