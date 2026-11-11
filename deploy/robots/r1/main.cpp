#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     R1 Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    // Release the robot's built-in motion service so it stops publishing
    // lowcmd, as the unitree_sdk2 R1 low-level examples do. In simulation no
    // service answers, CheckMode times out with an empty name and this falls
    // through.
    {
        auto msc = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
        msc->SetTimeout(5.0f);
        msc->Init();
        std::string form, name;
        while (msc->CheckMode(form, name), !name.empty()) {
            spdlog::info("Releasing motion service '{}' ...", name);
            if (msc->ReleaseMode()) {
                spdlog::warn("Failed to release motion service, retrying.");
            }
            sleep(5);
        }
    }

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 1;
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }
    
    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R1 + X] to start controlling the robot.\n";
    std::cout << "Or press [RB + A] from FixStand to play the tracking motion.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

