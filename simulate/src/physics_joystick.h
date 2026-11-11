#pragma once

#include <iostream>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include "joystick/joystick.h"
#include <memory>
#include <atomic>
#include <chrono>


class XBoxJoystick : public unitree::common::UnitreeJoystick
{
public:
    XBoxJoystick(std::string device, int bits = 15)
	: unitree::common::UnitreeJoystick()
	{
		js_ = std::make_unique<Joystick>(device);
		if(!js_->isFound()) {
			std::cout << "Error: Joystick open failed." << std::endl;
			exit(1);
		}
        max_value_ = 1 << (bits - 1);
	}

    void update() override
    {
        js_->getState();
        back(js_->button_[6]);
        start(js_->button_[7]);
        LB(js_->button_[4]);
        RB(js_->button_[5]);
        A(js_->button_[0]);
        B(js_->button_[1]); 
        X(js_->button_[2]);
        Y(js_->button_[3]);
        up(js_->axis_[7] < 0);
        down(js_->axis_[7] > 0);
        left(js_->axis_[6] < 0);
        right(js_->axis_[6] > 0);
        LT(js_->axis_[2] > 0);
        RT(js_->axis_[5] > 0);
        lx(double(js_->axis_[0]) / max_value_);
        ly(-double(js_->axis_[1]) / max_value_);
        rx(double(js_->axis_[3]) / max_value_);
        ry(-double(js_->axis_[4]) / max_value_);
    }
private:
	std::unique_ptr<Joystick> js_;
	int max_value_;
};


class SwitchJoystick : public unitree::common::UnitreeJoystick
{
public:
    SwitchJoystick(std::string device, int bits = 15)
	: unitree::common::UnitreeJoystick()
	{
		js_ = std::make_unique<Joystick>(device);
		if(!js_->isFound()) {
			std::cout << "Error: Joystick open failed." << std::endl;
			exit(1);
		}
        max_value_ = 1 << (bits - 1);
	}

    void update() override
    {
        js_->getState();
        back(js_->button_[10]);
        start(js_->button_[11]);
        LB(js_->button_[6]);
        RB(js_->button_[7]);
        A(js_->button_[0]);
        B(js_->button_[1]); 
        X(js_->button_[3]);
        Y(js_->button_[4]);
        up(js_->axis_[7] < 0);
        down(js_->axis_[7] > 0);
        left(js_->axis_[6] < 0);
        right(js_->axis_[6] > 0);
        LT(js_->axis_[5] > 0);
        RT(js_->axis_[4] > 0);
        lx(double(js_->axis_[0]) / max_value_);
        ly(-double(js_->axis_[1]) / max_value_);
        rx(double(js_->axis_[2]) / max_value_);
        ry(-double(js_->axis_[3]) / max_value_);
    }
private:
	std::unique_ptr<Joystick> js_;
	int max_value_;
};

// Stands in for a gamepad when none is available (for example under WSL):
// keys pressed in the MuJoCo window play the button combinations the deploy
// controllers listen for. LT is an axis that the SDK smooths, so it is held
// for a moment before the second button goes down, as a person would.
class KeyboardJoystick : public unitree::common::UnitreeJoystick
{
public:
    enum Combo { NONE, LT_UP, RB_A, LT_B };

    void press(Combo combo)
    {
        combo_ = combo;
        t0_ = std::chrono::steady_clock::now().time_since_epoch().count();
    }

    void update() override
    {
        using namespace std::chrono;
        const auto t0 = steady_clock::time_point(steady_clock::duration(t0_.load()));
        const double t = duration<double>(steady_clock::now() - t0).count();
        const Combo combo = t < 0.6 ? combo_.load() : NONE;
        const bool second = t > 0.3;
        LT((combo == LT_UP || combo == LT_B) ? 1.0 : 0.0);
        up(combo == LT_UP && second);
        B(combo == LT_B && second);
        RB(combo == RB_A);
        A(combo == RB_A);
    }

private:
    std::atomic<Combo> combo_{NONE};
    std::atomic<std::chrono::steady_clock::rep> t0_{0};
};

inline std::shared_ptr<KeyboardJoystick> keyboard_joystick = nullptr;
