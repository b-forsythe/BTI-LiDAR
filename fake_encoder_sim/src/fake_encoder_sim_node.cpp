#include "rclcpp/rclcpp.hpp"
#include "fake_encoder_sim/msg/fake_encoder.hpp"
#include <iostream>


class FakeEncoderSimulator : public rclcpp::Node {
public:
    FakeEncoderSimulator() : Node("fake_encoder_simulator") {
        publisher_ = this->create_publisher<fake_encoder_sim::msg::FakeEncoder>("fake_encoder_data", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FakeEncoderSimulator::publishEncoderData, this));
        
        ticks_ = 0;
        distance_ = 0.0;
        linear_velocity_ = 0.1;  // Simulated linear velocity of the robot (adjust as needed)
        direction_ = 1;  // Direction: 1 for forward, -1 for backward
    }

private:
    void publishEncoderData() {
        auto message = fake_encoder_sim::msg::FakeEncoder();
        
        // Simulate encoder data based on the robot's apparent non-movement
        // The robot is 'moving' forward and backward to simulate a stationary state
        ticks_ += direction_;  // Simulated ticks, alternating between 1 and -1
        distance_ += direction_ * linear_velocity_;  // Simulated distance

        message.ticks = ticks_;
        message.distance = distance_;
        publisher_->publish(message);

        // Change direction after a certain number of ticks to simulate 'non-movement'
        if (ticks_ % 10 == 0) {  // Change direction every 10 ticks
            direction_ *= -1;  // Reverse direction
        }
        
        
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<fake_encoder_sim::msg::FakeEncoder>::SharedPtr publisher_;
    
    int ticks_;
    double distance_;
    double linear_velocity_;
    int direction_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeEncoderSimulator>();
    std::cout << "ALIVE!";
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

